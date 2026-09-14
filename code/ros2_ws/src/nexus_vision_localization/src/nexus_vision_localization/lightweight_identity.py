"""Small token-only cross/temporal attention for designated-instance identity.

This is deliberately not a whole-image ViT.  It consumes at most a few dozen
already-computed local descriptors from a reference ROI and a candidate ROI.
The projection is identity-initialised, so the module is useful before a
distilled checkpoint exists and can later load a checkpoint without changing
the tracking protocol.
"""

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np


def _tokens(features, maximum_tokens):
    descriptors = np.asarray(features.descriptors, dtype=np.float32)
    scores = np.asarray(features.scores, dtype=np.float32)
    if descriptors.ndim != 2 or descriptors.shape[1] != 256 or scores.shape != (len(descriptors),):
        raise ValueError("identity attention requires SuperPoint 256D descriptors")
    order = np.argsort(-scores, kind="stable")[:maximum_tokens]
    return descriptors[order]


@dataclass(frozen=True)
class IdentityAttentionResult:
    cross_score: float
    temporal_score: float
    token_count: int

    @property
    def score(self):
        return .7 * self.cross_score + .3 * self.temporal_score


class LightweightIdentityTransformer:
    """One cross-attention block plus a bounded temporal memory.

    A distilled student can supply 256xD q/k/v matrices in an NPZ checkpoint.
    Until then D=256 identity matrices keep the computation deterministic and
    avoid treating random weights as an identity decision.
    """
    def __init__(self, checkpoint=None, *, maximum_tokens=32, maximum_history=8, temperature=.12):
        if not isinstance(maximum_tokens, int) or not 4 <= maximum_tokens <= 128 or not isinstance(maximum_history, int) or not 1 <= maximum_history <= 32:
            raise ValueError("invalid identity transformer token/history budget")
        if not np.isfinite(temperature) or not 0 < temperature <= 2:
            raise ValueError("invalid identity transformer temperature")
        self.maximum_tokens, self.maximum_history, self.temperature = maximum_tokens, maximum_history, float(temperature)
        self.query = self.key = self.value = np.eye(256, dtype=np.float32)
        self.checkpoint = None
        if checkpoint:
            self._load(checkpoint)

    def _load(self, checkpoint):
        path = Path(checkpoint)
        with np.load(path, allow_pickle=False) as values:
            matrices = [values[name].astype(np.float32) for name in ("query", "key", "value")]
            if any(matrix.ndim != 2 or matrix.shape[0] != 256 or not np.all(np.isfinite(matrix)) for matrix in matrices):
                raise ValueError("identity transformer checkpoint has invalid q/k/v matrices")
            if len({matrix.shape[1] for matrix in matrices}) != 1 or not 16 <= matrices[0].shape[1] <= 256:
                raise ValueError("identity transformer checkpoint dimension is invalid")
            self.query, self.key, self.value = matrices
            self.checkpoint = str(path)

    def embed(self, features):
        tokens = _tokens(features, self.maximum_tokens)
        if not len(tokens):
            return np.zeros(self.value.shape[1], np.float32), 0
        value = tokens @ self.value
        value /= np.maximum(np.linalg.norm(value, axis=1, keepdims=True), 1e-8)
        embedding = value.mean(axis=0)
        embedding /= max(1e-8, np.linalg.norm(embedding))
        return embedding.astype(np.float32), len(tokens)

    def kv_cache(self, features):
        """Encode one ROI once for reusable temporal attention."""
        tokens = _tokens(features, self.maximum_tokens)
        dimension = self.key.shape[1]
        if not len(tokens):
            return {"key": np.zeros((0, dimension), np.float32), "value": np.zeros((0, dimension), np.float32)}
        key, value = tokens @ self.key, tokens @ self.value
        key /= np.maximum(np.linalg.norm(key, axis=1, keepdims=True), 1e-8)
        value /= np.maximum(np.linalg.norm(value, axis=1, keepdims=True), 1e-8)
        return {"key": key.astype(np.float32), "value": value.astype(np.float32)}

    def _cached_temporal_score(self, candidate, temporal_cache):
        cache = [entry for entry in temporal_cache[-self.maximum_history:]
                 if isinstance(entry, dict) and set(entry) == {"key", "value"}]
        if not cache:
            return None
        raw = _tokens(candidate, self.maximum_tokens)
        if not len(raw):
            return 0.
        query = raw @ self.query
        query /= np.maximum(np.linalg.norm(query, axis=1, keepdims=True), 1e-8)
        keys = [np.asarray(entry["key"], dtype=np.float32) for entry in cache]
        values = [np.asarray(entry["value"], dtype=np.float32) for entry in cache]
        if any(key.ndim != 2 or value.ndim != 2 or key.shape != value.shape or key.shape[1] != query.shape[1]
               for key, value in zip(keys, values)):
            return None
        keys, values = np.concatenate(keys), np.concatenate(values)
        if not len(keys):
            return 0.
        logits = query @ keys.T / self.temperature
        logits -= logits.max(axis=1, keepdims=True)
        weights = np.exp(np.clip(logits, -60, 60))
        weights /= np.maximum(weights.sum(axis=1, keepdims=True), 1e-8)
        attended = weights @ values
        projected = raw @ self.value
        attended /= np.maximum(np.linalg.norm(attended, axis=1, keepdims=True), 1e-8)
        projected /= np.maximum(np.linalg.norm(projected, axis=1, keepdims=True), 1e-8)
        return float(np.clip((np.mean(np.sum(attended * projected, axis=1)) + 1.) / 2., 0., 1.))

    def compare(self, reference, candidate, history=(), temporal_cache=()):
        first, second = _tokens(reference, self.maximum_tokens), _tokens(candidate, self.maximum_tokens)
        if not len(first) or not len(second):
            return IdentityAttentionResult(0., 0., 0)
        query, key = first @ self.query, second @ self.key
        query /= np.maximum(np.linalg.norm(query, axis=1, keepdims=True), 1e-8)
        key /= np.maximum(np.linalg.norm(key, axis=1, keepdims=True), 1e-8)
        logits = query @ key.T / self.temperature
        logits -= logits.max(axis=1, keepdims=True)
        attention = np.exp(np.clip(logits, -60, 60))
        attention /= attention.sum(axis=1, keepdims=True)
        attended = attention @ (second @ self.value)
        projected = first @ self.value
        attended /= np.maximum(np.linalg.norm(attended, axis=1, keepdims=True), 1e-8)
        projected /= np.maximum(np.linalg.norm(projected, axis=1, keepdims=True), 1e-8)
        cross = float(np.clip((np.sum(attended * projected, axis=1).mean() + 1.) / 2., 0., 1.))
        cached = self._cached_temporal_score(candidate, temporal_cache)
        candidate_embedding, _ = self.embed(candidate)
        memories = [np.asarray(item, dtype=np.float32) for item in history[-self.maximum_history:] if np.asarray(item).shape == candidate_embedding.shape]
        temporal = (cached if cached is not None else
                    (cross if not memories else float(np.clip((np.mean([np.dot(candidate_embedding, item) for item in memories]) + 1.) / 2., 0., 1.))))
        return IdentityAttentionResult(cross, temporal, min(len(first), len(second)))
