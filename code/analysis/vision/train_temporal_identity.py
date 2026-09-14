"""Train the ROI cross/temporal-attention Q/K/V student from labelled sequences.

The input NPZ is deliberately explicit: ``reference`` and ``candidate`` are
Nx256 descriptors, ``history`` is NxTx256 descriptors of preceding frames,
``history_mask`` is NxT, and ``same`` is a binary N-vector.  It prevents a
pair-only Fisher projection from being presented as temporal training.
"""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequences", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--dimension", type=int, default=32)
    parser.add_argument("--batch", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--label-provenance", required=True)
    args = parser.parse_args()
    if (not args.sequences.is_file() or args.output.exists() or args.epochs < 1 or not 16 <= args.dimension <= 256
            or args.batch < 1 or not np.isfinite(args.learning_rate) or args.learning_rate <= 0 or not args.label_provenance.strip()):
        raise ValueError("temporal identity training arguments are invalid")
    archive = np.load(args.sequences, allow_pickle=False)
    reference, candidate, history, mask, same = (archive[name] for name in
                                                   ("reference", "candidate", "history", "history_mask", "same"))
    if (reference.ndim != 2 or candidate.shape != reference.shape or reference.shape[1] != 256
            or history.ndim != 3 or history.shape[:1] != reference.shape[:1] or history.shape[2] != 256
            or mask.shape != history.shape[:2] or same.shape != (len(reference),)
            or not all(np.all(np.isfinite(value)) for value in (reference, candidate, history))
            or not np.all(np.isin(mask, [0, 1])) or not np.all(np.isin(same, [0, 1]))):
        raise ValueError("sequence archive must contain finite aligned labelled descriptor sequences")
    import torch
    import torch.nn.functional as functional
    device = torch.device(args.device)
    tensors = [torch.as_tensor(value, dtype=torch.float32, device=device)
               for value in (reference, candidate, history, mask, same)]
    reference, candidate, history, mask, same = tensors
    query = torch.nn.Parameter(torch.eye(256, args.dimension, device=device))
    key = torch.nn.Parameter(torch.eye(256, args.dimension, device=device))
    value = torch.nn.Parameter(torch.eye(256, args.dimension, device=device))
    optimizer = torch.optim.AdamW((query, key, value), lr=args.learning_rate, weight_decay=1e-5)
    generator = torch.Generator(device=device).manual_seed(20260906)
    for _ in range(args.epochs):
        for indices in torch.randperm(len(reference), generator=generator, device=device).split(args.batch):
            ref, current, past, valid, labels = (item[indices] for item in (reference, candidate, history, mask, same))
            q = functional.normalize(ref @ query, dim=-1)
            k = functional.normalize(current @ key, dim=-1)
            v = functional.normalize(current @ value, dim=-1)
            cross = (q * k).sum(-1)
            past_k = functional.normalize(past @ key, dim=-1)
            past_v = functional.normalize(past @ value, dim=-1)
            logits = torch.einsum("nd,ntd->nt", q, past_k) / .12
            logits = logits.masked_fill(valid == 0, -1e4)
            weights = torch.softmax(logits, dim=-1) * valid
            weights = weights / torch.clamp(weights.sum(-1, keepdim=True), min=1e-8)
            attended = functional.normalize(torch.einsum("nt,ntd->nd", weights, past_v), dim=-1)
            temporal = (v * attended).sum(-1)
            has_history = valid.any(dim=-1)
            score = .7 * cross + .3 * torch.where(has_history, temporal, cross)
            loss = functional.binary_cross_entropy_with_logits(4. * score, labels)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            optimizer.step()
    args.output.mkdir(parents=True)
    path = args.output / "identity_temporal_student.npz"
    np.savez_compressed(path, query=query.detach().cpu().numpy(), key=key.detach().cpu().numpy(), value=value.detach().cpu().numpy(),
                        metadata=json.dumps({"training": "labelled_cross_temporal_attention", "sequence_count": int(len(same)),
                                             "history_length": int(history.shape[1]), "label_provenance": args.label_provenance}))
    print(json.dumps({"weights": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                      "sequences": int(len(same))}, ensure_ascii=False))


if __name__ == "__main__":
    main()
