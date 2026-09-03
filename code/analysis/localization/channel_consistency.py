"""Channel consistency for the robust chain (design section 8.2).

Three visual/UWB channels are run in parallel and compared before anything is
published.  Adversarial patches and attention-shift degradations can corrupt the
dense correspondence channel while the support-plane and control-point channels
stay valid, so disagreement is the detector: when the channels do not agree the
frame is marked INVALID instead of publishing the least-suspicious pose.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import numpy as np


@dataclass(frozen=True)
class ChannelPose:
    """One independent estimate of the same target position in ``map``."""

    name: str
    translation_m: np.ndarray
    available: bool = True


@dataclass(frozen=True)
class ConsistencyReport:
    disagreement_m: float
    pair_distances_m: dict[str, float]
    available_channels: tuple[str, ...]
    outlier_channel: str | None


def channel_consistency(channels: list[ChannelPose]) -> ConsistencyReport:
    """Largest pairwise distance plus the channel that drives it.

    With fewer than two available channels the disagreement is not measurable
    and is reported as ``inf``: an unmeasurable cross-check must not read as a
    passing cross-check.
    """
    usable = [channel for channel in channels if channel.available and np.all(np.isfinite(channel.translation_m))]
    if len(usable) < 2:
        return ConsistencyReport(float("inf"), {}, tuple(channel.name for channel in usable), None)
    distances = {f"{first.name}|{second.name}": float(np.linalg.norm(np.asarray(first.translation_m, dtype=float) - np.asarray(second.translation_m, dtype=float)))
                 for first, second in combinations(usable, 2)}
    worst_pair = max(distances, key=distances.get)
    outlier = None
    if len(usable) >= 3:
        # The channel whose median distance to the others is largest is the one
        # to blame; with only two channels neither can be singled out.
        medians = {channel.name: float(np.median([np.linalg.norm(np.asarray(channel.translation_m, dtype=float) - np.asarray(other.translation_m, dtype=float))
                                                  for other in usable if other is not channel]))
                   for channel in usable}
        outlier = max(medians, key=medians.get)
    return ConsistencyReport(float(distances[worst_pair]), distances, tuple(channel.name for channel in usable), outlier)
