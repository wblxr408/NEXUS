import numpy as np


def gdop(anchors, point):
    anchors = np.asarray(anchors, dtype=float)
    point = np.asarray(point, dtype=float).reshape(3)
    if anchors.ndim != 2 or anchors.shape[1] != 3 or anchors.shape[0] < 4:
        raise ValueError("at least four anchors with XYZ coordinates are required")
    if not np.all(np.isfinite(anchors)) or not np.all(np.isfinite(point)):
        raise ValueError("anchors and point must be finite")
    directions = anchors - point
    lengths = np.linalg.norm(directions, axis=1)
    if np.any(lengths <= 0):
        raise ValueError("evaluation point cannot equal an anchor")
    geometry = np.column_stack((directions / lengths[:, None], np.ones(anchors.shape[0])))
    covariance = np.linalg.pinv(geometry.T @ geometry)
    return float(np.sqrt(np.trace(covariance)))


def layout_summary(anchors, points):
    values = [gdop(anchors, point) for point in points]
    return {"min": float(np.min(values)), "mean": float(np.mean(values)), "max": float(np.max(values)), "samples": len(values)}
