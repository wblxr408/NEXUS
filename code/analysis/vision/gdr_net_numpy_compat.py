"""NumPy 2 compatibility shim for the unmodified 2021 GDR-Net checkout.

Call :func:`apply_numpy2_compat` before importing upstream GDR-Net in the
dedicated Windows environment.  It alters only NumPy's removed legacy aliases
for the current Python process; the Git submodule remains unchanged.
"""

from __future__ import annotations

import numpy as np


def apply_numpy2_compat() -> bool:
    """Restore NumPy aliases used by upstream GDR-Net when running NumPy 2.x.

    Returns whether the process needed a compatibility change.  NumPy 1.x is
    left untouched.
    """
    if "float" in np.__dict__:
        return False
    np.float = np.float64
    np.int = np.int_
    np.bool = np.bool_
    np.complex = np.complex128
    if "maximum_sctype" not in np.__dict__:
        def maximum_sctype(value):
            return np.promote_types(np.dtype(value), np.dtype(np.longdouble)).type
        np.maximum_sctype = maximum_sctype
    return True


__all__ = ["apply_numpy2_compat"]
