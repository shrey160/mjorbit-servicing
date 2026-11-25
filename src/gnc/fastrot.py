"""Fast single-rotation conversions, bit-identical to scipy's Rotation (wall-time work, 19 Nov).

scipy 1.17's Rotation wraps its Cython backend in array-API dispatch (_promote, xp_result_type,
atleast_nd, ...), which costs ~20-60 us per call; the P6.3 loop makes millions of these calls (the
corner fit's residual, the filters' exp map). These functions call the same Cython backend
directly on float64 numpy input, so the numbers are the same bits (checked by `python fastrot.py`).
Single rotations only (rotvec (3,), matrix (3, 3)); anything else goes through Rotation.

patch_scipy() also gives Rotation.from_rotvec / from_matrix / from_quat a fast path for one rotation
given as a numpy array, list or tuple (default options; everything else goes to scipy's own code):
it builds the Rotation exactly as scipy's _from_raw_quat does, from the same backend output, so the
frozen P3 / P5 code (inline Rotation calls) gets the speed without being edited.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform._rotation import array_namespace, select_backend

_cy = select_backend(array_namespace(np.zeros(3)), cython_compatible=True)
assert _cy.__name__.endswith("_rotation_cy"), _cy


def _f64(x):
    return np.asarray(x, dtype=np.float64)


def rotvec_to_matrix(v):
    """Rotation.from_rotvec(v).as_matrix() for one rotation vector."""
    return _cy.as_matrix(_cy.from_rotvec(_f64(v).reshape(1, 3), degrees=False))[0]


def matrix_to_rotvec(M):
    """Rotation.from_matrix(M).as_rotvec() for one 3x3 matrix."""
    return _cy.as_rotvec(_cy.from_matrix(_f64(M).reshape(1, 3, 3), assume_valid=False), degrees=False)[0]


def matrix_to_matrix(M):
    """Rotation.from_matrix(M).as_matrix(): the nearest rotation to M (orthonormalised)."""
    return _cy.as_matrix(_cy.from_matrix(_f64(M).reshape(1, 3, 3), assume_valid=False))[0]


def quat_to_matrix(q):
    """Rotation.from_quat(q).as_matrix() for one scalar-last quaternion (normalised as scipy does)."""
    return _cy.as_matrix(_cy.from_quat(_f64(q).reshape(1, 4), normalize=True, copy=True, scalar_first=False))[0]


_XP = array_namespace(np.zeros(3))
_ORIG = {}


def _raw(quat):
    r = Rotation.__new__(Rotation)  # as Rotation._from_raw_quat for a single numpy quaternion
    r._single, r._quat, r._xp, r._backend = True, quat, _XP, _cy
    return r


def _one(x, shape):
    if isinstance(x, (np.ndarray, list, tuple)):
        a = np.asarray(x)
        if a.shape == shape and a.dtype.kind in "fiu":
            return a.astype(np.float64, copy=False)
    return None


def _from_rotvec(rotvec, degrees=False):
    a = None if degrees else _one(rotvec, (3,))
    if a is None:
        return _ORIG["from_rotvec"](rotvec, degrees=degrees)
    return _raw(_cy.from_rotvec(a.reshape(1, 3), degrees=False))


def _from_matrix(matrix, assume_valid=False):
    a = _one(matrix, (3, 3))
    if a is None:
        return _ORIG["from_matrix"](matrix, assume_valid=assume_valid)
    return _raw(_cy.from_matrix(a.reshape(1, 3, 3), assume_valid=assume_valid))


def _from_quat(quat, *, scalar_first=False):
    a = _one(quat, (4,))
    if a is None:
        return _ORIG["from_quat"](quat, scalar_first=scalar_first)
    return _raw(_cy.from_quat(a.reshape(1, 4), normalize=True, copy=True, scalar_first=scalar_first))


def patch_scipy():
    """Install the fast paths on scipy's Rotation (idempotent)."""
    if _ORIG:
        return
    for nm, f in (("from_rotvec", _from_rotvec), ("from_matrix", _from_matrix), ("from_quat", _from_quat)):
        _ORIG[nm] = Rotation.__dict__[nm].__func__
        setattr(Rotation, nm, staticmethod(f))


def _same(r1, r2):
    o = [np.array_equal(r1.as_quat(), r2.as_quat()), np.array_equal(r1.as_matrix(), r2.as_matrix()),
         np.array_equal(r1.as_rotvec(), r2.as_rotvec()), np.array_equal(r1.inv().as_matrix(), r2.inv().as_matrix()),
         np.array_equal(r1.apply([0.3, -1.0, 2.0]), r2.apply([0.3, -1.0, 2.0])),
         np.array_equal(r1.as_euler("xyz"), r2.as_euler("xyz")), r1.single == r2.single, len(r1.as_quat().shape) == len(r2.as_quat().shape)]
    return all(o)


def _check_patch(rng, n=50000):
    patch_scipy()
    bad = 0
    for i in range(n):
        v = rng.normal(size=3) * 10.0 ** rng.uniform(-9, 0.6)
        kind = i % 5
        if kind == 1:
            v = v.tolist()
        elif kind == 2:
            v = v.astype(np.float32)
        elif kind == 3:
            v = np.round(v).astype(int)
        r_new, r_old = Rotation.from_rotvec(v), _ORIG["from_rotvec"](v)
        bad += not _same(r_new, r_old)
        M = r_old.as_matrix() + (rng.normal(size=(3, 3)) * 1e-7 if i % 2 else 0.0)
        bad += not _same(Rotation.from_matrix(M), _ORIG["from_matrix"](M))
        q = rng.normal(size=4)
        bad += not _same(Rotation.from_quat(q), _ORIG["from_quat"](q))
        bad += not _same(Rotation.from_quat(q, scalar_first=True), _ORIG["from_quat"](q, scalar_first=True))
    # fallbacks: stacks and options still go to scipy
    V = rng.normal(size=(5, 3))
    bad += not np.array_equal(Rotation.from_rotvec(V).as_matrix(), _ORIG["from_rotvec"](V).as_matrix())
    bad += not np.array_equal(Rotation.from_rotvec(V[0], degrees=True).as_matrix(), _ORIG["from_rotvec"](V[0], degrees=True).as_matrix())
    print(f"patch: {4 * n} cases, {bad} differ")


def _check(n=200000):
    rng = np.random.default_rng(0)
    bad = 0
    for i in range(n):
        s = 10.0 ** rng.uniform(-9, 0.6)
        v = rng.normal(size=3) * s
        if i % 7 == 0:
            v[rng.integers(3)] = 0.0
        a = Rotation.from_rotvec(v).as_matrix()
        b = rotvec_to_matrix(v)
        M = a @ Rotation.from_rotvec(rng.normal(size=3) * 1e-6).as_matrix() if i % 3 else a
        c, d = Rotation.from_matrix(M).as_rotvec(), matrix_to_rotvec(M)
        e, f = Rotation.from_matrix(M).as_matrix(), matrix_to_matrix(M)
        qq = rng.normal(size=4) * 10.0 ** rng.uniform(-3, 3)
        g, h = Rotation.from_quat(qq).as_matrix(), quat_to_matrix(qq)
        bad += not (np.array_equal(a, b) and np.array_equal(c, d) and np.array_equal(e, f) and np.array_equal(g, h))
    lst = [0.1, -0.2, 0.3]  # list input
    bad += not np.array_equal(Rotation.from_rotvec(lst).as_matrix(), rotvec_to_matrix(lst))
    print(f"{n} cases, {bad} differ")
    import timeit
    v = rng.normal(size=3)
    M = rotvec_to_matrix(v)
    for nm, f1, f2 in (("from_rotvec.as_matrix", lambda: Rotation.from_rotvec(v).as_matrix(), lambda: rotvec_to_matrix(v)),
                       ("from_matrix.as_rotvec", lambda: Rotation.from_matrix(M).as_rotvec(), lambda: matrix_to_rotvec(M))):
        t1, t2 = timeit.timeit(f1, number=20000) / 2e-2, timeit.timeit(f2, number=20000) / 2e-2
        print(f"{nm}: scipy {t1:.1f} us, fast {t2:.1f} us")
    t1 = timeit.timeit(lambda: Rotation.from_rotvec(v).as_matrix(), number=20000) / 2e-2
    _check_patch(rng)
    t2 = timeit.timeit(lambda: Rotation.from_rotvec(v).as_matrix(), number=20000) / 2e-2
    print(f"Rotation.from_rotvec(v).as_matrix(): unpatched {t1:.1f} us, patched {t2:.1f} us")


if __name__ == "__main__":
    _check()
