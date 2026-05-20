from __future__ import annotations

import os
import sys
from pathlib import Path


_BOOTSTRAP_SRC = Path(
    os.getenv(
        "PAPARAZZI_SRC",
        Path(__file__).resolve().parents[3],
    )
)
_BOOTSTRAP_LIB = _BOOTSTRAP_SRC / "sw" / "lib" / "python"
if str(_BOOTSTRAP_LIB) not in sys.path:
    sys.path.append(str(_BOOTSTRAP_LIB))

import pprz_env


def paparazzi_home() -> Path:
    return Path(pprz_env.PAPARAZZI_HOME)


def paparazzi_src() -> Path:
    return Path(pprz_env.PAPARAZZI_SRC)


def ivy_bus() -> str:
    return pprz_env.IVY_BUS


def ensure_pprz_paths() -> None:
    home = paparazzi_home()
    src = paparazzi_src()
    for path in (src / "sw" / "lib" / "python", home / "var" / "lib" / "python"):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.append(path_str)
