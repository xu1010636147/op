"""Safe accessors for BYD-only params.

Some of the BYD tuning params are not registered in common/params_keys.h on
this branch (they come from the dragonpilot port). Params.get_bool/get_int
raise on unknown keys, so fall back to a caller-supplied default instead of
crashing the car interface.
"""

from openpilot.common.params import Params

_params: Params | None = None


def _get_params() -> Params:
  global _params
  if _params is None:
    _params = Params()
  return _params


def get_bool(key: str, default: bool = False) -> bool:
  try:
    return bool(_get_params().get_bool(key))
  except Exception:
    return default


def get_int(key: str, default: int = 0) -> int:
  try:
    return int(_get_params().get_int(key))
  except Exception:
    return default
