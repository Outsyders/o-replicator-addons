from ._impl import *

from .scripts import core
from .scripts import rep
from .scripts import utils
from .scripts import modify

from inspect import getmembers, isfunction, isclass, ismodule

import sys

sys.modules["o.replicator.addons.modify"] = modify


def predicate(x):
    from omni.replicator.core.utils import ReplicatorWrapper

    return (isfunction(x) or isclass(x) or type(x) == ReplicatorWrapper) and x.__module__ == modify.__name__


def __monkeypatch_rep(module, module_name, module_path="omni.replicator.core.scripts"):
    for name, obj in getmembers(module, predicate):
        # # Check for collisions
        # if hasattr(sys.modules[f"{module_path}.{module_name}"], name):
        #     print(f"Collision detected for `{name}` in {module_path}.{module_name}.{name}")
        #     continue

        setattr(sys.modules[f"{module_path}.{module_name}"], name, obj)
        print(f"Patched `{name}` to {module_path}.{module_name}.{name}")


__monkeypatch_rep(modify, "modify")
