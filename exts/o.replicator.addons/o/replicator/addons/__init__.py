from ._impl import *

from inspect import getmembers, isfunction, isclass, ismodule

# Get all the functions from the modify module
from .scripts import modify
from .scripts import core

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
