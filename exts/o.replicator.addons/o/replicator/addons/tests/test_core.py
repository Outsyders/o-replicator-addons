import doctest
import os
import unittest
from pathlib import Path

import carb
import numpy as np

import omni.kit
import omni.replicator.core as rep
import omni.graph.core as og

# from omni.replicator.core import utils
# from omni.replicator.core.distribution import choice
import o.replicator.addons as addons

from pxr import Gf, Sdf, Semantics, Usd, UsdGeom, UsdShade

manager = omni.kit.app.get_app().get_extension_manager()
ext_id = manager.get_enabled_extension_id("o.replicator.addons")
ext_path = manager.get_extension_path(ext_id)
MDL_FOLDER = Path(ext_path).joinpath("mdl").as_posix()
TEST_DATA_DIR = Path(os.path.dirname(os.path.realpath(__file__))).joinpath("data")


def get_prim_at_path(path: str):
    stage = omni.usd.get_context().get_stage()
    return stage.GetPrimAtPath(str(path))


class TestCore(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        rep.set_global_seed(1234)

    async def tearDown(self):
        await omni.usd.get_context().new_stage_async()

    async def test_database_state(self):
        uniform = rep.distribution.uniform([0, 0, 0], [1, 1, 1])
        choice = rep.distribution.choice([[0, 0, 0], [1, 1, 1]])
        normal = rep.distribution.normal([0, 0, 0], [1, 1, 1])
        log_uniform = rep.distribution.log_uniform([0, 0, 0], [1, 1, 1])

        uniform_node = og.Controller().node(str(uniform.get_output("prims")[0]))
        choice_node = og.Controller().node(str(choice.get_output("prims")[0]))
        normal_node = og.Controller().node(str(normal.get_output("prims")[0]))
        log_uniform_node = og.Controller().node(str(log_uniform.get_output("prims")[0]))

        await omni.kit.app.get_app().next_update_async()

        uniform_state = addons.core._get_database_state(addons.core._get_node_db(uniform_node))
        choice_state = addons.core._get_database_state(addons.core._get_node_db(choice_node))
        normal_state = addons.core._get_database_state(addons.core._get_node_db(normal_node))
        log_uniform_state = addons.core._get_database_state(addons.core._get_node_db(log_uniform_node))

        assert isinstance(uniform_state, dict), f"{uniform_state} is not a dict"
        assert isinstance(choice_state, dict), f"{choice_state} is not a dict"
        assert isinstance(normal_state, dict), f"{normal_state} is not a dict"
        assert isinstance(log_uniform_state, dict), f"{log_uniform_state} is not a dict"

        for i in range(10):
            await rep.orchestrator.step_async()

        addons.core._set_database_state(addons.core._get_node_db(uniform_node), uniform_state)
        addons.core._set_database_state(addons.core._get_node_db(choice_node), choice_state)
        addons.core._set_database_state(addons.core._get_node_db(normal_node), normal_state)
        addons.core._set_database_state(addons.core._get_node_db(log_uniform_node), log_uniform_state)

        new_uniform_state = addons.core._get_database_state(addons.core._get_node_db(uniform_node))
        new_choice_state = addons.core._get_database_state(addons.core._get_node_db(choice_node))
        new_normal_state = addons.core._get_database_state(addons.core._get_node_db(normal_node))
        new_log_uniform_state = addons.core._get_database_state(addons.core._get_node_db(log_uniform_node))

        assert uniform_state == new_uniform_state, f"{uniform_state} != {new_uniform_state}"
        assert choice_state == new_choice_state, f"{choice_state} != {new_choice_state}"
        assert normal_state == new_normal_state, f"{normal_state} != {new_normal_state}"
        assert log_uniform_state == new_log_uniform_state, f"{log_uniform_state} != {new_log_uniform_state}"

    async def test_replicator_state(self):
        from omni.isaac.core.utils.prims import get_prim_attribute_value

        with rep.new_layer():
            sphere = rep.create.sphere()
            cube = rep.create.cube()
            torus = rep.create.torus()
            cone = rep.create.cone()
            with rep.trigger.on_frame():
                with sphere:
                    rep.modify.pose(
                        rotation=rep.distribution.uniform([0, 0, 0], [360, 360, 360]),
                        position=rep.distribution.uniform([0, 0, 0], [10, 10, 10]),
                        size=rep.distribution.uniform([1, 1, 1], [10, 10, 10]),
                    )

                with cube:
                    rep.modify.pose(
                        position=rep.distribution.choice([[i] * 3 for i in range(10)]),
                        size=rep.distribution.choice([[i] * 3 for i in range(5)]),
                    )

                with torus:
                    rep.modify.pose(
                        position=rep.distribution.normal([0, 0, 0], [1, 1, 1]),
                        size=rep.distribution.normal([0, 0, 0], [1, 1, 1]),
                    )

                with cone:
                    rep.modify.pose(
                        position=rep.distribution.log_uniform([1, 1, 1], [10, 10, 10]),
                        size=rep.distribution.log_uniform([1, 1, 1], [10, 10, 10]),
                    )

        await omni.usd.get_context().next_update_async()

        states = addons.core.get_replicator_state()

        sphere_path = sphere.get_output("prims")[0]
        cube_path = cube.get_output("prims")[0]
        torus_path = torus.get_output("prims")[0]
        cone_path = cone.get_output("prims")[0]

        prim_paths = [sphere_path, cube_path, torus_path, cone_path]

        values = []
        for i in range(10):
            await rep.orchestrator.step_async()

            # Get the position and size of the sphere
            vals = []
            for prim_path in prim_paths:
                position = get_prim_attribute_value(prim_path, "xformOp:translate", True)
                rotation = get_prim_attribute_value(prim_path, "xformOp:rotateXYZ", True)
                size = get_prim_attribute_value(prim_path, "xformOp:scale", True)
                vals.append((prim_path, position, rotation, size))

            values.append((i, vals))

        # Reset the state of RNG
        addons.core.set_replicator_state(states)

        # Check that the states were set correctly
        _new_states = addons.core.get_replicator_state()
        for (k, v), (_k, _v) in zip(states.items(), _new_states.items()):
            if not v or not _v:
                continue
            assert v == _v, f"{v} != {_v} for {k}"

        for node in addons.core._all_replicator_nodes():
            db = addons.core._get_node_db(node)

            if not db:
                continue

            if not hasattr(db, "shared_state"):
                continue

            state = db.shared_state

            if not hasattr(state, "rng"):
                continue

            is_seed_valid = db.inputs.seed is not None
            is_seed_changed = state.rng is None or db.inputs.seed != state.rng.seed

            assert not (
                is_seed_valid and is_seed_changed
            ), f"{is_seed_valid} != {is_seed_changed} for {node.get_prim_path()}"

            break

        for i, vals in values:
            await rep.orchestrator.step_async()

            for prim_path, old_position, old_rotation, old_size in vals:
                # Get the position and size of the sphere
                new_position = get_prim_attribute_value(prim_path, "xformOp:translate", True)
                new_rotation = get_prim_attribute_value(prim_path, "xformOp:rotateXYZ", True)
                new_size = get_prim_attribute_value(prim_path, "xformOp:scale", True)

                # Check if the position and size are the same
                assert np.allclose(
                    new_position, old_position
                ), f"{new_position} != {old_position} for position in {prim_path}"
                assert np.allclose(
                    new_rotation, old_rotation
                ), f"{new_rotation} != {old_rotation} for rotation in {prim_path}"
                assert np.allclose(new_size, old_size), f"{new_size} != {old_size} for size in {prim_path}"
