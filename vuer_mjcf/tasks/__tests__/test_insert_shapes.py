"""Tests for insert_shapes task."""

import pytest


def test_insert_shapes_xml_generation():
    """Test that insert_shapes generates valid XML."""
    from vuer_mjcf.tasks.insert_shapes import make_schema

    xml_str = make_schema()

    assert xml_str is not None, "make_schema() returned None"
    assert len(xml_str) > 0, "Generated XML is empty"
    assert '<mujoco' in xml_str, "XML doesn't contain <mujoco tag"


def test_insert_shapes_mujoco_loading():
    """Test that insert_shapes XML can be loaded by MuJoCo."""
    mujoco = pytest.importorskip("mujoco", reason="MuJoCo not available")

    from vuer_mjcf.tasks.insert_shapes import make_schema

    xml_str = make_schema()
    model = mujoco.MjModel.from_xml_string(xml_str)

    assert model is not None, "MuJoCo model is None"
    assert model.nbody > 0, f"Model has no bodies (nbody={model.nbody})"
