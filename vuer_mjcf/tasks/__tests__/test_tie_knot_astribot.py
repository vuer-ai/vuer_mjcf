"""Tests for tie_knot_astribot task."""

import pytest


def test_tie_knot_astribot_xml_generation():
    """Test that tie_knot_astribot generates valid XML."""
    from vuer_mjcf.tasks.tie_knot_astribot import make_schema

    xml_str = make_schema()

    assert xml_str is not None, "make_schema() returned None"
    assert len(xml_str) > 0, "Generated XML is empty"
    assert '<mujoco' in xml_str, "XML doesn't contain <mujoco tag"


def test_tie_knot_astribot_mujoco_loading():
    """Test that tie_knot_astribot XML can be loaded by MuJoCo."""
    mujoco = pytest.importorskip("mujoco", reason="MuJoCo not available")

    from vuer_mjcf.tasks.tie_knot_astribot import make_schema

    xml_str = make_schema()
    model = mujoco.MjModel.from_xml_string(xml_str)

    assert model is not None, "MuJoCo model is None"
    assert model.nbody > 0, f"Model has no bodies (nbody={model.nbody})"
