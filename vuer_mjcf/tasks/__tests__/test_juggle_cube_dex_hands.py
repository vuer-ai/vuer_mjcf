"""Tests for juggle_cube_dex_hands task."""

import pytest


def test_juggle_cube_dex_hands_xml_generation():
    """Test that juggle_cube_dex_hands generates valid XML."""
    from vuer_mjcf.tasks.juggle_cube_dex_hands import make_schema

    xml_str = make_schema()

    assert xml_str is not None, "make_schema() returned None"
    assert len(xml_str) > 0, "Generated XML is empty"
    assert '<mujoco' in xml_str, "XML doesn't contain <mujoco tag"


def test_juggle_cube_dex_hands_mujoco_loading():
    """Test that juggle_cube_dex_hands XML can be loaded by MuJoCo."""
    mujoco = pytest.importorskip("mujoco", reason="MuJoCo not available")

    from vuer_mjcf.tasks.juggle_cube_dex_hands import make_schema

    xml_str = make_schema()
    model = mujoco.MjModel.from_xml_string(xml_str)

    assert model is not None, "MuJoCo model is None"
    assert model.nbody > 0, f"Model has no bodies (nbody={model.nbody})"
