"""Tests for dual_ur5_pick_block task."""

import pytest


def test_dual_ur5_pick_block_module_imports():
    """Test that dual_ur5_pick_block module can be imported."""
    import vuer_mjcf.tasks.dual_ur5_pick_block

    # This is a real robot task without make_schema()
    assert vuer_mjcf.tasks.dual_ur5_pick_block is not None
