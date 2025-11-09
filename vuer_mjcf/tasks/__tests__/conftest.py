"""
Pytest configuration for vuer_mjcf task tests.

This conftest ensures that all tests run from the project root directory,
which is necessary because:
1. MuJoCo resolves asset paths relative to the current working directory
2. Task modules generate XML with relative asset paths like "assets/ball_sorting_toy/model.obj"
3. Tests need CWD = project_root so MuJoCo can find the assets/ directory
"""

import os
import pytest
from pathlib import Path


@pytest.fixture(scope="session", autouse=True)
def change_test_dir():
    """Ensure all tests run from project root directory.

    This fixture runs once at the start of the test session and changes
    the working directory to the project root. This ensures that MuJoCo
    can resolve asset paths correctly when loading XML from make_schema().

    Without this, tests may fail when run together via pytest if the
    working directory is not the project root.
    """
    # Navigate from __tests__/ -> tasks/ -> vuer_mjcf/ -> project root
    project_root = Path(__file__).parent.parent.parent.parent

    # Store original directory in case we need it
    original_dir = os.getcwd()

    # Change to project root
    os.chdir(project_root)

    # Yield control back to pytest to run tests
    yield

    # Optional: restore original directory after all tests complete
    # (Not strictly necessary, but good practice)
    os.chdir(original_dir)
