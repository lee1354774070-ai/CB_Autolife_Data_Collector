"""Load the source under test without requiring a completed colcon install.

The integration staging directory used during development is flat, while the
final ROS package places Python modules below
``openarmx_teleop_vr_306_v4/``.  The loader supports both layouts
and gives relative imports (for example ``.teleop_core``) a package context.
"""

import importlib.util
from pathlib import Path
import sys
import types


TEST_DIRECTORY = Path(__file__).resolve().parent
PACKAGE_ROOT = TEST_DIRECTORY.parent
SOURCE_CANDIDATES = (
    PACKAGE_ROOT,
    PACKAGE_ROOT / 'openarmx_teleop_vr_306_v4',
)
SYNTHETIC_PACKAGE = '_autolife_independent_arm_control_subject'


def source_directory():
    for candidate in SOURCE_CANDIDATES:
        if (candidate / 'teleop_core.py').is_file():
            return candidate
    raise RuntimeError('could not locate independent arm-control Python sources')


def source_path(module_name):
    path = source_directory() / f'{module_name}.py'
    if not path.is_file():
        raise RuntimeError(f'could not locate source module {module_name!r}')
    return path


def load_subject(module_name):
    """Import one source module directly from the checked-out package."""
    package = sys.modules.get(SYNTHETIC_PACKAGE)
    if package is None:
        package = types.ModuleType(SYNTHETIC_PACKAGE)
        package.__path__ = [str(source_directory())]
        package.__package__ = SYNTHETIC_PACKAGE
        sys.modules[SYNTHETIC_PACKAGE] = package

    qualified_name = f'{SYNTHETIC_PACKAGE}.{module_name}'
    loaded = sys.modules.get(qualified_name)
    if loaded is not None:
        return loaded

    spec = importlib.util.spec_from_file_location(
        qualified_name,
        source_path(module_name),
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f'could not create import spec for {module_name!r}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[qualified_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(qualified_name, None)
        raise
    return module

