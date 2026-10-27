import os
import tempfile

from ament_index_python.packages import get_package_share_directory
import yaml

def base_controllers_file():
    return os.path.join(get_package_share_directory('ur_robot_driver'), 'config', 'ur_controllers.yaml')

def overlay_file(name):
    return os.path.join(get_package_share_directory('robot_table_control'), 'config', name)

def deep_merge(base, overlay):
    merged = dict(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged

def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}

def merge_files(base_path, overlay_paths):
    merged = load_yaml(base_path)
    for path in overlay_paths:
        merged = deep_merge(merged, load_yaml(path))
    return merged

def write_controllers_file(overlay_names, name='robot_table_controllers'):
    merged = merge_files(base_controllers_file(), [overlay_file(n) for n in overlay_names])
    fd, path = tempfile.mkstemp(prefix=name + '_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(merged, f, sort_keys=False)
    return path
