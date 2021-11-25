from os import environ
import os.path
import yaml

from ..find_default_asset_path import find_robot_path, find_world_path
from ..deep_update import deep_update

# This function collects simulation configuration from the following sources
# (the later sources override previous ones):
#   1. robot_directory/config.yaml
#   2. world_directory/config.yaml
#   3. default sim config
#   4. /sim/config.yaml
#   5. ${cwd}/config.yaml
#   6. environment variables
# The structure is described in TODO.html
#
def collect_config():
    config = {}

    # Load default config
    config = deep_update(config, load_default_config())

    # Load the config specified by user
    config = deep_update(config, load_yaml_config('/sim/config.yaml'))
    config = deep_update(config, load_yaml_config('./config.yaml'))
    config = deep_update(config, load_env_config())

    # Load config specified by robot
    robot_dir = os.path.dirname(find_robot_path(config['robot']))
    robot_config_path = os.path.join(robot_dir, 'config.yaml')
    config = deep_update(config, load_yaml_config(robot_config_path))

    # Load config specified by world
    world_dir = os.path.dirname(find_world_path(config['world']))
    world_config_path = os.path.join(world_dir, 'config.yaml')
    config = deep_update(config, load_yaml_config(world_config_path))

    # Load the config specified by user AGAIN (To override everything if needed)
    config = deep_update(config, load_yaml_config('/sim/config.yaml'))
    config = deep_update(config, load_yaml_config('./config.yaml'))
    config = deep_update(config, load_env_config())

    return config


def load_default_config():
    module_dir = os.path.dirname(os.path.realpath(__file__))
    default_config_path = os.path.join(module_dir, 'default.yaml')
    return load_yaml_config(default_config_path)

def load_yaml_config(filepath):
    if not os.path.exists(filepath):
        return {}
    with open(filepath) as f:
        return yaml.safe_load(f)

def load_env_config():
    config = {}
    if environ.get('ROBOT'):
        config['robot'] = environ['ROBOT']
    if environ.get('WORLD'):
        config['world'] = environ['WORLD']
    if environ.get('START_XYZ'):
        x, y, z = map(float, environ['START_XYZ'].split(','))
        config = deep_update(config, {'start': {'x': x, 'y': y, 'z': z}})
    if environ.get('START_RPY'):
        R, P, Y = map(float, environ['START_RPY'].split(','))
        config = deep_update(config, {'start': {'R': R, 'P': P, 'Y': Y}})
    return config
