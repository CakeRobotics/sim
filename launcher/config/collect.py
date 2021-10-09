from os import environ
import os.path
import yaml

# This function collects simulation configuration from the following sources
# (the later sources override previous ones):
#   1. default sim config
#   2. /sim/config.yaml
#   3. ${cwd}/config.yaml
#   4. environment variables
# The structure is described in TODO.html
#
def collect_config():
    config = {}
    config.update(load_default_config())
    config.update(load_yaml_config('/sim/config.yaml'))
    config.update(load_yaml_config('./config.yaml'))
    config.update(load_env_config())
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
        config.update({'start_point': {'x': x, 'y': y, 'z': z}})
    if environ.get('START_RPY'):
        R, P, Y = map(float, environ['START_RPY'].split(','))
        config.update({'start_point': {'R': R, 'P': P, 'Y': Y}})
    return config
