from os import environ
from os.path import dirname, join, realpath

def find_robot_path(name):
    return find_asset('robots', name, 'robot.urdf')

def find_world_path(name):
    return find_asset('worlds', name, 'world.xml')

def find_asset(category, name, basename):
    default_assets_dir = environ.get('ASSETS_PATH') or '/sim/assets/default'
    category_dir = join(default_assets_dir, category)
    asset_dir = join(category_dir, name)
    main_file_dir = join(asset_dir, basename)
    return main_file_dir
