import launch
from .generate_launch_description import generate_launch_description

def main():
    launch_description = generate_launch_description()
    launch_service = launch.LaunchService()
    launch_service.include_launch_description(launch_description)
    launch_service.run()

if __name__== '__main__':
    main()
