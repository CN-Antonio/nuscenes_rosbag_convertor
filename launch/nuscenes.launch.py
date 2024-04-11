import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        # nuscenes
        launch.actions.DeclareLaunchArgument(
            name='config',
            default_value='config/flashocc.yaml',       # To Modify
            description='flashocc config yaml file'
        ),
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()