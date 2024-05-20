import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription([
        # nuscenes
        launch.actions.DeclareLaunchArgument(
            name='dataset_path',
            default_value='~/Data/nuscenes/Full_dataset_v1.0/trainval/',
            description='nuscenes dataset path'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dataset_ver',
            default_value='v1.0-trainval',
            description='nuscenes dataset version'
        ),

        launch_ros.actions.Node(
            package='nuscenes_player',
            namespace='nuscenes_player', 
            executable='convertor',
            name='nuscenes_player_1',
            parameters=[{
                '~NUSCENES_DIR': launch.substitutions.LaunchConfiguration('dataset_path'),
                '~NUSCENES_VER': launch.substitutions.LaunchConfiguration('dataset_path')
            }]
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()