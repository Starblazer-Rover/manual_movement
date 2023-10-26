import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    launch_actions = launch.actions.DeclareLaunchArgument(
                        'node_prefix',
                        default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
                        description='Manual Movement')
    
    controller = launch_ros.actions.Node(
        package="manual_movement", executable="controller", output="screen",
        name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'controller'],
    )

    rover_movement = launch_ros.actions.Node(
            package='manual_movement', executable='rover_movement', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'rover_movement'],
            prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True
            )

    return launch.LaunchDescription([
        launch_actions,
        controller,
        rover_movement
    ])
        
    
