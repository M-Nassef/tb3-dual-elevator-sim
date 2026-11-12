"""Run the missions of both robots, in parallel or one after the other.

    ros2 launch elevator_sim missions.launch.py mode:=parallel \
        tb3_1_floor:=3 tb3_1_goal:=room_b tb3_2_floor:=2 tb3_2_goal:=room_c

mode:=sequential starts tb3_2's mission only after tb3_1's mission process has exited.
robots:=tb3_1 (or tb3_2) runs a single robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *_, **__):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('parallel', 'sequential'):
        raise RuntimeError(f'mode must be parallel or sequential, got "{mode}"')
    robots = [r.strip() for r in LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    procs = []
    for r in robots:
        floor = LaunchConfiguration(f'{r}_floor').perform(context)
        goal = LaunchConfiguration(f'{r}_goal').perform(context)
        procs.append(Node(package='elevator_sim', executable='mission', name=f'{r}_mission_runner',
                          output='screen', emulate_tty=True,
                          arguments=['--robot', r, '--floor', floor, '--goal', goal, '--mode', mode]))
    if not procs:
        raise RuntimeError('no robots selected')
    actions = []
    if mode == 'parallel':
        actions += procs
    else:
        actions.append(procs[0])
        for prev, nxt in zip(procs, procs[1:]):
            actions.append(RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[nxt])))
    # shut the launch down after the last mission ends
    remaining = {id(p) for p in procs}

    def done(event, ctx, p=None):
        remaining.discard(id(p))
        return [Shutdown(reason='all missions finished')] if not remaining else []

    for p in procs:
        actions.append(RegisterEventHandler(OnProcessExit(target_action=p,
                                                          on_exit=lambda e, c, p=p: done(e, c, p))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='parallel', description='parallel | sequential'),
        DeclareLaunchArgument('robots', default_value='tb3_1,tb3_2', description='comma-separated robot list (order = sequence)'),
        DeclareLaunchArgument('tb3_1_floor', default_value='3'),
        DeclareLaunchArgument('tb3_1_goal', default_value='room_b'),
        DeclareLaunchArgument('tb3_2_floor', default_value='2'),
        DeclareLaunchArgument('tb3_2_goal', default_value='room_c'),
        OpaqueFunction(function=setup),
    ])
