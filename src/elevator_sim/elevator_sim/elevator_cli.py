"""Send a command to an elevator (operator / fault-injection tool).

    ros2 run elevator_sim elevator_cmd e1 block      # simulate an obstruction in the doorway
    ros2 run elevator_sim elevator_cmd e1 clear
    ros2 run elevator_sim elevator_cmd e2 emergency
    ros2 run elevator_sim elevator_cmd e2 reset
    ros2 run elevator_sim elevator_cmd e1 open | close
"""
import sys

import rclpy

from elevator_interfaces.srv import ElevatorCommand

R = ElevatorCommand.Request
COMMANDS = {'close': R.CLOSE_DOORS, 'open': R.OPEN_DOORS, 'block': R.BLOCK_DOOR, 'clear': R.CLEAR_DOOR,
            'emergency': R.EMERGENCY, 'reset': R.RESET}


def main(argv=None):
    args = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:]
    if len(args) != 2 or args[1] not in COMMANDS:
        print(__doc__)
        sys.exit(2)
    eid, cmd = args
    rclpy.init()
    node = rclpy.create_node('elevator_cmd')
    cli = node.create_client(ElevatorCommand, f'/{eid}/command')
    if not cli.wait_for_service(timeout_sec=5.0):
        print(f'/{eid}/command not available')
        sys.exit(1)
    fut = cli.call_async(ElevatorCommand.Request(command=COMMANDS[cmd], requester='operator'))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    r = fut.result()
    print(f'{eid} {cmd}: accepted={r.accepted} {r.message}' if r else 'no response')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if r and r.accepted else 1)


if __name__ == '__main__':
    main()
