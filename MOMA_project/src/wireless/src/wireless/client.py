import rclpy
from rclpy.node import Node
from tm_msgs.srv import WriteItem, WirelessInfo, MovementRequest

EXIT_COMMAND = "exit"
GRIP_COMMAND = "grip"
RELEASE_COMMAND = "release"
MOVEIT_MOVE2POINT_COMMAND = "moveitmove2point"

SINGLE_PARAMETER_COMMANDS = [EXIT_COMMAND, GRIP_COMMAND, RELEASE_COMMAND]

class WirelessInfoClient(Node):
    def __init__(self):
        super().__init__('wireless_info_client')

        # Declare clients
        self.wireless_info_client = self.create_client(WirelessInfo, '/wireless_info')

        # Wait for services
        self.wait_for_services()

    def wait_for_services(self):
        self.get_logger().info('Waiting for services...')
        while not (self.wireless_info_client.wait_for_service(timeout_sec=1.0)):
            self.get_logger().info('Service(s) not available, retrying...')
        self.get_logger().info('All services available!')

    def send_request(self, input_):
        input_ = input_.strip()
        if not input_:
            self.get_logger().info('Empty input. Skipping...')
            return

        parts = [part.strip() for part in input_.split(",")]
        if not parts:
            self.get_logger().error("Invalid input format.")
            return

        command = parts[0].lower()
        args = parts[1:]

        # Route request
        if command in SINGLE_PARAMETER_COMMANDS:
            self.handle_wireless_info_request(command, "")
        else:
            param1 = args[0] if args else ""
            self.handle_wireless_info_request(command, param1)

    def handle_wireless_info_request(self, command, param1):
        request = WirelessInfo.Request()
        request.command = command
        request.param1 = param1

        self.get_logger().info(f'Sending WirelessInfo request: {command}, {param1}')
        future = self.wireless_info_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().error('WirelessInfo service call failed.')

    def handle_movement_request(self, command, args):
        if not (1 <= len(args) <= 6):
            self.get_logger().error(f'MovementRequest expects 3 to 6 numeric parameters (got {len(args)})')
            return

        try:
            positions = [float(x) for x in args]
        except ValueError as e:
            self.get_logger().error(f'Invalid parameter(s), must be numeric: {e}')
            return

        request = MovementRequest.Request()
        request.command = command
        request.positions = positions

        self.get_logger().info(f'Sending MovementRequest: {command}, {positions}')
        future = self.movement_request_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().error('MovementRequest service call failed.')


def main(args=None):
    rclpy.init(args=args)
    node = WirelessInfoClient()

    try:
        while True:
            user_input = input("Enter command (cmd, param1): ")
            if not user_input.strip():
                break
            node.send_request(user_input)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
