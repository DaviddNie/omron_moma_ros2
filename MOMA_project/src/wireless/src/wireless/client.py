import rclpy
from rclpy.node import Node
from tm_msgs.srv import WriteItem,WirelessInfo
from std_msgs.msg import String

EXIT_COMMAND = "exit"
GRIP_COMMAND = "grip"
RELEASE_COMMAND = "release"

SINGLE_PARAMETER_COMMANDS = [EXIT_COMMAND, GRIP_COMMAND, RELEASE_COMMAND]

class WirelessInfoClient(Node):
    def __init__(self):
        super().__init__('wireless_info_client')
        self.client = self.create_client(WirelessInfo, '/wireless_info')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available!')

    def send_request(self, input_):
        input_ = input_.strip()  # Clean the input first
        command, param1 = "", ""
        
        # Check for single-parameter commands first
        for cmd in SINGLE_PARAMETER_COMMANDS:
            if input_.startswith(cmd):
                # If exact match or match followed by whitespace
                if len(input_) == len(cmd) or input_[len(cmd):].isspace():
                    command = cmd
                    param1 = ""
                    break
        
        # If not a single-parameter command, try to split
        if not command:
            try:
                parts = input_.split(",", 1)  # Split on first comma only
                command = parts[0].strip()
                param1 = parts[1].strip() if len(parts) > 1 else ""
            except Exception as e:
                self.get_logger().error(f'Failed to parse input: {e}')
                return

        # Create and send the request
        try:
            request = WirelessInfo.Request()
            request.command = command
            request.param1 = param1
            self.get_logger().info(f'Sending request - Command: {command}, Param: {param1}')
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info(f'Response: {future.result()}')
            else:
                self.get_logger().error('Service call failed: No response received')
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = WirelessInfoClient()

    try:
        while True:
            # Send request continuously until Enter is pressed
            user_input = input("Enter (command, param1) to send to the action server (Press Enter to exit):")
            node.send_request(user_input)

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
