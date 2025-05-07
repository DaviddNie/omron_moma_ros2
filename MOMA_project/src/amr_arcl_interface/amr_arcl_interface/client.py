import rclpy
from rclpy.node import Node
from om_aiv_msg.srv import AmrCmd

class AmrArclClient(Node):
    def __init__(self):
        super().__init__('amr_arcl_client')
        self.client = self.create_client(AmrCmd, '/amr_cmd')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Amr_Arcl Service not available, waiting again...')
        
        self.get_logger().info('Amr_Arcl Service available!')

    def send_request(self,parts):

        request = AmrCmd.Request()
        request.command = parts[0] if len(parts) > 0 else ""
        request.param1 = parts[1] if len(parts) > 1 else ""
        request.param2 = parts[2] if len(parts) > 2 else ""
        self.get_logger().info('Sending request to /amr_cmd service')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Response: {future.result()}')
        else:
            self.get_logger().error('Failed to call service')

def main(args=None):
    rclpy.init(args=args)
    node = AmrArclClient()

    try:
        while True:
            # Send request continuously until Enter is pressed
            user_input = input("Enter {command, param1(optional), param2(optional)} to send to the Amr Client (Press Enter to exit):")

            # Split input by commas and strip each part
            parts = [p.strip() for p in user_input.split(",")]

            node.send_request(parts) 
    except ValueError:
        print("Invalid input format! Please use: command,param1,param2")
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
