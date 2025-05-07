import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from om_aiv_msg.srv import AmrCmd, ArclApi
import asyncio

class AmrArclServer(Node):
    def __init__(self):
        super().__init__('amr_arcl_server')
    
        # Separate groups for isolation
        # Reentrant groups allow callbacks to run in parallel, and allow multiple threads from the same group to run concurrently
        self.arcl_group = ReentrantCallbackGroup() 
        self.cmd_group = ReentrantCallbackGroup()
        
        self.arcl_client = self.create_client(
            ArclApi, 'arcl_api_service', 
            callback_group=self.arcl_group
        )
        
        self.srv = self.create_service(
            AmrCmd, '/amr_cmd',
            self.handle_amr_command,
            callback_group=self.cmd_group
        )
        
        while not self.arcl_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for ARCL service...')
        
        self.get_logger().info('AMR ARCL SERVER READY')

    async def handle_amr_command_async(self, request):
        """Async implementation without timeout"""
        arcl_req = ArclApi.Request()
        arcl_req.line_identifier = ""

        if request.command == 'move':
            arcl_req.command = f'dotask move {request.param1} 400 200 50 30 0 0 0 False False False'
            arcl_req.line_identifier = "Completed doing task " + arcl_req.command[7:] + "\r\n"
        elif request.command == 'setHeading':
            arcl_req.command = f'dotask setHeading {request.param1}'
            arcl_req.line_identifier = "Completed doing task " + arcl_req.command[7:] + "\r\n"
        elif request.command == 'say':
            arcl_req.command = f'dotask say \"{request.param1}\"'
            arcl_req.line_identifier = "Completed doing task " + arcl_req.command[7:] + "\r\n"
        elif request.command == 'queueGoal':
            arcl_req.command = f'queuepickup {request.param1} 10 {request.param2}'
        elif request.command == 'queryjob':
            arcl_req.command = f'queueQuery jobId {request.param1}'
            arcl_req.line_identifier = "EndQueueQuery"
        elif request.command == 'go':
            arcl_req.command = f'go'
        elif request.command == 'stay':
            arcl_req.command = f'stay'
        else:
            return "Invalid command"
        
        # VERY IMPORTANT
        # Only returns when "completed" response is received
        # see socket_driver.py, queue_command() for more details

        future = self.arcl_client.call_async(arcl_req)
        
        try:
            # Simply await the future without timeout
            await future
            return future.result().response
        except Exception as e:
            return f"ARCL error: {str(e)}"

    def handle_amr_command(self, request, response):
        """Wrapper that runs async code in event loop"""
        try:
            # Create or get existing event loop
            try:
                loop = asyncio.get_event_loop()
            except RuntimeError:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
            
            # Run async handler
            result = loop.run_until_complete(
                self.handle_amr_command_async(request)
            )
            response.status = result
        except Exception as e:
            response.status = f"Server error: {str(e)}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = AmrArclServer()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Shutting down...")
        finally:
            node.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()