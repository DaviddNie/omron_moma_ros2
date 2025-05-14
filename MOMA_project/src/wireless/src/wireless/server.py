import math
import rclpy
from rclpy.node import Node
from tm_msgs.srv import WriteItem,WirelessInfo,SetPositions,SendScript, MovementRequest
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import asyncio
from tm_msgs.msg import *
import time
from .moveit_movement_handler import MoveitMovementHandler

LIGHT_COMMAND = "light"
MOVE2JOINTPOS = "movejoint"
EXIT_COMMAND = "exit"
LISTENNING_COMMAND = "listen"
GRIP_COMMAND = "grip"
RELEASE_COMMAND = "release"
MOVE2POINT_COMMAND = "move2point"

MOVEIT_MOVE2POINT_COMMAND = "moveitmove2point"

class WriteItemServer(Node):

	def __init__(self):
		super().__init__('write_item_client')

		self.moveit_movement_handler = MoveitMovementHandler(self)

		# Reentrant groups allow parallel callbacks
		self.write_item_group = ReentrantCallbackGroup()
		self.set_positions_group = ReentrantCallbackGroup()
		self.send_script_group = ReentrantCallbackGroup()
		self.service_group = ReentrantCallbackGroup()
		self.movement_group = ReentrantCallbackGroup()

		self.is_listening = False
		# self.connected = False

		# Clients with specific callback groups
		self.cli = self.create_client(WriteItem, '/write_item', callback_group=self.write_item_group)
		self.cli_2 = self.create_client(SetPositions, '/set_positions', callback_group=self.set_positions_group)
		self.cli_3 = self.create_client(SendScript, '/send_script', callback_group=self.send_script_group)
		
		self.moveit_srv = self.create_service(
            MovementRequest, '/movement_request',
            self.handle_movement_request,
            callback_group=self.movement_group
        )
		
		# Subscriptions
		self.subscription = self.create_subscription(SctResponse, 'sct_response', self.listener_callback, 10)

		self.error = False
		self.error_code = ""
		self.error_msg = ""
		self.error_subscription = self.create_subscription(
			FeedbackState,
			'feedback_states',
			self.feedback_callback,
			10)

		self.wait_for_services()

		# Service with reentrant callback group
		self.srv = self.create_service(
			WirelessInfo, '/wireless_info', self.handle_wireless_info,
			callback_group=self.service_group
		)
		# Define the mapping of string names to position arrays
		self.joint_config_map = {
			# dwp1 stands for david_waypoint_1
			"dwp1": [0.0, 0.0, 90.0, 0.0, 90.0, 0.0],
			"home": [-90.0, 0.0, 90.0, 0.0, 90.0, 0.0],
			"grip1": [-64.576, 11.229, 125.22, -44.482, 92.837, 25.893],
			"release1": [-123.047, 38.727, 37.145, 14.696, 91.281, -3.0],
			"camera_home": [-90.0, -11.09, 89.32, 11.75, 90.01, -0.0049],
			# Add more positions as needed
		}

		self.position_map = {
			"home": [-0.182, -0.664, 0.613, 180.0, 0.0, 0.0],
			"movex": [-0.082, -0.664, 0.613, 180.0, 0.0, 0.0],
			"movey": [-0.182, -0.564, 0.613, 180.0, 0.0, 0.0],
			"movez": [-0.182, -0.764, 0.713, 180.0, 0.0, 0.0],
			"trial": [-0.175, -0.638, 0.3, 180.0, 0.0, 0.0],
			# Add more positions as needed
		}

	"""Assigns the error message when an exception occursUses the service client for IO control with modbus to open the end effector"""
	def feedback_callback(self, msg):
		self.error_code = msg.error_code
		if msg.robot_error == True:
			self.error_code = msg.error_code
			self.error = True
			self.error_msg = "TM Robot Error."
			self.get_logger().info("error_msg: " + self.error_msg)
		if msg.project_run == False:
			self.error = True
			self.error_msg = "Project is not running!"
			self.get_logger().info("error_msg: " + self.error_msg)
		if msg.e_stop == True:
			self.error = True
			self.error_msg = "Emergency stop activated!"
			self.get_logger().info("error_msg: " + self.error_msg)


	def wait_for_services(self):
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for /write_item service...')
		while not self.cli_2.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for /set_positions service...')
		while not self.cli_3.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for /send_script service...')

	def handle_wireless_info(self, request, response):
		try:
			try:
				loop = asyncio.get_event_loop()
			except RuntimeError:
				loop = asyncio.new_event_loop()
				asyncio.set_event_loop(loop)

			result = loop.run_until_complete(self.handle_wireless_info_async(request))
			response.status = str(result)  # Convert bool to string if WirelessInfo.srv uses string status
		except Exception as e:
			response.status = f"Error: {str(e)}"

		return response

	def listener_callback(self, msg):
		self.get_logger().info("MSGGGGGG is: " + msg.script)
		if (msg.script == 'Listen1'):
			self.is_listening = True
		# if (msg.script == 'OK'):
		# 	self.connected = True

	async def handle_wireless_info_async(self, request):
		# Assign the response based on the incoming request
		if request.command == LIGHT_COMMAND: 
			return await self.light_request(request.param1) 
		if request.command == LISTENNING_COMMAND:
			return await self.listenning_request(request.param1) # True or False
		elif request.command == MOVE2JOINTPOS:
			return await self.move2JointConfig(request.param1)
		elif request.command == MOVE2POINT_COMMAND:
			return await self.move2Point(request.param1)
		elif request.command == GRIP_COMMAND:
			return await self.gripper_engage(True)
		elif request.command == RELEASE_COMMAND:
			return await self.gripper_engage(False)
		elif request.command == EXIT_COMMAND:
			return await self.send_exit()
		else:
			return "Invalid value received. Only 'light', 'movejoint', 'move2point', 'grip', 'release', 'exit' is allowed."
	
	def handle_movement_request(self, request, response):
		try:
			try:
				loop = asyncio.get_event_loop()
			except RuntimeError:
				loop = asyncio.new_event_loop()
				asyncio.set_event_loop(loop)

			result = loop.run_until_complete(self.handle_movement_request_async(request))
			response.status = str(result)  # Convert bool to string if WirelessInfo.srv uses string status
		except Exception as e:
			response.status = f"Error: {str(e)}"

		return response
	
	async def handle_movement_request_async(self, request):
		if request.command == MOVEIT_MOVE2POINT_COMMAND: 
			return await self.moveit_movement_handler.handle_move2point(request.positions) 
	
	async def gripper_engage(self, engage):
		# Please see gripper documentation and TM Expression (modbus_write) for more detail

		# Byte 0: 0x09 = 0x00000101
		# -> bit 1 : activite
		# -> bit 3 : go to position
		# Byte 1: reserved

		# Byte 2: reserved

		# Byte 3: Position
			# 00 -> open
			# between open and close, 255 steps to choose
			# FF -> close

		gripper_pos = 0x09000000

		if engage:
			gripper_pos = 0x090000FF

		req = SendScript.Request()
		req.id = '1'

		# Option 1: Always work
		req.script = 'modbus_write("mrtu_Gripper", 0x0009, "RO", 0x03E8, ' + hex(gripper_pos) + ')'

		# Option 2: Use the preset, in project function -> modbus device, TMFlow Project Flowchart
		# req.script = 'modbus_write("mrtu_Gripper", "preset_gripper_action_request", ' + hex(gripper_pos) + ')'
		future = self.cli_3.call_async(req)

		await future

		return future.result().ok
	
	async def light_request(self, param1):
		req = WriteItem.Request()
		req.id = '1'
		req.item = 'Camera_Light'
		req.value = param1
		future = self.cli.call_async(req)
		await future
		return future.result().ok
	
	async def listenning_request(self, param1):
		req = WriteItem.Request()
		req.id = '1'
		req.item = 'g_listening'
		req.value = param1
		future = self.cli.call_async(req)
		await future

		# make sure TMFLow is in listen by the time it returns

		time.sleep(1)
		return future.result().ok


	async def move2JointConfig(self, param1):
		if param1 not in self.joint_config_map:
			return f"Error: '{param1}' not found in position map."

		req = SetPositions.Request()
		req.motion_type = 1

		req.positions = [math.radians(angle) for angle in self.joint_config_map[param1]]
		# e.g. 
		# req.positions = [-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]

		req.velocity = 0.9
		req.acc_time = 0.05
		req.blend_percentage = 1
		req.fine_goal = False
		future = self.cli_2.call_async(req)
		await future
		return future.result().ok
	
	async def move2Point(self, param1):
		if param1 not in self.position_map:
			return f"Error: '{param1}' not found in position map."

		req = SetPositions.Request()
		req.motion_type = 4

		req.positions = self.position_map[param1]

		req.velocity = 0.9
		req.acc_time = 0.5
		req.blend_percentage = 1
		req.fine_goal = False
		future = self.cli_2.call_async(req)
		await future
		return future.result().ok
	
	def blockUntilListen(self):
		while (not self.is_listening):
			self.get_logger().info("waiting for listen node")
			time.sleep(2)

	async def send_exit(self):
		
		req = SendScript.Request()
		req.id = '1'
		req.script = 'ScriptExit()'
		future = self.cli_3.call_async(req)

		await future

		self.is_listening = False

		return future.result().ok

def main(args=None):
	rclpy.init(args=args)

	
	try:
		node = WriteItemServer()
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

	# write_item_client = WriteItemServer()
	# rclpy.spin(write_item_client)
	# #write_item_client.light_request()
	# #Shutdown after the request
	# write_item_client.destroy_node()
	# rclpy.shutdown()


if __name__ == '__main__':
	main()
