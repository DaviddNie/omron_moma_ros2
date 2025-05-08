import rclpy
from rclpy.node import Node
from om_aiv_msg.srv import AmrCmd
import time
import random
import string
from tm_msgs.srv import WirelessInfo


WAITAFTER_COMMAND = "waitAfter"
WAITCOMPLETE_COMMAND = "waitComplete"
WAIT_CMDS = {WAITAFTER_COMMAND, WAITCOMPLETE_COMMAND}
QUERYJOB_COMMAND = "queryjob"
EXIT_COMMAND = "exit"
AMR = "AMR"
TM = "TM"
WAIT = "wait"

class DemoRoutine(Node):
	def __init__(self):
		super().__init__('demo_routine')
		self.client = self.create_client(AmrCmd, '/amr_cmd')
		self.wirelessClient = self.create_client(WirelessInfo, '/wireless_info')

		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Amr_Arcl Service not available, waiting again...')

		while not self.wirelessClient.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('wirelessClient Service not available, waiting again...')
		
		self.get_logger().info('DEMO ready!')

	def send_nonblocking_request_amr(self,command,param1):
		# Create the request message (modify as necessary)
		request = AmrCmd.Request()
		request.command = command
		request.param1 = param1
		self.get_logger().info('Sending request to /amr_cmd service')
		
		future = self.client.call_async(request)
		rclpy.spin_until_future_complete(self, future)
		
		if future.result() is not None:
			self.get_logger().info(f'Response: {future.result()}')
		else:
			self.get_logger().error('Failed to call service')

	def process_request(self, type_, cmd, param1="", param2=""):
		if type_ == TM:
			response = self.send_blocking_request_tm(cmd, param1, param2)
			# time.sleep(10)
			return response

		if type_ == WAIT:
			time.sleep(float(cmd))
			return "Great Success~"

		if cmd in WAIT_CMDS:
			return self.wait_till_status(cmd, param1)

		return self.send_blocking_request_amr(cmd, param1, param2)
	
	def wait_till_status(self, targetStatus, jobId):
		while True:
			response = self.send_blocking_request_amr(QUERYJOB_COMMAND, jobId)

			self.get_logger().info(f"[wait_till_status] Status check: {response.status}")

			if targetStatus == WAITAFTER_COMMAND:
				if "After" in response.status:
					self.get_logger().info("Found 'After' status. Proceeding...")
					break
			elif targetStatus == WAITCOMPLETE_COMMAND:
				if "Completed" in response.status:
					self.get_logger().info("Found 'Completed' status. Proceeding...")
					break
			else:
				break

			time.sleep(1)  # prevent spamming the service

		# The timout is proven to be essential, as it takes time for the listen node to be ready after the state transition
		# time.sleep(5)
		return "continue"

	def send_blocking_request_amr(self, command, param1="", param2=""):
		# Create the request message
		request = AmrCmd.Request()
		request.command = command
		request.param1 = param1
		request.param2 = param2
		self.get_logger().info(f'Sending AMR request: {command} {param1} {param2}')

		future = self.client.call_async(request)

		while rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			
			if future.done():
				try:
					response = future.result()
					self.get_logger().info(f'Response: {response}')
					return response
				except Exception as e:
					self.get_logger().error(f'Service call failed: {e}')
					return None
				
	def send_blocking_request_tm(self, command, param1="", param2=""):
		request = WirelessInfo.Request()
		request.command = command
		request.param1 = param1

		self.get_logger().info(f'Sending TM request: {command} {param1} {param2}')

		future = self.wirelessClient.call_async(request)

		while rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			
			if future.done():
				try:
					response = future.result()
					self.get_logger().info(f'Response: {response}')
					return response
				except Exception as e:
					self.get_logger().error(f'Service call failed: {e}')
					return None

	def run_demo(self):

		token = ''.join(random.choices(string.ascii_letters + string.digits, k=6))

		job_1_str = "job_1_" + token

		commands = [
			(AMR,"say", "starting"),
			(TM, "movearm", "home"),
			(AMR,"say", "moving forward"),
			(AMR,"move", "500"),
			(AMR,"say", "moving backward"),
			(AMR,"move", "-100"),
			(AMR,"say", "setheading"),
			(AMR,"setHeading", "180"),
			(AMR,"say", "move arm"),
			(TM, "movearm", "grip1"),
			(WAIT, "7"),
			(AMR,"say", "gripper engaging"),
			(TM, "grip"),
			(WAIT, "3"),
			(AMR,"say", "move arm"),
			(TM, "movearm", "release1"),
			(AMR,"say", "moving forward"),
			(AMR,"move", "300"),
			(WAIT, "7"),
			(AMR,"say", "release in 3 seconds"),
			(WAIT, "0.4"),
			(AMR,"say", "3"),
			(WAIT, "0.4"),
			(AMR,"say", "2"),
			(WAIT, "0.4"),
			(AMR,"say", "1"),
			(WAIT, "0.4"),
			(AMR,"say", "releasing"),
			(WAIT, "0.2"),
			(TM, "release"),
			(TM, "movearm", "home"),
			(AMR,"say", "finished"),
		]
				
		for command in commands:
			response = self.process_request(*command)
			if response is None:
				self.get_logger().error("Aborting demo due to failed command")
				break

def main(args=None):
	rclpy.init(args=args)

	# modbus = Modbus.ModbusClass()

	node = DemoRoutine()
	# time.sleep(10000)
	node.run_demo()
	# node = Node("david")

	# try:
	# 	# modbus.start_program()
	# 	# modbus.stop_program()
	# except Exception as e:
	# 	node.get_logger().error(f"Modbus failed: {str(e)}")
	# finally:
	# 	rclpy.shutdown()

if __name__ == '__main__':
	main()

# LEGACY

		# commands = [
		# 	(AMR,"say", "starting"),
		# 	(AMR,"say", "No need to go to listen node because it's always on"),
		# 	# (TM, "listen", "True"),
		# 	(AMR,"say", "look at me doing both at the same time"),
		# 	(TM, "movearm", "home"),
		# 	(AMR,"say", "moving forward"),
		# 	(AMR,"move", "500"),
		# 	(AMR,"say", "moving backward"),
		# 	(AMR,"move", "-100"),
		# 	(AMR,"say", "setheading"),
		# 	(AMR,"setHeading", "180"),
		# 	# (AMR,"say", "queuing tm goal"),
		# 	# (AMR,"queueGoal", "TM_Goal1", job_1_str),
		# 	# (AMR,"say", "going to tm goal"),
		# 	# (AMR,"go"),

		# 	# Note that waiting for the "After" status is nolonger needed, as wireless package now block until TM is in listen node
		# 	# (AMR, WAITAFTER_COMMAND, job_1_str),
		# 	# (AMR,"say", "about to switch control to TM"),
		# 	(AMR,"say", "move arm"),
		# 	(TM, "movearm", "dwp1"),
		# 	# (WAIT, "5"),
		# 	# (TM, "exit"),
		# 	# (AMR,"say", "switching back to A M R"),
		# 	# (AMR, WAITCOMPLETE_COMMAND, job_1_str),
		# 	# (AMR,"say", "about to switch control to AMR"),
		# 	(AMR,"say", "moving backward again"),
		# 	(AMR,"move", "-500"),
		# 	# (AMR,"say", "about to switch control to TM"),
		# 	# (TM, "listen", "True"),
		# 	(AMR,"say", "move arm"),
		# 	(TM, "movearm", "home"),
		# 	# (WAIT, "5"),
		# 	# (TM, "exit"),
		# 	(AMR,"say", "moving forward again"),
		# 	(AMR,"move", "200"),
		# 	(AMR,"say", "finished"),
		# ]