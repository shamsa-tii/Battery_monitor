import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import BatteryStatus
from mavsdk import System
from mavsdk.offboard import PositionNedYaw
import asyncio
from battery_monitor_interfaces.srv import BatteryService


class BatteryMonitorNode(Node):

    def __init__(self, safe_threshold=50.0, return_to_base_threshold_lower=20.0, return_to_base_threshold_upper=50.0,
                 immediate_return_threshold_lower=10.0, immediate_return_threshold_upper=20.0,
                 emergency_landing_threshold_lower=5.0, emergency_landing_threshold_upper=10.0,
                 critically_low_threshold=5.0):
        super().__init__('battery_monitor_node')
        self.safe_threshold = safe_threshold
        self.return_to_base_threshold_lower = return_to_base_threshold_lower
        self.return_to_base_threshold_upper = return_to_base_threshold_upper
        self.immediate_return_threshold_lower = immediate_return_threshold_lower
        self.immediate_return_threshold_upper = immediate_return_threshold_upper
        self.emergency_landing_threshold_lower = emergency_landing_threshold_lower
        self.emergency_landing_threshold_upper = emergency_landing_threshold_upper
        self.critically_low_threshold = critically_low_threshold
        self.battery_status = None
        self.drone = System()

        # Set base position
        self.home_position = PositionNedYaw(47.3997336, 8.5407014, 0.0, 0.0)

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        self.battery_status_sub = self.create_subscription(BatteryStatus, '/fmu/out/battery_status',
                                                           self.battery_status_callback, qos_profile)

        # Connect to the drone using MAVSDK
        # self.connect_to_drone()
        self.cli = self.create_client(BatteryService, 'drone_battery_control')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # self.create_timer(timer_period in seconds, callback_function)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def send_request(self, action: str):
        request = BatteryService.Request()
        request.action = action
        self.future = self.cli.call_async(request)
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def connect_to_drone(self):
        try:
            self.get_logger().error("Connecting to the drone")
            asyncio.ensure_future(self.connect_drone())
        except Exception as e:
            self.get_logger().error(f"Error connecting to the drone: {e}")

    async def connect_drone(self):
        try:
            await self.drone.connect(system_address="udp://:14540")
            self.get_logger().error("Connected to the drone")
        except Exception as e:
            self.get_logger().error(f"Error connecting to the drone: {e}")

    def battery_status_callback(self, msg: BatteryStatus):
        self.battery_status = msg

    def timer_callback(self):
        if self.battery_status is not None:
            battery_remaining = self.battery_status.remaining * 100 - 20
            print("Remaining Battery:", battery_remaining)

            if battery_remaining < self.critically_low_threshold:
                self.get_logger().error("Battery level is critically low! Execute emergency landing procedure!")
                self.send_request("LAND")
                # self.land()
            elif battery_remaining < self.emergency_landing_threshold_lower:
                self.get_logger().error("Battery level is extremely low! Prepare for emergency landing!")
                action = self.confirm_and_land()
                if action:
                    self.send_request("LAND")
                else:
                    pass
                # self.confirm_and_land()
            elif battery_remaining < self.immediate_return_threshold_lower:
                self.get_logger().error("Critical Battery! Initiating immediate return to base!")
                action = self.confirm_return()
                if action:
                    self.send_request("RTB")
                else:
                    pass
                # self.confirm_return()
            elif self.return_to_base_threshold_lower <= battery_remaining <= self.return_to_base_threshold_upper:
                self.get_logger().info("Battery level between 20 and 50. Initiate return to base protocol for recharge.")
                action = self.confirm_return()
                if action:
                    self.send_request("RTB")
                else:
                    pass
                # self.confirm_return()
            elif battery_remaining > self.safe_threshold:
                self.get_logger().info("Battery level is safe. Continue the mission.")

    def land(self):
        try:
            asyncio.ensure_future(self.land_action())
        except Exception as e:
            self.get_logger().error(f"Error initiating landing action: {e}")

    async def land_action(self):
        try:
            self.get_logger().info("Initiating automatic landing procedure.")
            await self.drone.action.land()
        except Exception as e:
            self.get_logger().error(f"Error initiating landing action: {e}")

    def return_to_base(self):
        try:
            asyncio.ensure_future(self.return_to_base_action())
        except Exception as e:
            self.get_logger().error(f"Error initiating return-to-base action: {e}")

    async def return_to_base_action(self):
        try:
            self.get_logger().info("Initiating return-to-base procedure.")

            setpoint = self.home_position

            await self.drone.offboard.set_position_ned(setpoint)
        except Exception as e:
            self.get_logger().error(f"Error initiating return-to-base action: {e}")

    def confirm_and_land(self):
        user_input = input("Battery level is critically low. Do you want to land the drone? (Type 'y' for yes, 'n' for no): ")
        if user_input.lower() == 'y':
            self.land()
            return True
        else:
            self.get_logger().info("User declined landing.")
            return False

    def confirm_return(self):
        user_input = input("Battery level is low, Do you want to intiate the return to base protocol? (Type 'y' for yes, 'n' for no): ")
        if user_input.lower() == "y":
            # self.return_to_base()
            return True
        else:
            self.get_logger().info("User declined return to base protocol.")
            return False

def main(args=None):
    print('Starting node')
    rclpy.init(args=args)
    battery_monitor_node = BatteryMonitorNode(safe_threshold=50.0,
                                              return_to_base_threshold_lower=20.0, return_to_base_threshold_upper=50.0,
                                              immediate_return_threshold_lower=10.0,
                                              immediate_return_threshold_upper=20.0,)

    rclpy.spin(battery_monitor_node)
    battery_monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
