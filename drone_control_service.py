import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)
from battery_monitor_interfaces.srv import BatteryService
import asyncio

class MinimalService(Node):

    def __init__(self):
        super().__init__('drone_control_service')
        self.srv = self.create_service(BatteryService, 'drone_battery_control', self.control_drone_callback)

    def control_drone_callback(self, request, response):
        try:
            if request.action == "LAND":
                asyncio.run(self.land_now())
            elif request.action == "RTB":
                asyncio.run(self.rtb())
            response.success = True
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error handling request: {e}")

        self.get_logger().info(f"Incoming {request}")
        return response

    async def rtb(self):
        home_position = PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        drone = System()
        await drone.connect(system_address="udp://:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connected to drone!")
                break

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break


        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed \
                        with error code: {error._result.result}")
            print("-- Disarming")
            await drone.action.disarm()
            return

        # RETURN TO BASE ACTION
        await drone.offboard.set_position_ned(
            home_position)
        await asyncio.sleep(5)

    async def land_now(self):
        drone = System()
        await drone.connect(system_address="udp://:14540")
        async for state in drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("Connected to drone!")
                break

        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("Global position estimate OK")
                break

        # LAND ACTION
        await drone.action.land()
        await asyncio.sleep(5)


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()