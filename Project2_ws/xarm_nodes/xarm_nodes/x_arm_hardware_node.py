#!/usr/bin/env python3

from urllib import response

import rclpy
from rclpy.node import Node
# from xarm_pickup_interfaces.srv import YourServiceType  # TODO(STUDENTS): Import your service types here.
from xarm_pickup_interfaces.srv import StartSearch 
from xarm_pickup_interfaces.srv import MoveToGoal
from xarm_pickup_interfaces.srv import MoveToSquare
from xarm_pickup_interfaces.srv import ObjectDetect
from xarm_pickup_interfaces.srv import Cancel

try:
    import xarm
except ImportError:
    xarm = None


class XArmHardwareNode(Node):
    def __init__(self):
        super().__init__('x_arm_hardware_node')
        self.arm = None

        self._connect_usb()

        # TODO(STUDENTS): Add your service servers here. Make sure that all services are defined in the xarm_pickup_interfaces package and that you import them at the top of this file.
        # Example:
        # self.create_service(YourServiceType, 'service_name', self.service_callback)

        self.create_service(StartSearch, 'start_search', self.start_search_callback)

        self.create_service(MoveToGoal, 'move_to_goal', self.move_to_goal_callback)

        self.create_service(MoveToSquare, 'move_to_square', self.move_to_square_callback)

        self.create_service(ObjectDetect, 'object_detect', self.object_detect_callback)

        self.create_service(Cancel, 'cancel', self.cancel_callback)

        self.get_logger().info('x_arm_hardware_node is running.')

    def _connect_usb(self):
        if xarm is None:
            self.get_logger().error('xarm Python library not found. Install it before running hardware control.')
            return

        try:
            self.arm = xarm.Controller('USB')
            self.get_logger().info('Connected to xArm over USB.')
        except Exception as exc:
            self.get_logger().error(f'Failed to connect to xArm over USB: {exc}')

    # TODO(STUDENTS): Add your service callback methods here.
    # Suggestions:
    # 1) Validate inputs before sending commands (e.g. are joint angles within limits?).
    # 2) Return clear success/failure info to the caller via the response object.
    #
    # Example:
    # def service_callback(self, request, response):
    #     # ... perform arm action ...
    #     response.success = True
    #     return response

    def start_search_callback(self, request, response):
    # ... perform start search action ...
        response.success = True
        response.status_message = "Search started"
        return response

    def move_to_goal_callback(self, request, response):
        # ... perform move to goal action ...
        response.success = True
        response.status_message = "Move to goal completed!"
        return response

    def move_to_square_callback(self, request, response):
        # ... perform move to square action ...
        response.success = True
        response.status_message = "Moved to square"
        return response

    def object_detect_callback(self, request, response):
        # ... perform object detection action ...
        response.success = True
        response.status_message = "Object detected"
        return response

    def cancel_callback(self, request, response):
        # ... perform cancel action ...
        response.success = True
        response.status_message = "Search cancelled"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = XArmHardwareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
