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

# ========== ROBOT POSITIONS ==========
gripper_closed_count = 709
gripper_open_count   = 145

POSITIONS = {
    0: [145, 500, 500, 500, 505, 500], # Home Position
    1: [145, 646, 46, 646, 342, 632],
    2: [145, 545, 114, 818, 464, 510],
    3: [145, 396, 157, 826, 455, 342],
    4: [145, 605, 211, 784, 417, 592],
    5: [145, 497, 197, 792, 425, 473],
    6: [145, 423, 206, 784, 417, 381],
    7: [145, 567, 296, 763, 380, 573],
    8: [145, 524, 267, 764, 384, 487],
    9: [145, 437, 274, 737, 363, 400],
}

#  pos: [J1, J2, J3, J4, J5, J6]
#  pos: [Gripper, Wrist1, Wrist2, Elbow, Shoulder, Base]

POSITION_DROP = [709, 492, 849, 121, 484, 485]
# ========== END ROBOT POSITIONS ==========


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

    # Implemented Service callbacks for each hardware interaction. 

    def start_search_callback(self, request, response):
        """Start search - move arm to home position and open gripper."""
        if self.arm is None:
            response.is_active = False
            response.status_message = "Arm not connected"
            return response
        
        try:
            # Move to home position
            self.arm.setPosition(POSITIONS[0], wait=True)
            response.is_active = True
            response.status_message = "Search started - moved to home position"
            self.get_logger().info('Search started, moved to home position')
        except Exception as exc:
            response.is_active = False
            response.status_message = f"Failed to start search: {exc}"
            self.get_logger().error(f'Start search failed: {exc}')
        
        return response

    def move_to_goal_callback(self, request, response):
        """Move to drop position to release object."""
        if self.arm is None:
            response.object_detected = False
            response.status_message = "Arm not connected"
            return response
        
        """Move arm to a specific square on the grid."""
        if self.arm is None:
            response.finished_moving = False
            response.status_message = "Arm not connected"
            return response
        
        square_num = request.square_number
        
        # Validate square number
        if square_num not in POSITIONS:
            response.finished_moving = False
            response.status_message = f"Invalid square number: {square_num}. Must be 0-9."
            self.get_logger().warning(f'Invalid square number: {square_num}')
            return response
        
        try:
            # Move to the square position
            position = POSITIONS[square_num]
            self.arm.setPosition(position, wait=True)
            
            response.finished_moving = True
            response.status_message = f"Moved to square {square_num}"
            self.get_logger().info(f'Moved to square {square_num}')
        except Exception as exc:
            response.finished_moving = False
            response.status_message = f"Failed to move to square {square_num}: {exc}"
        """Close gripper and detect if object was grasped."""
        if self.arm is None:
            response.object_detected = False
            response.message = "Arm not connected"
            return response
        
        """Cancel operation - turn off all servos."""
        if self.arm is None:
            response.is_cancelled = False
            response.status_message = "Arm not connected"
            return response
        
        try:
            if request.cancel:
                # Disable all servos
                self.arm.servoOff()
                response.is_cancelled = True
                response.status_message = "All servos turned off"
                self.get_logger().info('Servos turned off - operation cancelled')
            else:
                response.is_cancelled = False
                response.status_message = "Cancel not requested"
        except Exception as exc:
            response.is_cancelled = False
            response.status_message = f"Failed to cancel: {exc}"
            self.get_logger().error(f'Cancel failed: {exc}')
    
    def move_to_square_callback(self, request, response):
        # ... perform move to square action ...
        response.success = True
        response.status_message = "Moved to square"
        return response

    def object_detect_callback(self, request, response):
        """Detect object by attempting to close gripper."""
        gripper_closed_count = 200  # Define this constant
        gripper_open_count = 850    # Define this constant
        
        try:
            current_pos = self.arm.getPosition()
            
            # Close gripper
            closed_pos = [gripper_closed_count] + current_pos[1:]
            self.arm.setPosition(closed_pos, wait=True)
            
            import time
            time.sleep(0.5)
            
            final_pos = self.arm.getPosition()
            gripper_position = final_pos[0]
            
            # Allow some tolerance (e.g., within 50 counts of target)
            if abs(gripper_position - gripper_closed_count) > 50:
                response.object_detected = True
                response.message = f"Object detected (gripper at {gripper_position})"
                self.get_logger().info(f'Object detected at gripper position {gripper_position}')
            else:
                response.object_detected = False
                response.message = "No object detected (gripper fully closed)"
                # Reopen gripper if no object
                open_pos = [gripper_open_count] + current_pos[1:]
                self.arm.setPosition(open_pos, wait=True)
                self.get_logger().info('No object detected, gripper reopened')
                
        except Exception as exc:
            response.object_detected = False
            response.message = f"Object detection failed: {exc}"
            self.get_logger().error(f'Object detection failed: {exc}')
        
        return response

    def cancel_callback(self, request, response):
        """Cancel operation by disabling servos."""
        try:
            if request.cancel:
                self.arm.servoOff()
                response.is_cancelled = True
                response.status_message = "All servos turned off"
                self.get_logger().info('Servos turned off - operation cancelled')
            else:
                response.is_cancelled = False
                response.status_message = "Cancel not requested"
        except Exception as exc:
            response.is_cancelled = False
            response.status_message = f"Failed to cancel: {exc}"
            self.get_logger().error(f'Cancel failed: {exc}')
    
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
