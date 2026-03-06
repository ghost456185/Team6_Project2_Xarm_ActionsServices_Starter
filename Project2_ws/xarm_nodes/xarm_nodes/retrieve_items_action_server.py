#!/usr/bin/env python3
"""
retrieve_items_action_server.py — Scaffold for the RetrieveItems action server.

TODO(STUDENTS): Implement the action server logic in this file.
You will need to:
  1. Define any service types you need and import them below.
  2. Create service clients in __init__ for each hardware service you call.
  3. Fill in goal_callback, cancel_callback, and execute_callback.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from xarm_pickup_interfaces.action import RetrieveItems
# Imported service types for hardware interaction here.
from xarm_pickup_interfaces.srv import StartSearch  
from xarm_pickup_interfaces.srv import MoveToSquare
from xarm_pickup_interfaces.srv import ObjectDetect
from xarm_pickup_interfaces.srv import MoveToGoal
from xarm_pickup_interfaces.srv import Cancel


class RetrieveItemsActionServer(Node):
    """Action server that executes a RetrieveItems goal."""

    def __init__(self):
        super().__init__('retrieve_items_action_server')

        # State tracking for cancel callback
        self._search_active = False
        self._cancel_pending = False

        # Create service clients for hardware services
        self._start_search_client = self.create_client(StartSearch, 'start_search')
        self._move_to_square_client = self.create_client(MoveToSquare, 'move_to_square')
        self._object_detect_client = self.create_client(ObjectDetect, 'object_detect')
        self._move_to_goal_client = self.create_client(MoveToGoal, 'move_to_goal')
        self._cancel_client = self.create_client(Cancel, 'cancel')

        self._action_server = ActionServer(
            self,
            RetrieveItems,
            'retrieve_items',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        self.get_logger().info('retrieve_items_action_server is running.')

    def goal_callback(self, goal_request):
        """Accept or reject an incoming goal request."""
        
        # Validate num_items is within acceptable range
        min_items = 1
        max_items = 9  # Adjust based on your grid size
        
        if goal_request.num_items < min_items or goal_request.num_items > max_items:
            self.get_logger().warning(
                f'Goal rejected: num_items={goal_request.num_items} is out of range '
                f'[{min_items}, {max_items}]'
            )
            return GoalResponse.REJECT
        
        self.get_logger().info(f'Goal accepted: num_items={goal_request.num_items}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request for an active goal."""

        # will: Return CancelResponse.REJECT if cancellation should be refused.

        if not self._search_active:
            self.get_logger().info('Cancel rejected: search is not active.')
            return CancelResponse.REJECT
       
        self.get_logger().info('Cancel requested: accepted (deferred until motion completes).')
        self._cancel_pending = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the RetrieveItems goal.

        This method is called in its own thread by the MultiThreadedExecutor,
        so blocking calls are safe here. It must publish feedback periodically
        and return a populated Result when finished.

        TODO(STUDENTS): Implement the pick-and-place loop here.
        """
        self.get_logger().info('Executing goal...')

        feedback_msg = RetrieveItems.Feedback()
        result = RetrieveItems.Result()

        # Initialize tracking variables
        items_collected = 0
        target_items = goal_handle.request.num_items
        
        # Start search
        start_req = StartSearch.Request()
        start_req.activate = True
        start_resp = await self._start_search_client.call_async(start_req)
        
        if start_resp.is_active:
            self._search_active = True
        else:
            result.success = False
            result.message = 'Failed to start search'
            result.items_collected = 0
            goal_handle.abort()
            return result

        # Item retrieval loop - iterate through grid boxes (1-9 for 3x3 grid)
        for current_box in range(1, 10):  # Grid boxes 1-9 (0 is home position)
            # Check if we've collected enough items
            if items_collected >= target_items:
                break

            # Update feedback - moving to box
            feedback_msg.state = 'moving'
            feedback_msg.current_box = current_box
            feedback_msg.items_collected = items_collected
            goal_handle.publish_feedback(feedback_msg)

            # Move to square
            move_req = MoveToSquare.Request()
            move_req.square_number = current_box
            move_resp = await self._move_to_square_client.call_async(move_req)
            
            if not move_resp.finished_moving:
                self.get_logger().warning(f'Failed to move to square {current_box}: {move_resp.status_message}')
                continue

            # Check for cancel after motion completes
            if self._cancel_pending:
                self._cancel_pending = False
                self._search_active = False

                cancel_req = Cancel.Request()
                cancel_req.cancel = True
                await self._cancel_client.call_async(cancel_req)

                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled after current motion; servos turned off.'
                result.items_collected = items_collected
                return result

            # Update feedback - detecting object
            feedback_msg.state = 'grasping'
            feedback_msg.current_box = current_box
            feedback_msg.items_collected = items_collected
            goal_handle.publish_feedback(feedback_msg)

            # Close gripper and detect object
            detect_req = ObjectDetect.Request()
            detect_req.close_gripper = True
            detect_resp = await self._object_detect_client.call_async(detect_req)

            if detect_resp.object_detected:
                # Object found! Move to goal location
                feedback_msg.state = 'dropping'
                goal_handle.publish_feedback(feedback_msg)

                goal_req = MoveToGoal.Request()
                goal_req.move_to_goal = True
                goal_resp = await self._move_to_goal_client.call_async(goal_req)

                if goal_resp.object_detected:
                    items_collected += 1
                    self.get_logger().info(f'Item {items_collected} collected from box {current_box}')
                else:
                    self.get_logger().warning(f'Failed to drop object: {goal_resp.status_message}')

            # Check for cancel request
            if goal_handle.is_cancel_requested and not self._cancel_pending:
                self._cancel_pending = True

        # Search complete
        self._search_active = False
        
        # Return to home or safe position
        feedback_msg.state = 'complete'
        feedback_msg.current_box = 0  # Home position
        feedback_msg.items_collected = items_collected
        goal_handle.publish_feedback(feedback_msg)

        result.items_collected = items_collected
        result.success = (items_collected >= target_items)
        result.message = f'Collected {items_collected}/{target_items} items'
        
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RetrieveItemsActionServer()

    # MultiThreadedExecutor allows goal, cancel, and execute callbacks to run
    # concurrently — required when the execute_callback blocks or uses await.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
