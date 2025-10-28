#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import math
import time # Used for client call timeout

class ObstacleAvoider(Node):
    """
    ROS 2 Node for controlling the turtlesim turtle to reach a destination
    while avoiding a defined rectangular obstacle in the center of the map.
    """

    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)

        # 1. Configuration Parameters
        self.goal_x = 9.0  # Target X coordinate
        self.goal_y = 9.0  # Target Y coordinate
        self.pose = Pose()

        # Define the obstacle zone (a simple rectangle in the center)
        self.OBSTACLE_X_MIN = 4.0
        self.OBSTACLE_X_MAX = 7.0
        self.OBSTACLE_Y_MIN = 4.0
        self.OBSTACLE_Y_MAX = 7.0
        self.AVOIDANCE_DISTANCE = 1.5 # How close is too close to the obstacle perimeter (e.g., 2.5 to 8.5)

        # 2. Setup Clients for drawing the obstacle
        self.set_pen_cli = self.create_client(SetPen, 'turtle1/set_pen')
        self.teleport_cli = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.clear_cli = self.create_client(Empty, 'clear')

        # 3. Initialize services synchronously
        self.get_logger().info('Obstacle Avoider Node Initialized. Attempting to set up Turtlesim environment...')
        if not self.set_pen_cli.wait_for_service(timeout_sec=5.0) or \
           not self.teleport_cli.wait_for_service(timeout_sec=5.0) or \
           not self.clear_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Turtlesim services not available after waiting 5 seconds. Exiting.")
            # Set a flag or destroy the node if services are critical but unavailable
            rclpy.shutdown()
            return

        # Synchronous Initialization
        self.draw_obstacle_boundary()
        # Resetting to (1.0, 5.5) to start safely outside the avoidance zone.
        self.reset_turtle(1.0, 5.5, 0.0)
        self.get_logger().info(f'Turtle reset to start position (1.0, 5.5). Navigation starting.')

        # 4. Timer to drive the control loop (50Hz)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.control_loop)


    def call_service(self, client, request):
        """Helper to call a service synchronously and handle the future."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Service call failed for client {client.srv_name}.')
            return None


    def draw_obstacle_boundary(self):
        """Draws the visible boundary of the obstacle using synchronous service calls."""

        # 1. Clear the screen (Synchronous)
        self.call_service(self.clear_cli, Empty.Request())

        # 2. Define the path points 
        points = [
            (self.OBSTACLE_X_MIN, self.OBSTACLE_Y_MIN), # P1 (bottom-left)
            (self.OBSTACLE_X_MAX, self.OBSTACLE_Y_MIN), # P2 (bottom-right)
            (self.OBSTACLE_X_MAX, self.OBSTACLE_Y_MAX), # P3 (top-right)
            (self.OBSTACLE_X_MIN, self.OBSTACLE_Y_MAX), # P4 (top-left)
            (self.OBSTACLE_X_MIN, self.OBSTACLE_Y_MIN)  # Back to P1 (to close the loop)
        ]
        
        # 3. Set pen color/thickness for the boundary (Red, visible)
        # Set pen OFF and move to the start point (P1)
        pen_off_req = SetPen.Request(r=255, g=0, b=0, width=5, off=1)
        self.call_service(self.set_pen_cli, pen_off_req)
        
        # Teleport to P1 (Synchronous)
        teleport_start_req = TeleportAbsolute.Request(x=points[0][0], y=points[0][1], theta=0.0)
        self.call_service(self.teleport_cli, teleport_start_req)
        
        # Now, set the pen ON (Synchronous)
        pen_on_req = SetPen.Request(r=255, g=0, b=0, width=5, off=0)
        self.call_service(self.set_pen_cli, pen_on_req)

        # 4. Draw the boundary segments (P2 through P1) (Synchronous)
        # Start from the second point (index 1) since we already teleported to the first point
        for x, y in points[1:]:
            teleport_req = TeleportAbsolute.Request(x=x, y=y, theta=0.0)
            self.call_service(self.teleport_cli, teleport_req)
        
        # 5. Turn the pen off and set it back to default white for the main turtle (Synchronous)
        pen_default_req = SetPen.Request(r=255, g=255, b=255, width=2, off=1)
        self.call_service(self.set_pen_cli, pen_default_req)

    def reset_turtle(self, x, y, theta):
        """Teleports the turtle back to a clean starting position."""
        # Teleport to start position (Synchronous)
        teleport_req = TeleportAbsolute.Request(x=x, y=y, theta=theta)
        self.call_service(self.teleport_cli, teleport_req)
        
        # Re-enable the pen after the reset (Synchronous)
        pen_on_req = SetPen.Request(r=255, g=255, b=255, width=2, off=0)
        self.call_service(self.set_pen_cli, pen_on_req)


    def pose_callback(self, msg: Pose):
        """Callback for the /turtle1/pose subscriber."""
        self.pose = msg

    def is_near_obstacle(self):
        """
        Checks if the turtle is within the expanded avoidance range of the defined obstacle.
        The expanded zone includes the obstacle bounds plus the AVOIDANCE_DISTANCE margin.
        """
        x, y = self.pose.x, self.pose.y

        # Check if the turtle is within the expanded bounding box
        is_near_x = (x >= self.OBSTACLE_X_MIN - self.AVOIDANCE_DISTANCE) and \
                    (x <= self.OBSTACLE_X_MAX + self.AVOIDANCE_DISTANCE)

        is_near_y = (y >= self.OBSTACLE_Y_MIN - self.AVOIDANCE_DISTANCE) and \
                    (y <= self.OBSTACLE_Y_MAX + self.AVOIDANCE_DISTANCE)

        # If it's within both the expanded X and Y boundaries, we must evade.
        return is_near_x and is_near_y


    def control_loop(self):
        """Main control loop run by the timer."""
        move = Twist()

        distance_to_goal = math.sqrt((self.goal_x - self.pose.x)**2 + (self.goal_y - self.pose.y)**2)
        
        # Check for goal proximity
        if distance_to_goal < 0.1:
            # Stop the turtle if the goal is reached
            move.linear.x = 0.0
            move.angular.z = 0.0
            self.publisher_.publish(move)
            self.get_logger().info('Goal reached! Stopping.')
            # Cancel the timer to stop the loop
            self.timer.cancel() 
            return

        # --- Obstacle Avoidance Logic ---
        if self.is_near_obstacle():
            self.get_logger().warn('Near Obstacle! Evading by reversing and turning.')
            # TUNE: Increased reverse speed and turn rate for aggressive evasion
            move.linear.x = -1.5  # Move backward faster
            move.angular.z = 3.0  # Turn sharply left
            
        # --- Normal Navigation Logic (Go to Goal) ---
        else:
            # Calculate required angle to goal (radians)
            angle_to_goal = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)

            # Calculate angular error (difference between current heading and required angle)
            angle_error = angle_to_goal - self.pose.theta

            # Normalize the angle error to be between -pi and pi
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            elif angle_error < -math.pi:
                angle_error += 2 * math.pi

            # TUNE: Reduced Proportional Control for angular velocity (P-gain = 1.5)
            # This makes the turtle steer less aggressively.
            move.angular.z = angle_error * 1.5

            # --- Simplified Linear Speed Logic ---
            # Always move forward at a base speed, scaled by distance to goal.
            base_speed = 0.5
            
            # Use distance for P-control, but ensure it is at least the base speed (0.5)
            linear_p_control = distance_to_goal * 0.5
            move.linear.x = max(base_speed, min(linear_p_control, 2.0))

        # Publish the calculated Twist message
        self.publisher_.publish(move)

def main(args=None):
    rclpy.init(args=args)
    # The node will handle the initialization logic now
    avoider_node = ObstacleAvoider() 
    try:
        rclpy.spin(avoider_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the turtle stops on shutdown
        stop_move = Twist()
        stop_move.linear.x = 0.0
        stop_move.angular.z = 0.0
        # Check if the publisher still exists before trying to publish
        if rclpy.ok() and avoider_node.publisher_ is not None:
             avoider_node.publisher_.publish(stop_move)
        avoider_node.destroy_node()
        # Only shutdown if rclpy is still initialized
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
