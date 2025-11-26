from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from circle import calibrationBlock, conveyorBelt
from error_plot import analyze_line_fit, degree_error
import math
import random
import sys

FOUR_POINT_CALIBRATION = "4_point_calibration"
THREE_POINT_CALIBRATION = "3_point_calibration"
X_AXIS = "find_x_axis"
Y_AXIS = "find_y_axis"
FIND = 1.0
RETURN = -1.0

class TrajectoryPublisher(Node):
    def __init__(self, calibration_type=FOUR_POINT_CALIBRATION):
        super().__init__('trajectory_node')

        self.path_pub = self.create_publisher(Path, 'particle_path', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # --- Parameters ---
        self.step_xy = 0.05
        self.step_z = 0.05
        self.radius = 0.3
        self.angular_speed = 0.1
        self.start_height = 2.0
        self.block_height = 1.0
        self.three_point_axis_dist = 0.7

        if calibration_type == FOUR_POINT_CALIBRATION:
            self.conveyor_belt = conveyorBelt(num_points=4)
        elif calibration_type == THREE_POINT_CALIBRATION:
            self.conveyor_belt = conveyorBelt(num_points=1)
            
        self.calibration_block = calibrationBlock(self.step_xy, self.conveyor_belt.conveyor_intervals)
        self.conveyor_belt.publish_markers()
        self.calibration_block.publish_markers()
        self.x_hits = []
        self.y_hits = []
        self.block_movement_initialized = False
        self.predicted_points = []
        self.actual_points = []
        self.axis_points = []
        self.three_points = []

        # --- State variables ---
        self.get_logger().info("Current Circle Origin ({:.2f}, {:.2f})".format(self.calibration_block.conveyor_position, 0.0))
        self.x = self.calibration_block.conveyor_position + random.randint(-15, 15) * 0.01
        self.y = random.randint(-15, 15) * 0.01
        self.get_logger().info("Random Circle Center ({:.2f}, {:.2f})".format(self.x, self.y))
        self.z = self.start_height
        self.angle = 0.0
        self.init_angle = 0.0
        self.state = "init_descent"
        self.center_x = None
        self.center_y = None
        self.estimated_center_x = None
        self.estimated_center_y = None
        self.last_hit_time = {"x": 0.0, "y": 0.0}  # debounce timer
        self.hit_cooldown = 1.0  # seconds
        self.calibration_state = calibration_type

        self.create_timer(0.1, self.state_machine)

    def publish_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = self.z
        pose.pose.orientation.w = 1.0

        self.path_msg.header.stamp = pose.header.stamp
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

    def init_descent(self):
        # TODO: What motion is best to detect circle depth? What to do if you exceed max depth without detection?
        self.z = max(self.block_height, self.z - self.step_z)
        if self.z > self.block_height:
            self.publish_pose()
        else:
            self.center_x = self.x
            self.center_y = self.y
            self.state = "radius_away"
            self.get_logger().info("Reached circle, moving a radius away.")
        return
    
    def radius_away(self):
        distance = self.two_d_euclidean_distance(self.x, self.y, self.center_x, self.center_y)
        if distance < self.radius:
            self.x = self.x + self.radius*self.step_xy
            self.y = self.y + self.radius*self.step_xy
            self.publish_pose()
        else:
            self.state = "circle"
            self.angle = math.atan2(self.y - self.center_y, self.x - self.center_x)
            self.init_angle = self.angle
            self.get_logger().info("Reached radius, starting circle motion.")

        return
    
    def rotate_circle(self):
        # TODO: If you don't have enough points, increase circle radius up to a limit?
        if self.angle <= self.init_angle + 2 * math.pi:
            self.angle += self.angular_speed
            self.x = self.center_x + self.radius * math.cos(self.angle)
            self.y = self.center_y + self.radius * math.sin(self.angle)
            self.publish_pose()

            # TODO: find better logic for laser detection than 3d euclidean distance (needs to be in laser array)
            for pt in self.calibration_block.laser_x:
                dist = math.sqrt((self.x - pt[0])**2 + (self.y - pt[1])**2 + (self.z - pt[2])**2)
                if dist < self.step_xy * 0.6:
                    current_time = self.get_clock().now().nanoseconds / 1e9 
                    if current_time - self.last_hit_time["x"] > self.hit_cooldown:
                        self.x_hits.append((self.x, self.y, self.z))
                        self.last_hit_time["x"] = current_time
                        self.get_logger().info("Detected laser on X-axis at position ({:.2f}, {:.2f}, {:.2f}), Distance: {:.4f}".format(self.x, self.y, self.z, dist))

            for pt in self.calibration_block.laser_y:
                dist = math.sqrt((self.x - pt[0])**2 + (self.y - pt[1])**2 + (self.z - pt[2])**2)
                if dist < self.step_xy * 0.6:
                    current_time = self.get_clock().now().nanoseconds / 1e9
                    if current_time - self.last_hit_time["y"] > self.hit_cooldown:
                        self.y_hits.append((self.x, self.y, self.z))
                        self.last_hit_time["y"] = current_time
                        self.get_logger().info("Detected laser on Y-axis at position ({:.2f}, {:.2f}, {:.2f}), Distance: {:.4f}".format(self.x, self.y, self.z, dist))
        else:
            if not self.x_hits or not self.y_hits:
                self.get_logger().warning("Insufficient laser hits detected. Trajectory failed.")

            self.estimated_center_x = sum([hit[0] for hit in self.x_hits]) / len(self.x_hits) if self.x_hits else self.center_x
            self.estimated_center_y = sum([hit[1] for hit in self.y_hits]) / len(self.y_hits) if self.y_hits else self.center_y
            self.get_logger().info("Estimated Center ({:.2f}, {:.2f})".format(self.estimated_center_x, self.estimated_center_y))
            self.get_logger().info("Actual Center ({:.2f}, {:.2f})".format(self.center_x, self.center_y))

            self.state = "find_center"
            self.get_logger().info("Circle complete, returning to circle center.")
        return
    
    def return_center(self):
        dx = self.estimated_center_x - self.x
        dy = self.estimated_center_y - self.y
        dist = math.sqrt(dx**2 + dy**2)

        if dist > self.step_xy:
            self.x += self.step_xy * dx / dist
            self.y += self.step_xy * dy / dist
            self.publish_pose()
        else:
            self.get_logger().info("Returned to circle center, returning to origin.")
            self.get_logger().info("Current Position ({:.2f}, {:.2f})".format(self.x, self.y))
            self.state = "find_origin"
        return
    
    def find_origin(self):
        # TODO: if center is missed, reset to radius_away and repeat the circle motion
        dx = self.calibration_block.conveyor_position - self.x
        dy = 0.0 - self.y
        dist = math.sqrt(dx**2 + dy**2)
        # self.get_logger().info("Current Position ({:.2f}, {:.2f})".format(dx, dy))
        # TODO: Keep moving with laser estimate until the sign of your distance changes (which means you passed the center)
        if dist > self.step_xy:
            dx /= dist
            dy /= dist
            self.x += self.step_xy * dx
            self.y += self.step_xy * dy
            self.publish_pose()
        else:
            self.get_logger().info("Returned to global origin, descending to find eddy sensor.")
            self.get_logger().info("Current Position ({:.2f}, {:.2f})".format(self.x, self.y))
            self.state = "find_eddy"
    
    def find_eddy(self):
        # TODO: If Eddy sensor not found, reset entire process, or maybe just return to circle depth and try again?
        eddy_x = self.calibration_block.eddy_marker.pose.position.x
        eddy_y = self.calibration_block.eddy_marker.pose.position.y
        eddy_z = self.calibration_block.eddy_marker.pose.position.z
        eddy_distance = self.three_d_euclidean_distance(self.x, self.y, self.z, eddy_x, eddy_y, eddy_z)
        self.get_logger().info(str(eddy_distance))
        if self.z <= eddy_z and eddy_distance > self.step_xy:
            self.get_logger().info("Missed eddy sensor. Trajectory failed.")
            sys.exit(0)
        elif eddy_distance > self.step_xy:
            self.z = max(-1, self.z - self.step_z)
            self.publish_pose()
        else:
            self.get_logger().info("Reached eddy sensor height. Trajectory complete.")
            self.actual_points.append((self.x, self.y))
            self.predicted_points.append((self.calibration_block.conveyor_position, 0.0))
            self.state = "init_ascent"

    def init_ascent(self):
        if self.z < self.block_height:
            self.z += self.step_z
            self.publish_pose()
        else:
            self.get_logger().info("Initial Ascent complete")
            self.three_points.append((self.x, self.y, self.z))
            self.state = "find_x_axis"
        return
    
    def find_axis(self, direction=FIND, axis=X_AXIS):
        if self.calibration_state == FOUR_POINT_CALIBRATION:
            self.state = "final_ascent"
            return

        finished_state = False
        if axis == X_AXIS:
            if direction == FIND and (self.x -  self.calibration_block.conveyor_position) < self.three_point_axis_dist:
                self.x += self.step_xy * self.three_point_axis_dist
                self.get_logger().info("Current Position ({:.2f}, {:.2f})".format(self.x, self.y))
                self.publish_pose()
            elif direction == RETURN and (self.x -  self.calibration_block.conveyor_position) > 0:
                self.x -= self.step_xy * self.three_point_axis_dist
                self.publish_pose()
            else:
                finished_state = True
        elif axis == Y_AXIS:
            if direction == FIND and self.y < self.three_point_axis_dist:
                self.y += self.step_xy * self.three_point_axis_dist
                self.publish_pose()
            elif direction == RETURN and self.y > 0:
                self.y -=  self.step_xy * self.three_point_axis_dist
                self.publish_pose()
            else:
                finished_state = True
        
        if finished_state:
            if self.state == "find_x_axis":
                self.three_points.append((self.x, self.y, self.z))
                self.get_logger().info("Find X axis complete")
                self.state = "return_x_axis"
            elif self.state == "return_x_axis":
                self.get_logger().info("Return X axis complete")
                self.state = "find_y_axis"
            elif self.state == "find_y_axis":
                self.three_points.append((self.x, self.y, self.z))
                self.get_logger().info("Find Y axis complete")
                self.state = "return_y_axis"
            else:
                self.get_logger().info("Return Y axis complete")
                self.state = "final_ascent"
        return

    def final_ascent(self):
        if self.z < self.start_height:
            self.z += self.step_z
            self.publish_pose()
        else:
            self.get_logger().info("Final Ascent complete")
            self.state = "find_block"
        return
    
    def find_block(self):
        # TODO: Right now we are assuming manual movement of arm to the block position
        if not self.block_movement_initialized:
            self.calibration_block.move_block()
            self.block_movement_initialized = True
            return

        dx = self.calibration_block.conveyor_position - self.x
        if abs(dx) > self.step_xy:
            self.x += self.step_xy * dx
            self.publish_pose()
        else:
            if self.calibration_block.state == "complete":
                if self.calibration_state == FOUR_POINT_CALIBRATION:
                    print(self.actual_points)
                    print(self.predicted_points)
                    self.get_logger().info("4-Point Calibration Complete. Exiting.")
                    analyze_line_fit(self.actual_points, show_plot=False)
                    sys.exit(0)
                else:
                    self.get_logger().info("3-Point Calibration Complete. Exiting.")
                    theta = degree_error(self.three_points)
                    print("Calibration Accuracy:", theta)
                    sys.exit(0)

            else:
                self.get_logger().info("Reached calibration block. Restarting process.")
                self.state = "init_descent"
                self.block_movement_initialized = False
                self.x_hits.clear()
                self.y_hits.clear()

    def two_d_euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    
    def three_d_euclidean_distance(self, x1, y1, z1, x2, y2, z2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def state_machine(self):
        
        if self.state == "init_descent":
            self.init_descent()
            return

        elif self.state == "radius_away":
            self.radius_away()
            return
        
        elif self.state == "circle":
            self.rotate_circle()
            return

        elif self.state == "find_center":
            self.return_center()
            return
    
        elif self.state == "find_origin":
            self.find_origin()
            return

        elif self.state == "find_eddy":
            self.find_eddy()
            return
        
        elif self.state == "init_ascent":
            self.init_ascent()
            return

        elif self.state == "find_x_axis":
            self.find_axis(direction=FIND, axis=X_AXIS)
            return         

        elif self.state == "return_x_axis":
            self.find_axis(direction=RETURN, axis=X_AXIS)
            return

        elif self.state == "find_y_axis":
            self.find_axis(direction=FIND, axis=Y_AXIS)
            return

        elif self.state == "return_y_axis":
            self.find_axis(direction=RETURN, axis=Y_AXIS)
            return

        elif self.state == "final_ascent":
            self.final_ascent()
            return
        
        elif self.state == "find_block":
            self.find_block()
            return

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher(calibration_type=THREE_POINT_CALIBRATION)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
