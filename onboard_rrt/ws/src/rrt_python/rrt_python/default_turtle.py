import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TransformStamped, Vector3, PointStamped
from nav_msgs.msg import Path
from tf2_msgs.msg import TFMessage
from turtlesim.msg import Pose
# from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MyPythonNode(Node):
    def __init__(self):
        super().__init__("rrt_python")
        self.get_logger().info("This node just controls the tutle using a linear approach.")

        self.pos = PoseStamped()
        self.turtle_move = Twist()
        self.turtle_path = Path()

        self.turtle_pose_x = 0.0
        self.turtle_pose_y = 0.0
        self.turtle_pose_z = 0.0
        self.turtle_pose_theta = 0.0

        self.goal_pose_x = 5.0
        self.goal_pose_y = 5.0
        self.goal_pose_z = 0.0

        self.vel_Kp = 2.0
        self.theta_Kp = 2.0

        self.turtle_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle_pose_callback, 10)
        self.move_goal_sub = self.create_subscription(PoseStamped, "move_base_simple/goal", self.goal_pose_callback, 10)
        self.turtle_pose_pub = self.create_publisher(PoseStamped, '/turtle1/pose_stamped', 1000)
        self.turtle_path_pub = self.create_publisher(Path, '/turtle1/path', 1000)
        self.turtle_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 1000)
        self.turtle_tf_pub = self.create_publisher(TFMessage, '/tf', 1000)
        self.goal_point_pub = self.create_publisher(PointStamped, '/turtle1/goal', 1000)


    # update current turtle position
    def turtle_pose_callback(self, data):
        self.turtle_pose_x = data.x
        self.turtle_pose_y = data.y
        self.turtle_pose_theta = data.theta

        self.pos.header = Header()
        self.pos.header.frame_id = "map"
        self.pos.pose.position.x = self.turtle_pose_x
        self.pos.pose.position.y = self.turtle_pose_y
        self.pos.pose.position.z = self.turtle_pose_z

        # print("turtle_pose_theta = ", self.turtle_pose_theta)
        # print("something: ", self.get_quaternion_from_euler(0,0,0))
        quaternion = self.get_quaternion_from_euler(0, 0, self.turtle_pose_theta)
        self.pos.pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Publish topic data
        self.turtle_pose_pub.publish(self.pos)

        self.turtleControl()
        self.turtleTF()

    def turtleTF(self):
        trans_msg = TransformStamped()
        trans_msg.header.frame_id = "map"
        trans_msg.header.stamp = self.get_clock().now().to_msg()
        trans_msg.child_frame_id = "turtle"
        quaternion = self.get_quaternion_from_euler(0, 0, self.turtle_pose_theta)
        trans_msg.transform.rotation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
        trans_vec = Vector3()
        trans_vec.x = self.turtle_pose_x
        trans_vec.y = self.turtle_pose_y
        trans_vec.z = self.turtle_pose_z
        trans_msg.transform.translation = trans_vec

        turtleTF_msg = TFMessage()
        turtleTF_msg.transforms = [trans_msg]

        self.turtle_tf_pub.publish(turtleTF_msg)

    # update goal position
    def goal_pose_callback(self, data):
        self.goal_pose_x = data.pose.position.x
        self.goal_pose_y = data.pose.position.y
        self.goal_pose_z = data.pose.position.z

        print("goal_x = ", self.goal_pose_x)
        print("goal_y = ", self.goal_pose_y)

        # Update Posestamped for goal location
        goal_point = PointStamped()
        goal_point.header.frame_id = "map"
        goal_point.point.x = self.goal_pose_x
        goal_point.point.y = self.goal_pose_y
        goal_point.point.z = self.goal_pose_z
        self.goal_point_pub.publish(goal_point)

        # Update planned path using current location and 
        self.turtle_path.header = Header()
        self.turtle_path.header.frame_id = "map"

        # Find direction from turtle to goal
        goal_vec = np.array([self.goal_pose_x, self.goal_pose_y])
        turtle_vec = np.array([self.turtle_pose_x, self.turtle_pose_y])
        
        dir_vec = goal_vec - turtle_vec
        dir_vec = dir_vec/np.linalg.norm(dir_vec)
        goal_theta = np.arctan2(dir_vec[1], dir_vec[0])

        poses = [PoseStamped(), PoseStamped()]
        # Set current location PoseStamped
        poses[0].pose.position.x = self.turtle_pose_x
        poses[0].pose.position.y = self.turtle_pose_y
        poses[0].pose.position.z = self.turtle_pose_z
        quaternion = self.get_quaternion_from_euler(0, 0, goal_theta)
        poses[0].pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Set goal location PoseStamped
        poses[1].pose.position.x = self.goal_pose_x
        poses[1].pose.position.y = self.goal_pose_y
        poses[1].pose.position.z = self.goal_pose_z
        quaternion = self.get_quaternion_from_euler(0, 0, goal_theta)
        poses[1].pose.orientation = Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])

        # Write PoseStamped locations to Path
        self.turtle_path.poses = poses

        # Publish path
        self.turtle_path_pub.publish(self.turtle_path)

    def turtleControl(self):
        # Find direction from turtle to goal
        goal_vec = np.array([self.goal_pose_x, self.goal_pose_y])
        turtle_vec = np.array([self.turtle_pose_x, self.turtle_pose_y])
        
        dir_vec = goal_vec - turtle_vec
        dir_mag = np.linalg.norm(dir_vec)
        dir_vec = dir_vec/np.linalg.norm(dir_vec)
        # print("dir_vec: ", dir_vec)

        # Calculate turtle turning
        goal_theta = np.arctan2(dir_vec[1], dir_vec[0])
        turtle_theta = self.turtle_pose_theta
        if (goal_theta-turtle_theta) > np.pi:
            turn_rate = self.pidCalc(goal_theta-2*np.pi, turtle_theta, Kp=self.theta_Kp)
        elif (goal_theta-turtle_theta) < -np.pi:
            turn_rate = self.pidCalc(goal_theta, turtle_theta-2*np.pi, Kp=self.theta_Kp)
        else:
            turn_rate = self.pidCalc(goal_theta, turtle_theta, Kp=self.theta_Kp)

        # Calculate turtle forward movement
        forward_vel = 0.0
        if abs(goal_theta-turtle_theta) < 0.1:
            forward_vel = self.pidCalc(0, dir_mag, Kp=self.vel_Kp)

        # Set and send commands to turtle
        self.turtle_move.linear.x = 0.0
        self.turtle_move.linear.y = 0.0
        self.turtle_move.linear.z = 0.0
        self.turtle_move.angular.x = 0.0
        self.turtle_move.angular.y = 0.0
        self.turtle_move.angular.z = 0.0
        if abs(forward_vel) > 1e-1:
            self.turtle_move.linear.x = -forward_vel
        if abs(turn_rate) > 1e-1:
            self.turtle_move.angular.z = turn_rate

        self.turtle_vel_pub.publish(self.turtle_move)

    def pidCalc(self, desired, measured, Kp=0.0, Ki=0.0, Kd=0.0):
        output = 0.0

        # Proportional
        error = desired - measured
        output += Kp*error

        return output

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    # rclpy.spin(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()