'''
this service handles the initiation of the motor driver card
handles the joint_states feedback call
determines the different wheel configurations 2wd / 4wd / mecanum
'''
import math
import rclpy

from .motor import Motor

from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from urdf_parser_py.urdf import URDF

class VelocityService(Node):
    def __init__(self):
        super().__init__('nanosaur')
        # Initialize eyes controller
        # self.eyes = eyes(self)

        self.mecanumOr4wd = True
        # Get rate joint_states
        self.declare_parameter("rate", 5)
        self.timer_period = 1. / float(self.get_parameter("rate").value)
        # Get RPM motors
        self.declare_parameter("rpm", 150)
        self.rpm = int(self.get_parameter("rpm").value)
        
        # INITIATE MOTORS
        
        # Get parameter left wheel name
        # https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/
        left_id = 1
        self.declare_parameter("motor.left.channel", left_id)
        self.left_wheel_name = "sprocket_left_joint"
        self.declare_parameter("motor.left.wheel", self.left_wheel_name)
        
        # Get parameter right wheel name
        right_id = 4
        self.declare_parameter("motor.right.channel", right_id)
        self.right_wheel_name = "sprocket_right_joint"
        self.declare_parameter("motor.right.wheel", self.right_wheel_name)
        
        # Motor info message
        self.get_logger().info(f"RPM motors {self.rpm} - timer {self.timer_period}")
        self.get_logger().info(f"Motor left: Channel {left_id} - Wheel {self.left_wheel_name}")
        self.get_logger().info(f"Motor Right: Channel {right_id} - Wheel {self.right_wheel_name}")
        
        # Load motors
        self.mright = Motor(right_id, self.rpm)
        self.mleft = Motor(left_id, self.rpm)

        if self.mecanumOr4wd :
            # initiate 2 more motors
            left_front_id = 2
            self.declare_parameter("motor.left.front.channel",left_front_id)
            self.left_wheel_front_name = "sprocket_left_front_joint"
            self.declare_parameter("motor.left.front.wheel", self.left_wheel_front_name )

            right_front_id = 3
            self.declare_parameter("motor.right.front.channel", right_front_id)
            self.right_wheel_front_name = "sprocket_right_front_joint"
            self.declare_parameter("motor.right.front.wheel", self.right_wheel_front_name)

            self.get_logger().info(f"Motor left front: Channel {left_front_id} - Wheel {self.left_wheel_front_name}")
            self.get_logger().info(f"Motor Right front: Channel {right_front_id} - Wheel {self.right_wheel_front_name}")
        
            self.mright_front = Motor(right_front_id, self.rpm, inverted=True)
            self.mleft_front = Motor(left_front_id, self.rpm, inverted=True)
        
        self.create_subscription(String, 'robot_description',
            lambda msg: self.configure_robot(msg.data),
            QoSProfile(depth=1, durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL))
               
        # Drive control
        self.p = [0.0, 0.0] # [right, left]
        self.r = [0.0, 0.0] # [right, left]
        self.subscription = self.create_subscription(Twist,'cmd_vel', self.drive_callback, 10)
        self.subscription  # prevent unused variable warning
        # Node started
        self.get_logger().info("Hello NanoSaur!")

    def drive_callback(self, msg):
        # awake displays
        # self.eyes.ping()
        # todo : apply different calculation for mecanum.
        # Store linear velocity and angular velocity
        v = msg.linear.x
        w = msg.angular.z
        # Convert linear and angular velocity to radiant motor speed
        self.get_logger().debug(f"v={v} w={w}")
        r = self.convert_speed(v, w)
        self.get_logger().debug(f"rad {r}")
        max_speed = self.rpm / 60.
        # Constrain between -max_speed << speed << max_speed.
        self.r = [max(-max_speed, min(max_speed, r[0])), max(-max_speed, min(max_speed, r[1]))]
        # Send a warning message if speed is over 
        if r[0] != self.r[0]:
            self.get_logger().warning(f"ref speed over {r[0] - self.r[0]}")
        if r[1] != self.r[1]:
            self.get_logger().warning(f"ref speed over {r[1] - self.r[1]}")
        # Convert speed to motor speed in RPM
        rpmr = self.r[0] * 60.
        rpml = self.r[1] * 60.
        # Set to max RPM available
        self.get_logger().info(f"RPM R={rpmr} L={rpml}")
        self.mright.set_speed(rpmr)
        self.mleft.set_speed(rpml)

        if self.mecanumOr4wd :
            self.mright_front.set_speed(rpmr)
            self.mleft_front.set_speed(rpmr)


    def convert_speed(self, v, w):
        half_wheel_separation = self.wheel_separation / 2.
        vr = v + half_wheel_separation * w
        vl = v - half_wheel_separation * w
        rr = vr / self.radius
        rl = vl / self.radius
        return [rr, rl]
    


    def configure_robot(self, description):
        self.get_logger().info('Got description, configuring robot')
        # Load description
        # From https://github.com/ros-controls/ros_controllers/blob/noetic-devel/diff_drive_controller/src/diff_drive_controller.cpp
        robot = URDF.from_xml_string(description)
        # Get left joint wheel
        joint_left = self.get_joint(robot, self.left_wheel_name)
        # Get right joint wheel
        joint_right = self.get_joint(robot, self.right_wheel_name)
        # Measure wheel separation
        self.wheel_separation = self.euclidean_of_vectors(joint_left.origin.xyz, joint_right.origin.xyz)
        # Get radius joint
        link_left = self.get_link(robot, joint_left.child)
        self.radius = link_left.collision.geometry.radius
        self.get_logger().info(f"Wheel separation {self.wheel_separation} - Radius {self.radius}")

    def get_joint(robot, joint_name):
        for joint in robot.joints:
            if joint.name == joint_name:
                return joint
        raise Exception(f"Joint {joint_name} not found")

    def get_link(robot, link_name):
        for link in robot.links:
            if link.name == link_name:
                return link
        raise Exception(f"link {link_name} not found")

    def euclidean_of_vectors(xyz1, xyz2):
        return math.sqrt(
            (xyz1[0] - xyz2[0]) ** 2 +
            (xyz1[1] - xyz2[1]) ** 2 +
            (xyz1[2] - xyz2[2]) ** 2)


