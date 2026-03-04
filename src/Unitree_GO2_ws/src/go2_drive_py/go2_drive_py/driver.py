import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState, LowState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base1')
        self.declare_parameter('publish_tf', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # odom object
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.mode_sub = self.create_subscription(
            SportModeState,
            'lf/sportmodestate',
            self.mode_callback,
            10)
        self.get_logger().info('GO2 Drive Node started')

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # creat joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.lowstate_sub = self.create_subscription(
            LowState,
            '/lf/lowstate',
            self.lowstate_callback,
            10)
        
    def lowstate_callback(self, lowstate :LowState):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'FL_hip_joint',
            'FR_hip_joint',
            'RL_hip_joint',
            'RR_hip_joint',
            'FL_thigh_joint',
            'FR_thigh_joint',
            'RL_thigh_joint',
            'RR_thigh_joint',
            'FL_calf_joint',
            'FR_calf_joint',
            'RL_calf_joint',
            'RR_calf_joint',
        ]
        for i in range(12):
            q = float(lowstate.motor_state[i].q)
            joint_state.position.append(q)
        self.joint_state_pub.publish(joint_state)


    def mode_callback(self, mode :SportModeState):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(mode.position[0])
        odom.pose.pose.position.y = float(mode.position[1])
        odom.pose.pose.position.z = float(mode.position[2])

        odom.pose.pose.orientation.w = float(mode.imu_state.quaternion[0])
        odom.pose.pose.orientation.x = float(mode.imu_state.quaternion[1])
        odom.pose.pose.orientation.y = float(mode.imu_state.quaternion[2])
        odom.pose.pose.orientation.z = float(mode.imu_state.quaternion[3])

        odom.twist.twist.linear.x = float(mode.velocity[0])
        odom.twist.twist.linear.y = float(mode.velocity[1])
        odom.twist.twist.linear.z = float(mode.velocity[2])

        odom.twist.twist.angular.z = float(mode.yaw_speed)
        self.odom_pub.publish(odom)


        # publish tf
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame

            tf.transform.translation.x = float(mode.position[0])
            tf.transform.translation.y = float(mode.position[1])
            tf.transform.translation.z = float(mode.position[2])

            tf.transform.rotation = odom.pose.pose.orientation


            self.tf_broadcaster.sendTransform(tf)
            # self.get_logger().info('Publishing TF from %s to %s' % (self.odom_frame, self.base_frame))
            # self.get_logger().info('TF: %s' % str(tf))
            #



def main(args=None):
    rclpy.init(args=args)
    driver = Driver()
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()