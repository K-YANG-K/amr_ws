#!/usr/bin/env python3
"""Gazebo 真值定位节点。

通过订阅 Gazebo 的 /gazebo_ros/model_states topic 获取机器人真实位姿和速度，
发布 TF 变换和里程计消息，替代 AMCL 定位。

坐标变换链：
  map ──(static identity)──> odom ──(ground truth)──> base_footprint
  base_footprint ──(URDF fixed joint)──> base_link
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class GroundTruthLocalization(Node):
    """从 Gazebo 真值发布 TF 和 Odometry。"""

    def __init__(self):
        super().__init__('ground_truth_localization')

        self.declare_parameter('robot_model_name', 'logistics_omni')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_topic', '/amr_controller/odom')
        self.declare_parameter('publish_rate', 50.0)

        self.robot_name = self.get_parameter('robot_model_name').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # 订阅 Gazebo 真值
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo_ros/model_states',
            self.model_states_callback,
            10)

        # 发布 TF 和 Odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # 缓存最近一次的真值
        self.latest_pose = None
        self.latest_twist = None

        # ── 一次性发布 map → odom 静态变换（恒等变换，不会变）──────
        self.publish_static_map_odom()

        # 定时发布 odom → base_footprint 和 Odometry
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(
            f'真值定位已启动: robot="{self.robot_name}", '
            f'odom_topic="{self.odom_topic}", base_frame="{self.base_frame}"')

    def publish_static_map_odom(self):
        """发布 map → odom 静态恒等变换（仅一次）。"""
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f'已发布静态变换: {self.map_frame} → {self.odom_frame}')

    def model_states_callback(self, msg: ModelStates):
        """从 model_states 中提取机器人真值。"""
        try:
            idx = msg.name.index(self.robot_name)
        except ValueError:
            return

        self.latest_pose = msg.pose[idx]
        self.latest_twist = msg.twist[idx]

    def timer_callback(self):
        """定时发布 odom → base_footprint 和 Odometry。"""
        now = self.get_clock().now().to_msg()

        # ── TF: odom → base_footprint（Gazebo 真值，未就绪时用零位姿兜底）─
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = now
        t_odom_base.header.frame_id = self.odom_frame
        t_odom_base.child_frame_id = self.base_frame
        if self.latest_pose is not None:
            t_odom_base.transform.translation.x = self.latest_pose.position.x
            t_odom_base.transform.translation.y = self.latest_pose.position.y
            t_odom_base.transform.translation.z = self.latest_pose.position.z
            t_odom_base.transform.rotation = self.latest_pose.orientation
        else:
            # Gazebo 数据尚未到达，用单位变换保证 TF 树连通
            t_odom_base.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t_odom_base)

        # ── Odometry（仅在有真值数据时发布）───────────────────────
        if self.latest_pose is None:
            return

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = self.latest_pose
        if self.latest_twist is not None:
            odom.twist.twist = self.latest_twist

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
