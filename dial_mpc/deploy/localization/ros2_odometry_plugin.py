import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from dial_mpc.deploy.localization.base_plugin import BaseLocalizationPlugin


class ROS2OdometryPlugin(BaseLocalizationPlugin, Node):
    def __init__(self, config):
        BaseLocalizationPlugin.__init__(self, config)
        rclpy.init()
        Node.__init__(self, "ros2_odom_plugin")
        self.subscription = self.create_subscription(
            Odometry, config["odom_topic"], self.odom_callback, 1
        )

        self.qpos = None
        self.qvel = None
        self.last_time = None
        print("Subscribing to: ", config["odom_topic"])

        # while rclpy.ok():
        #     rclpy.spin(self) 
                # start rclpy.spin in a background thread
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._spin_thread.start()
        print("ROS2OdometryPlugin spinning in background on", config["odom_topic"])


    def __del__(self):
        print("Died")
        rclpy.shutdown()

    def odom_callback(self, msg):
        print("=========>>> Received odometry <<<===========")
        qpos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                msg.pose.pose.orientation.w,
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
            ]
        )
        vb = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ]
        )
        ab = np.array(
            [
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
            ]
        )
        print("Odometry: ", qpos, vb, ab)
        # rotate velocities to world frame
        q = R.from_quat([qpos[3], qpos[4], qpos[5], qpos[6]])
        vw = q.apply(vb)
        aw = q.apply(ab)
        self.qpos = qpos
        self.qvel = np.concatenate([vw, aw])
        self.last_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def get_state(self):
        return np.concatenate([self.qpos, self.qvel]) if self.qpos is not None else None

    def get_last_update_time(self):
        return self.last_time
