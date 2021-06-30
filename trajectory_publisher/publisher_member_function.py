# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from os import POSIX_FADV_SEQUENTIAL
import rclpy
from rclpy.node import Node

import numpy as np
import pandas as pd
import casadi as cs

from nav_msgs.msg import Odometry




class TrajectoryPublisher(Node):


    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Transforms a set of euler angles (phi,theta,psi) into quaternions.
        """

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return np.array([qw, qx, qy, qz])

    def unit_quat(self,q):
        """
        Normalizes a quaternion to be unit modulus.
        :param q: 4-dimensional numpy array or CasADi object
        :return: the unit quaternion in the same data format as the original one
        """

        if isinstance(q, np.ndarray):
            # if (q == np.zeros(4)).all():
            #     q = np.array([1, 0, 0, 0])
            q_norm = np.sqrt(np.sum(q ** 2))
        else:
            q_norm = cs.sqrt(cs.sumsqr(q))
        return 1 / q_norm * q

    def __init__(self):
        super().__init__('traj_pub')
        self.publisher_ = self.create_publisher(Odometry, 'desired_pose', 10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.df = pd.read_csv('/home/elie/ros_ws/src/trajectory_publisher/trajectory_publisher/measX.csv',delimiter=',',index_col=False)
        self.get_logger().info('Read the .csv file')
        self.i = 0



    def timer_callback(self):
        self.get_logger().info('Callback function initiated')
        
        if not self.df.empty:
            
            # Taking the first row of data
            row = self.df.loc[0] 

            # Declaring the variable of type nav_msgs/odometry (the desired pose message)
            msg = Odometry()

            # Adding a time stamp and a frame ID to the desired pose message
            msg.header.stamp = self._clock.now().to_msg()
            msg.header.frame_id = "desired trajectory"
            
            # storing the linear position data in the desired pose message
            msg.pose.pose.position.x = 0.0
            msg.pose.pose.position.y = row.y
            msg.pose.pose.position.z = row.z

            # Finding and storing the angular position data in the desired pose message
            q = self.euler_to_quaternion(row.phi,0.0,0.0)
            
            # normalizing the quaternion
            q = self.unit_quat(q)

            self.get_logger().info('The value of q is:')
            self.get_logger().info(f'qw = {q[0]}, qx = {q[1]}, qy = {q[2]}, qz = {q[3]}')
            
            

            # storing the quaternion in the desired pose message
            msg.pose.pose.orientation.w = q[0]
            msg.pose.pose.orientation.x = q[1]
            msg.pose.pose.orientation.y = q[2]
            msg.pose.pose.orientation.z = q[3]

            # storing the linear twist in the desired pose message
            msg._twist._twist._linear._x = 0.0
            msg._twist._twist._linear._y = row.vy
            msg._twist._twist._linear._z = row.vz

            # storing the angular twist in the desired pose message
            msg._twist._twist._angular._x = row.phi_dot
            msg._twist._twist._angular._y = 0.0
            msg._twist._twist._angular._z = 0.0

            # publishing the desired pose message
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing..............: "%s"' % self.i)
            
            self.df.drop(self.df.index[0], inplace=True) # deleting the row from the dataset 
            self.df = self.df.reset_index(drop = True) # resetting the indices
            self.i += 1
        else :
            self.get_logger().info('Finished publishing waypoints!')




def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()

    rclpy.spin(trajectory_publisher)
    trajectory_publisher.get_logger().info('Shutting down node!')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
