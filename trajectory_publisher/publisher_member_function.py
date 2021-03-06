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
import numpy as np
import pandas as pd

from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose





class MinimalPublisher(Node):


    def euler_to_quaternion(self, roll, pitch, yaw):

            qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            return [qx, qy, qz, qw]

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Pose, 'pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.df = pd.read_csv('/home/elie/colcon_ws/src/trajectory_publisher/trajectory_publisher/traj.csv',delimiter=',',index_col=False)
        self.get_logger().debug('Read the .csv file')
        self.i = 0



    def timer_callback(self):
        self.get_logger().debug('Callback function intiated')
        pos = Pose()
        if not self.df.empty:
            row = self.df.loc[0] # taking the first row of data
            
            # storing the linear position data in the Pose message
            pos.position.x = row.x
            pos.position.y = row.y
            pos.position.z = row.z

            # Finding and storing the angular position data in the Pose message
            q = self.euler_to_quaternion(0.0,0.0,row.psi)
            pos.orientation.x = q[0]
            pos.orientation.y = q[1]
            pos.orientation.z = q[2]
            pos.orientation.w = q[3]

            self.publisher_.publish(pos)
            self.get_logger().info('Publishing..............: "%s"' % self.i)
            #self.get_logger().info( pos.position.x )
            
            self.df.drop(self.df.index[0], inplace=True) # deleting the row from the dataset 
            self.df = self.df.reset_index(drop = True) # resetting the indices
            self.i += 1
        else :
            self.get_logger().info('Finished publishing waypoints!')






def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.get_logger().info('Shutting down node!')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
