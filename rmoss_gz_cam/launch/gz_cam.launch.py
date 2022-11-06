# Copyright 2021 RoboMaster-OSS
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

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_name = '/world/default/model/standard_robot_red1' + \
        '/link/front_industrial_camera/sensor/front_industrial_camera'
    gz_camera_image_topic = camera_name + '/image'
    gz_camera_info_topic = camera_name + '/camera_info'
    return LaunchDescription([
        Node(package='rmoss_gz_cam',
             executable='gz_cam',
             parameters=[
                {'gz_camera_image_topic': gz_camera_image_topic,
                 'gz_camera_info_topic': gz_camera_info_topic,
                 'camera_name': 'front_camera',
                 'fps': 30}],
             output='screen')
    ])
