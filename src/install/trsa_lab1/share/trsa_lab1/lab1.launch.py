from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='trsa_lab1',
           executable='camera_calibration_pub',
           name='camera_calibration_pub',
           output='screen'
       ),
       Node(
           package='trsa_lab1',
           executable='image_reader',
           name='image_reader',
           output='screen'
       ),
       Node(
           package='trsa_lab1',
           executable='image_rectifier',
           name='image_rectifier',
           output='screen'
       ),
       Node(
           package='trsa_lab1',
           executable='image_reader_rectified',
           name='image_reader_rectified',
           output='screen'
       )
   ])