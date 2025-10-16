from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='trsa_lab1',
           executable='camera_driver',
           name='camera_driver',
       ),
       Node(
           package='trsa_lab1',
           executable='image_reader',
           name='image_reader',
       ),
       Node(
           package='trsa_lab1',
           executable='object_detection',
           name='object_detection',
       ),
       Node(
           package='trsa_lab1',
           executable='image_rectifier',
           name='image_rectifier',
       ),
       Node(
           package='trsa_lab1',
           executable='image_convert',
           name='image_convert',
       )
   ])