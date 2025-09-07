from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch import LaunchContext

def generate_launch_description():
    ld = LaunchDescription()
    
    face_detection_pkg_path = FindPackagePrefix('face_detection')
    face_cascade_name_0 = PathJoinSubstitution([face_detection_pkg_path, 'include', 'face_detection', 'HaarCascades', 'haarcascade_frontalface_alt.xml'])
    face_cascade_name_1 = PathJoinSubstitution([face_detection_pkg_path, 'include', 'face_detection', 'HaarCascades', 'haarcascade_frontalface_alt2.xml'])
    face_cascade_name_2 = PathJoinSubstitution([face_detection_pkg_path, 'include', 'face_detection', 'HaarCascades', 'haarcascade_frontalface_alt_tree.xml'])
    face_cascade_name_3 = PathJoinSubstitution([face_detection_pkg_path, 'include', 'face_detection', 'HaarCascades', 'haarcascade_frontalface_default.xml'])
    face_cascade_name_4 = PathJoinSubstitution([face_detection_pkg_path, 'include', 'face_detection', 'lbpCascades', 'lbpcascade_frontalface.xml'])


    face_tracking_node = Node(
            package='face_detection', 
            executable='face_tracking', 
            output='screen',
            parameters=[{'imageInput': '/usb_cam_node/image_raw/compressed'},
                        {'imageOutput': '/facerec/image_raw'},
                        {'displayed_Image': 0}, 
                        {'publish': 3},
                        {'start_condition': False}],
            arguments=[face_cascade_name_0,
                       face_cascade_name_1,
                       face_cascade_name_2,
                       face_cascade_name_3,
                       face_cascade_name_4]
    )
   
    ld.add_action(face_tracking_node)

    return ld
