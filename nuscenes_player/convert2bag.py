import rclpy

from nuscenes_player.convertor import Nuscenes_Node

def main(args=None):
    rclpy.init(args=args)
    node = Nuscenes_Node("nuscenes_Node")
    id = node.get_parameter("scene_index").value
    print(id)
    node.convert_scene(id)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()
