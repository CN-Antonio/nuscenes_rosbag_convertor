import rclpy

from nuscenes_player.convertor import Nuscenes_Node

def main(args=None):
    rclpy.init(args=args)
    node = Nuscenes_Node("nuscenes_Node")
    node.convert_scene(1)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()