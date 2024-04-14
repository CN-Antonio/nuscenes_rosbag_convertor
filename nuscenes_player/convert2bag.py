import os

# ROS
import rclpy
from rclpy.node import Node
import rclpy.time
import rosbag2_py
from rclpy.serialization import serialize_message
# ROS msg
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped

# nuscenes
from nuscenes.nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.can_bus.can_bus_api import NuScenesCanBus

###############################################
class Nuscenes_Node(Node):
    def __init__(self, name="nuscenes_node"):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        
        self.get_logger().info("Starting Nuscenes Visualization Node.")

        self.read_params()

        # NuScenes Init
        self.nusc = NuScenes(version=self.nuscenes_version, 
                             dataroot=self.nuscenes_dir, 
                             verbose=True)
        self.nusc.list_scenes()

    def read_params(self):
        self.nuscenes_dir = self.get_parameter_or(
            '~NUSCENES_DIR', rclpy.Parameter('~NUSCENES_DIR', rclpy.Parameter.Type.STRING, '/home/antonio/Data/nuscenes/Full_dataset_v1.0/mini/')).value
        self.nuscenes_version = self.get_parameter_or(
            '~NUSCENES_VER', rclpy.Parameter('~NUSCENES_VER', rclpy.Parameter.Type.STRING, 'v1.0-mini')).value
        
        self.get_logger().info("dir: %s, ver: %s" %
                           (str(self.nuscenes_dir),
                            str(self.nuscenes_version),))
        
    def unix_us2time(self, data):
        # seconds, microsecond 1000_000_000
        secs, msecs = divmod(data, 1_000_000)
        nsecs = msecs * 1000
        t = rclpy.time.Time(seconds=secs, nanoseconds=nsecs)

        return t
    
    def write_camera(self, sample_data_token):
        sample_data = self.nusc.get('sample_data', sample_data_token)
        sensor_id = sample_data['channel']
        topic = '/' + sensor_id
        msg = CompressedImage()
        msg.header.frame_id = sensor_id
        msg.format = "jpeg"

        # while sample_data is not None:
        jpg_filename = 'data/' + sample_data['filename']
        msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        with open(jpg_filename, 'rb') as jpg_file:
            msg.data = jpg_file.read()

        # write jpg
        self.writer.write(
            topic,
            serialize_message(msg),
            self.unix_us2time(sample_data['timestamp']).nanoseconds
        )

        # write info
        calib = self.nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])

        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.height = sample_data['height']
        msg.width = sample_data['width']
        msg.k[0] = calib['camera_intrinsic'][0][0]
        msg.k[1] = calib['camera_intrinsic'][0][1]
        msg.k[2] = calib['camera_intrinsic'][0][2]
        msg.k[3] = calib['camera_intrinsic'][1][0]
        msg.k[4] = calib['camera_intrinsic'][1][1]
        msg.k[5] = calib['camera_intrinsic'][1][2]
        msg.k[6] = calib['camera_intrinsic'][2][0]
        msg.k[7] = calib['camera_intrinsic'][2][1]
        msg.k[8] = calib['camera_intrinsic'][2][2]
        
        msg.R[0] = 1
        msg.R[3] = 1
        msg.R[6] = 1
        
        msg.P[0] = msg.K[0]
        msg.P[1] = msg.K[1]
        msg.P[2] = msg.K[2]
        msg.P[3] = 0
        msg.P[4] = msg.K[3]
        msg.P[5] = msg.K[4]
        msg.P[6] = msg.K[5]
        msg.P[7] = 0
        msg.P[8] = 0
        msg.P[9] = 0
        msg.P[10] = 1
        msg.P[11] = 0

            

            # last frame in scene
            # if sample_data['next'] == '': 
            #     sample_data = None
            #     # break
            # # next frame is key
            # elif self.nusc.get('sample_data', sample_data['next'])['is_key_frame'] == 'True': 
            #     sample_data = None
            #     # break
            # else:
            #     sample_data = self.nusc.get('sample_data', sample_data['next']) 

            
        
        # test
        # msg = String()
        # msg.data = sample_data['filename']
        # self.writer.write(
        #     'testString',   # 对应topic_info.name
        #     serialize_message(msg),
        #     self.unix_us2time(sample_data['timestamp']).nanoseconds
        # )

    def convert_scene(self, scene_i):
        # certain scene
        scene = self.nusc.scene[scene_i]
        scene_name = scene['name']

        cur_sample = self.nusc.get('sample', scene['first_sample_token'])

        # rosbag metadata
        bag_name = f'NuScenes-{self.nuscenes_version}-{scene_name}.bag'
        bag_path = os.path.join(os.path.abspath(os.curdir), bag_name)
        
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri= bag_name,
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        self.get_logger().info(f'Writing to {bag_path}')

        # create topic start
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/pose',
                type='geometry_msgs/msg/PoseStamped',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        channels = ['CAM_BACK',  'CAM_FRONT', 'CAM_FRONT_LEFT',
                    'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT', 'CAM_BACK_LEFT']
        for channel in channels:
            topic_info = rosbag2_py._storage.TopicMetadata(
                name='/' + channel + '/image_rect_compressed',
                type='sensor_msgs/msg/CompressedImage',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            topic_info = rosbag2_py._storage.TopicMetadata(
                name='/' + channel + '/camera_info',
                type='sensor_msgs/msg/CameraInfo',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)

        # stamp = get_time(self.nusc.get('ego_pose', self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])['ego_pose_token']))
        stamp = self.unix_us2time(self.nusc.get('ego_pose', self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])['ego_pose_token'])['timestamp'])
        # map_msg
        # centerlines_msg
        last_map_stamp = stamp

        # test bag write start
        # topic_info = rosbag2_py._storage.TopicMetadata(
        #     name='testString',
        #     type='std_msgs/msg/String',
        #     serialization_format='cdr')
        # self.writer.create_topic(topic_info)

        # string = 'Hello World_'
        # msg = String()
        # msg.data = string

        # for i in range(10):
        #     msg.data = string+str(i)
        #     self.writer.write(
        #         'testString',   # 对应topic_info.name
        #         serialize_message(msg),
        #         self.get_clock().now().nanoseconds
        #         # rclpy.time.Time()
        #     )
        #     time.sleep(0.5)
        # test end

        # iterate sample
        # cur_sample = self.nusc.get('sample', scene['first_sample_token'])
        while cur_sample is not None:

            # publish /tf
            
            # iterate sensors
            for (sensor_id, sample_data_token) in cur_sample['data'].items():
                sample_data = self.nusc.get('sample_data', sample_data_token)
                # topic = '/' + sensor_id
                # print(sensor_id)

                # write the sensor data
                if sample_data['sensor_modality'] == 'radar':
                    pass
                elif sample_data['sensor_modality'] == 'camera':
                    self.write_camera(sample_data_token) # key_frame & none_key_frame
                    # msg = get_camera(sample_data, sensor_id)
                    # time.sleep(0.05)

            # publish /pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base_link'
            pose_stamped.header.stamp = stamp.to_msg()
            pose_stamped.pose.orientation.w = 1.0
            # bag.write('/pose', pose_stamped, stamp)
            self.writer.write(
                '/pose',
                serialize_message(pose_stamped),
                stamp.nanoseconds
            )

            # move to the next sample
            cur_sample = self.nusc.get('sample', cur_sample['next']) if cur_sample.get('next') != '' else None

        self.get_logger().info(f'Finished writing {bag_name}')


###############################################
# NUSCENES_VERSION = 'v1.0-mini'
# nusc = NuScenes(version=NUSCENES_VERSION, dataroot='/home/antonio/Project/nuscenes_player/data', verbose=True)
# # nusc_can = NuScenesCanBus(dataroot='data')
# nusc.list_scenes()

# Utils #######################################
EARTH_RADIUS_METERS = 6.378137e6
REFERENCE_COORDINATES = {
    "boston-seaport": [42.336849169438615, -71.05785369873047],
    "singapore-onenorth": [1.2882100868743724, 103.78475189208984],
    "singapore-hollandvillage": [1.2993652317780957, 103.78217697143555],
    "singapore-queenstown": [1.2782562240223188, 103.76741409301758],
}

def get_time(data):
    t = rclpy.time.Time()
    t.seconds, msecs = divmod(data['timestamp'], 1_000_000) # s, ms
    print(data['timestamp'], t.seconds, msecs)
    t.nanoseconds = msecs * 1000                            # ns

    return t

def get_camera(sample_data, frame_id):
    jpg_filename = 'data/' + sample_data['filename']
    msg = CompressedImage()
    msg.header.frame_id = frame_id
    msg.header.stamp = get_time(sample_data)
    msg.format = "jpeg"
    with open(jpg_filename, 'rb') as jpg_file:
        msg.data = jpg_file.read()
    return msg

def get_camera_info(sample_data, frame_id):
    calib = nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])

    msg_info = CameraInfo()
    msg_info.header.frame_id = frame_id
    msg_info.header.stamp = get_time(sample_data)
    msg_info.height = sample_data['height']
    msg_info.width = sample_data['width']
    msg_info.K[0] = calib['camera_intrinsic'][0][0]
    msg_info.K[1] = calib['camera_intrinsic'][0][1]
    msg_info.K[2] = calib['camera_intrinsic'][0][2]
    msg_info.K[3] = calib['camera_intrinsic'][1][0]
    msg_info.K[4] = calib['camera_intrinsic'][1][1]
    msg_info.K[5] = calib['camera_intrinsic'][1][2]
    msg_info.K[6] = calib['camera_intrinsic'][2][0]
    msg_info.K[7] = calib['camera_intrinsic'][2][1]
    msg_info.K[8] = calib['camera_intrinsic'][2][2]
    
    msg_info.R[0] = 1
    msg_info.R[3] = 1
    msg_info.R[6] = 1
    
    msg_info.P[0] = msg_info.K[0]
    msg_info.P[1] = msg_info.K[1]
    msg_info.P[2] = msg_info.K[2]
    msg_info.P[3] = 0
    msg_info.P[4] = msg_info.K[3]
    msg_info.P[5] = msg_info.K[4]
    msg_info.P[6] = msg_info.K[5]
    msg_info.P[7] = 0
    msg_info.P[8] = 0
    msg_info.P[9] = 0
    msg_info.P[10] = 1
    msg_info.P[11] = 0
    return msg_info

###############################################
def convert_scene(scene):
    scene_name = scene['name']
    # log = nusc.get('log', scene['log_token'])
    # location = log['location']
    # print(f'Loading map "{location}"')
    # nusc_map = NuScenesMap(dataroot='data', map_name=location)
    # print(f'Loading bitmap "{nusc_map.map_name}"')
    # bitmap = BitMap(nusc_map.dataroot, nusc_map.map_name, 'basemap')
    # print(f'Loaded {bitmap.image.shape} bitmap')
    
    cur_sample = nusc.get('sample', scene['first_sample_token'])

    bag_name = f'NuScenes-{NUSCENES_VERSION}-{scene_name}.bag'
    bag_path = os.path.join(os.path.abspath(os.curdir), bag_name)
    print(f'Writing to {bag_path}')
    # bag = rosbag.Bag(bag_path, 'w', compression='lz4')
    writer = rosbag2_py.SequentialWriter()

    storage_options = rosbag2_py._storage.StorageOptions(
        uri= bag_name,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')
    writer.open(storage_options, converter_options)

    topic_info = rosbag2_py._storage.TopicMetadata(
        name='chatter',
        type='std_msgs/msg/String',
        serialization_format='cdr')
    writer.create_topic(topic_info)

    msg = String()
    msg.data = 'Hello World'

    writer.write(
        'chatter',
        serialize_message(msg),
        r
    )

    while cur_sample is not None:
        sample_lidar = nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])
        ego_pose = nusc.get('ego_pose', sample_lidar['ego_pose_token'])
        # stamp = get_time(ego_pose)

        # iterate sensors
        for (sensor_id, sample_token) in cur_sample['data'].items():
            sample_data = nusc.get('sample_data', sample_token)
            topic = '/' + sensor_id

            # write the sensor data
            # if sample_data['sensor_modality'] == 'radar':
            #     msg = get_radar(sample_data, sensor_id)
            #     bag.write(topic, msg, stamp)
            # elif sample_data['sensor_modality'] == 'lidar':
            #     msg = get_lidar(sample_data, sensor_id)
            #     bag.write(topic, msg, stamp)
            if sample_data['sensor_modality'] == 'camera':
                msg = get_camera(sample_data, sensor_id)
            #     bag.write(topic + '/image_rect_compressed', msg, stamp)
                msg = get_camera_info(sample_data, sensor_id)
            #     bag.write(topic + '/camera_info', msg, stamp)

            # if sample_data['sensor_modality'] == 'camera':
            #     msg = get_lidar_imagemarkers(sample_lidar, sample_data, sensor_id)
            #     bag.write(topic + '/image_markers_lidar', msg, stamp)
            #     write_boxes_imagemarkers(bag, cur_sample['anns'], sample_data, sensor_id, topic, stamp)

        # collect all sensor frames after this sample but before the next sample
        non_keyframe_sensor_msgs = []

        # sort and publish the non-keyframe sensor msgs
        non_keyframe_sensor_msgs.sort(key=lambda x: x[0])
        for (_, topic, msg) in non_keyframe_sensor_msgs:
            # bag.write(topic, msg, msg.header.stamp)
            pass

        # move to the next sample
        cur_sample = nusc.get('sample', cur_sample['next']) if cur_sample.get('next') != '' else None

    # bag.close()
    print(f'Finished writing {bag_name}')

#################################################
def main(args=None):
    rclpy.init(args=args)
    node = Nuscenes_Node("nuscenes_Node")
    node.convert_scene(0)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()