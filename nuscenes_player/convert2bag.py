import os

# ROS
import rclpy
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import rosbag2_py
from rclpy.serialization import serialize_message
# ROS msg
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import Point, PoseStamped, Transform, TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import ImageMarker, Marker, MarkerArray

# nuscenes
from nuscenes.nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.eval.common.utils import quaternion_yaw

# from nuscenes_player.recorder import Nuscenes_Node
from nuscenes_player.bitmap import BitMap

###############################################
class Nuscenes_Node(Node):
    def __init__(self, name="nuscenes_node"):
        super().__init__(name,
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)
        
        self.get_logger().info("Starting Nuscenes Visualization Node.")

        self.read_params()

        # NuScenes Init
        self.EARTH_RADIUS_METERS = 6.378137e6
        self.REFERENCE_COORDINATES = {
            "boston-seaport": [42.336849169438615, -71.05785369873047],
            "singapore-onenorth": [1.2882100868743724, 103.78475189208984],
            "singapore-hollandvillage": [1.2993652317780957, 103.78217697143555],
            "singapore-queenstown": [1.2782562240223188, 103.76741409301758],
        }
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
    
    def scene_bounding_box(self, scene, nusc_map, padding=75.0):
        box = [np.inf, np.inf, -np.inf, -np.inf]
        cur_sample = self.nusc.get('sample', scene['first_sample_token'])
        while cur_sample is not None:
            sample_lidar = self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])
            ego_pose = self.nusc.get('ego_pose', sample_lidar['ego_pose_token'])
            x, y = ego_pose['translation'][:2]
            box[0] = min(box[0], x)
            box[1] = min(box[1], y)
            box[2] = max(box[2], x)
            box[3] = max(box[3], y)
            cur_sample = self.nusc.get('sample', cur_sample['next']) if cur_sample.get('next') != '' else None
        box[0] = max(box[0] - padding, 0.0)
        box[1] = max(box[1] - padding, 0.0)
        box[2] = min(box[2] + padding, nusc_map.canvas_edge[0]) - box[0]
        box[3] = min(box[3] + padding, nusc_map.canvas_edge[1]) - box[1]
        return box

    def rectContains(self, rect, point):
        a, b, c, d = rect
        x, y = point[:2]
        return a <= x < a + c and b <= y < b + d

    def derive_latlon(self, location: str, pose: Dict[str, float]):
        """
        For each pose value, extract its respective lat/lon coordinate and timestamp.
        
        This makes the following two assumptions in order to work:
            1. The reference coordinate for each map is in the south-western corner.
            2. The origin of the global poses is also in the south-western corner (and identical to 1).
        :param location: The name of the map the poses correspond to, ie: 'boston-seaport'.
        :param poses: All nuScenes egopose dictionaries of a scene.
        :return: A list of dicts (lat/lon coordinates and timestamps) for each pose.
        """
        assert location in self.REFERENCE_COORDINATES.keys(), \
            f'Error: The given location: {location}, has no available reference.'

        coordinates = []
        reference_lat, reference_lon = self.REFERENCE_COORDINATES[location]
        ts = pose['timestamp']
        x, y = pose['translation'][:2]
        bearing = math.atan(x / y)
        distance = math.sqrt(x**2 + y**2)
        lat, lon = self.get_coordinate(reference_lat, reference_lon, bearing, distance)
        return {'latitude': lat, 'longitude': lon}

    # get ##################################################################
    def get_scene_map(self, scene, nusc_map, bitmap, stamp):
        x, y, w, h = self.scene_bounding_box(scene, nusc_map)
        img_x = int(x * 10)
        img_y = int(y * 10)
        img_w = int(w * 10)
        img_h = int(h * 10)
        img = np.flipud(bitmap.image)[img_y:img_y+img_h, img_x:img_x+img_w]
        img = (img * (100.0 / 255.0)).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = stamp.to_msg()
        msg.info.map_load_time = stamp.to_msg()
        msg.info.resolution = 0.1
        msg.info.width = img_w
        msg.info.height = img_h
        msg.info.origin.position.x = x
        msg.info.origin.position.y = y
        msg.info.origin.orientation.w = 1.0
        msg.data = img.flatten().tolist()

        return msg

    def get_centerline_markers(self, scene, nusc_map, stamp):
        pose_lists = nusc_map.discretize_centerlines(1)
        bbox = self.scene_bounding_box(scene, nusc_map)

        contained_pose_lists = []
        for pose_list in pose_lists:
            new_pose_list = []
            for pose in pose_list:
                if self.rectContains(bbox, pose):
                    new_pose_list.append(pose)
            if len(new_pose_list) > 0:
                contained_pose_lists.append(new_pose_list)
        
        msg = MarkerArray()
        for i, pose_list in enumerate(contained_pose_lists):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp.to_msg()
            marker.ns = 'centerline'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.frame_locked = True
            marker.scale.x = 0.1
            marker.color.r = 51.0 / 255.0
            marker.color.g = 160.0 / 255.0
            marker.color.b = 44.0 / 255.0
            marker.color.a = 1.0
            marker.pose.orientation.w = 1.0
            for pose in pose_list:
                point = Point()
                point.x = pose[0]
                point.y = pose[1]
                point.z = 0.0
                marker.points.append(point)
            msg.markers.append(marker)

        return msg
    
    def get_transform(self, data):
        t = Transform()
        t.translation.x = data['translation'][0]
        t.translation.y = data['translation'][1]
        t.translation.z = data['translation'][2]
        
        t.rotation.w = data['rotation'][0]
        t.rotation.x = data['rotation'][1]
        t.rotation.y = data['rotation'][2]
        t.rotation.z = data['rotation'][3]
        
        return t

    def get_tfs(self, sample):
        sample_lidar = self.nusc.get('sample_data', sample['data']['LIDAR_TOP'])
        ego_pose = self.nusc.get('ego_pose', sample_lidar['ego_pose_token'])
        # stamp = get_time(ego_pose)
        stamp = self.unix_us2time(ego_pose['timestamp'])

        transforms = []

        # create ego transform
        ego_tf = TransformStamped()
        ego_tf.header.frame_id = 'map'
        ego_tf.header.stamp = stamp.to_msg()
        ego_tf.child_frame_id = 'base_link'
        ego_tf.transform = self.get_transform(ego_pose)
        transforms.append(ego_tf)

        for (sensor_id, sample_token) in sample['data'].items():
            sample_data = self.nusc.get('sample_data', sample_token)

            # create sensor transform
            sensor_tf = TransformStamped()
            sensor_tf.header.frame_id = 'base_link'
            sensor_tf.header.stamp = stamp.to_msg()
            sensor_tf.child_frame_id = sensor_id
            sensor_tf.transform = self.get_transform(
                self.nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token']))
            transforms.append(sensor_tf)

        return transforms

    def get_tfmessage(self, sample):
        # get transforms for the current sample
        tf_array = TFMessage()
        tf_array.transforms = self.get_tfs(sample)

        # add transforms from the next sample to enable interpolation
        next_sample = self.nusc.get('sample', sample['next']) if sample.get('next') != '' else None
        if next_sample is not None:
            tf_array.transforms += self.get_tfs(next_sample)

        return tf_array

    def get_lidar(self, sample_data, frame_id):
        pc_filename = 'data/' + sample_data['filename']
        pc_filesize = os.stat(pc_filename).st_size

        with open(pc_filename, 'rb') as pc_file:
            msg = PointCloud2()
            msg.header.frame_id = frame_id
            # msg.header.stamp = get_time(sample_data)
            msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()

            msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='ring', offset=16, datatype=PointField.FLOAT32, count=1),
            ]

            msg.is_bigendian = False
            msg.is_dense = True
            msg.point_step = len(msg.fields) * 4 # 4 bytes per field
            msg.row_step = pc_filesize
            msg.width = round(pc_filesize / msg.point_step)
            msg.height = 1 # unordered
            msg.data = pc_file.read()
            return msg

    def get_camera(self, sample_data, frame_id):
        jpg_filename = 'data/' + sample_data['filename']
        msg = CompressedImage()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        msg.format = "jpeg"
        with open(jpg_filename, 'rb') as jpg_file:
            msg.data = jpg_file.read()
        return msg

    def get_camera_info(self, sample_data, frame_id):
        calib = self.nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])

        msg = CameraInfo()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
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
        
        msg.r[0] = 1
        msg.r[3] = 1
        msg.r[6] = 1
        
        msg.p[0] = msg.k[0]
        msg.p[1] = msg.k[1]
        msg.p[2] = msg.k[2]
        msg.p[3] = 0
        msg.p[4] = msg.k[3]
        msg.p[5] = msg.k[4]
        msg.p[6] = msg.k[5]
        msg.p[7] = 0
        msg.p[8] = 0
        msg.p[9] = 0
        msg.p[10] = 1
        msg.p[11] = 0

        return msg

    def get_coordinate(self, ref_lat: float, ref_lon: float, bearing: float, dist: float) -> Tuple[float, float]:
        """
        Using a reference coordinate, extract the coordinates of another point in space given its distance and bearing
        to the reference coordinate. For reference, please see: https://www.movable-type.co.uk/scripts/latlong.html.
        :param ref_lat: Latitude of the reference coordinate in degrees, ie: 42.3368.
        :param ref_lon: Longitude of the reference coordinate in degrees, ie: 71.0578.
        :param bearing: The clockwise angle in radians between target point, reference point and the axis pointing north.
        :param dist: The distance in meters from the reference point to the target point.
        :return: A tuple of lat and lon.
        """
        lat, lon = math.radians(ref_lat), math.radians(ref_lon)
        angular_distance = dist / self.EARTH_RADIUS_METERS

        target_lat = math.asin(
            math.sin(lat) * math.cos(angular_distance) + 
            math.cos(lat) * math.sin(angular_distance) * math.cos(bearing)
        )
        target_lon = lon + math.atan2(
            math.sin(bearing) * math.sin(angular_distance) * math.cos(lat),
            math.cos(angular_distance) - math.sin(lat) * math.sin(target_lat)
        )
        return math.degrees(target_lat), math.degrees(target_lon)

    def derive_latlon(self, location: str, pose: Dict[str, float]):
        """
        For each pose value, extract its respective lat/lon coordinate and timestamp.
        
        This makes the following two assumptions in order to work:
            1. The reference coordinate for each map is in the south-western corner.
            2. The origin of the global poses is also in the south-western corner (and identical to 1).
        :param location: The name of the map the poses correspond to, ie: 'boston-seaport'.
        :param poses: All nuScenes egopose dictionaries of a scene.
        :return: A list of dicts (lat/lon coordinates and timestamps) for each pose.
        """
        assert location in self.REFERENCE_COORDINATES.keys(), \
            f'Error: The given location: {location}, has no available reference.'

        coordinates = []
        reference_lat, reference_lon = self.REFERENCE_COORDINATES[location]
        ts = pose['timestamp']
        x, y = pose['translation'][:2]
        bearing = math.atan(x / y)
        distance = math.sqrt(x**2 + y**2)
        lat, lon = self.get_coordinate(reference_lat, reference_lon, bearing, distance)
        return {'latitude': lat, 'longitude': lon}

    def convert_scene(self, scene_i):
        # certain scene
        scene = self.nusc.scene[scene_i]
        scene_name = scene['name']
        log = self.nusc.get('log', scene['log_token'])
        location = log['location']
        print(f'Loading map "{location}"')
        nusc_map = NuScenesMap(dataroot='data', map_name=location)
        print(f'Loading bitmap "{nusc_map.map_name}"')
        bitmap = BitMap(nusc_map.dataroot, nusc_map.map_name, 'basemap')
        print(f'Loaded {bitmap.image.shape} bitmap')

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
        # /map
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/map',
                type='nav_msgs/msg/OccupancyGrid',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # /semantic_map
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/semantic_map',
                type='visualization_msgs/msg/MarkerArray',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # /pose,TODO: /odom, and /diagnostics
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/pose',
                type='geometry_msgs/msg/PoseStamped',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # /tf
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/tf',
                type='tf2_msgs/msg/TFMessage',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # /drivable_area
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/drivable_area',
                type='tf2_msgs/msg/TFMessage',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # TODO: sensors
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/LIDAR_TOP',
            type='sensor_msgs/msg/PointCloud2',
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
        # /gps
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/gps',
                type='sensor_msgs/msg/NavSatFix',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)

        # stamp = get_time(self.nusc.get('ego_pose', self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])['ego_pose_token']))
        stamp = self.unix_us2time(self.nusc.get('ego_pose', self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])['ego_pose_token'])['timestamp'])
        map_msg = self.get_scene_map(scene, nusc_map, bitmap, stamp)
        centerlines_msg = self.get_centerline_markers(scene, nusc_map, stamp)
        self.writer.write(
            '/map',
            serialize_message(map_msg),
           stamp.nanoseconds
        )
        self.writer.write(
            '/semantic_map',
            serialize_message(centerlines_msg),
            stamp.nanoseconds
        )
        last_map_stamp = stamp

        # iterate sample
        # cur_sample = self.nusc.get('sample', scene['first_sample_token'])
        while cur_sample is not None:
            sample_lidar = self.nusc.get('sample_data', cur_sample['data']['LIDAR_TOP'])
            ego_pose = self.nusc.get('ego_pose', sample_lidar['ego_pose_token'])
            # stamp = get_time(ego_pose)
            stamp = self.unix_us2time(ego_pose['timestamp'])
            
            # write map topics every two seconds
            if stamp - rclpy.duration.Duration(seconds=2.0) >= last_map_stamp:
                map_msg.header.stamp = stamp.to_msg()
                for marker in centerlines_msg.markers:
                    marker.header.stamp = stamp.to_msg()
                self.writer.write(
                    '/map',
                    serialize_message(map_msg),
                stamp.nanoseconds
                )
                self.writer.write(
                    '/semantic_map',
                    serialize_message(centerlines_msg),
                    stamp.nanoseconds
                )
                last_map_stamp = stamp
            
            # publish /tf
            tf_array = self.get_tfmessage(cur_sample)
            self.writer.write(
                '/tf',
                serialize_message(tf_array),
                stamp.nanoseconds
            )

            # /driveable_area occupancy grid
            # self.write_occupancy_grid(bag, nusc_map, ego_pose, stamp)
            
            # TODO: iterate sensors
            for (sensor_id, sample_data_token) in cur_sample['data'].items():
                sample_data = self.nusc.get('sample_data', sample_data_token)
                topic = '/' + sensor_id

                # write the sensor data
                if sample_data['sensor_modality'] == 'radar':
                    pass
                elif sample_data['sensor_modality'] == 'lidar':
                    msg = self.get_lidar(sample_data, sensor_id)
                    self.writer.write(
                        topic,
                        serialize_message(msg),
                        stamp.nanoseconds
                    )
                elif sample_data['sensor_modality'] == 'camera':
                    # self.write_camera(sample_data_token) # key_frame & none_key_frame
                    msg = self.get_camera(sample_data, sensor_id)
                    # bag.write(topic + '/image_rect_compressed', msg, stamp)
                    self.writer.write(
                        topic + '/image_rect_compressed',
                        serialize_message(msg),
                        stamp.nanoseconds
                    )
                    msg = self.get_camera_info(sample_data, sensor_id)
                    # bag.write(topic + '/camera_info', msg, stamp)
                    self.writer.write(
                        topic + '/camera_info',
                        serialize_message(msg),
                        stamp.nanoseconds
                    )

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

            # publish /gps
            coordinates = self.derive_latlon(location, ego_pose)
            gps = NavSatFix()
            gps.header.frame_id = 'base_link'
            gps.header.stamp = stamp.to_msg()
            gps.status.status = 1
            gps.status.service = 1
            gps.latitude = coordinates['latitude']
            gps.longitude = coordinates['longitude']
            gps.altitude = self.get_transform(ego_pose).translation.z
            # bag.write('/gps', gps, stamp)
            self.writer.write(
                '/gps',
                serialize_message(gps),
                stamp.nanoseconds
            )

            # collect all sensor frames after this sample but before the next sample

            # sort and publish the non-keyframe sensor msgs

            # move to the next sample
            cur_sample = self.nusc.get('sample', cur_sample['next']) if cur_sample.get('next') != '' else None

        self.get_logger().info(f'Finished writing {bag_name}')

#################################################
def main(args=None):
    rclpy.init(args=args)
    node = Nuscenes_Node("nuscenes_Node")
    node.convert_scene(0)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()