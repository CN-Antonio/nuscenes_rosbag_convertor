import os
import math
from typing import Tuple, Dict
import numpy as np
import cv2
# from pypcd import numpy_pc2, pypcd
from pyquaternion import Quaternion
# ROS
from rclpy.node import Node
import rclpy.time
import rclpy.duration
import rosbag2_py
from rclpy.serialization import serialize_message
# ROS msg
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import CameraInfo, Image, CompressedImage, Imu, NavSatFix, PointCloud2, PointField
from geometry_msgs.msg import Point, Pose, PoseStamped, Transform, TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import ImageMarker, Marker, MarkerArray

from nuscenes_player.bitmap import BitMap

# nuscenes
from nuscenes.nuscenes import NuScenes
from nuscenes.map_expansion.map_api import NuScenesMap
# from nuscenes.can_bus.can_bus_api import NuScenesCanBus
from nuscenes.eval.common.utils import quaternion_yaw

class Nuscenes_Node(Node):
    def __init__(self, name="nuscenes_node"):
        super().__init__(name,
                         allow_undeclared_parameters=False)
                        #  automatically_declare_parameters_from_overrides=True)
        
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
        self.declare_parameter('~NUSCENES_DIR', '/home/antonio/Data/nuscenes/Full_dataset_v1.0/mini/')
        self.declare_parameter('~NUSCENES_VER', 'v1.0-mini')
        self.declare_parameter('dataset_index', 0)
        self.declare_parameter('convert_RGBImage', 0)

        self.nuscenes_dir = self.get_parameter("~NUSCENES_DIR").value
        self.nuscenes_version = self.get_parameter("~NUSCENES_VER").value

        # self.nuscenes_dir = self.get_parameter_or(
        #     '~NUSCENES_DIR', rclpy.Parameter('~NUSCENES_DIR', rclpy.Parameter.Type.STRING, '/home/antonio/Data/nuscenes/Full_dataset_v1.0/mini/')).value
        # self.nuscenes_version = self.get_parameter_or(
        #     '~NUSCENES_VER', rclpy.Parameter('~NUSCENES_VER', rclpy.Parameter.Type.STRING, 'v1.0-mini')).value
        
        self.get_logger().info("dir: %s, ver: %s" %
                           (str(self.nuscenes_dir),
                            str(self.nuscenes_version),))
        
    def unix_us2time(self, data):
        # seconds, microsecond
        secs, msecs = divmod(data, 1_000_000)
        nsecs = msecs * 1000
        t = rclpy.time.Time(seconds=secs, nanoseconds=nsecs)

        return t
    
    # def make_point(xyz):
    #     p = Point()
    #     p.x = xyz[0]
    #     p.y = xyz[1]
    #     p.z = xyz[2]
    #     return p
    
    def make_point2d(self, xy):
        p = Point()
        p.x = xy[0]
        p.y = xy[1]
        p.z = 0.0
        return p

    def make_color(self, rgb, a=1.0):
        c = ColorRGBA()
        c.r = rgb[0]
        c.g = rgb[1]
        c.b = rgb[2]
        c.a = a
        return c
    
    def turbomap(self, x):
        turbo_colormap_data = [[0.18995,0.07176,0.23217],[0.19483,0.08339,0.26149],[0.19956,0.09498,0.29024],[0.20415,0.10652,0.31844],[0.20860,0.11802,0.34607],[0.21291,0.12947,0.37314],[0.21708,0.14087,0.39964],[0.22111,0.15223,0.42558],[0.22500,0.16354,0.45096],[0.22875,0.17481,0.47578],[0.23236,0.18603,0.50004],[0.23582,0.19720,0.52373],[0.23915,0.20833,0.54686],[0.24234,0.21941,0.56942],[0.24539,0.23044,0.59142],[0.24830,0.24143,0.61286],[0.25107,0.25237,0.63374],[0.25369,0.26327,0.65406],[0.25618,0.27412,0.67381],[0.25853,0.28492,0.69300],[0.26074,0.29568,0.71162],[0.26280,0.30639,0.72968],[0.26473,0.31706,0.74718],[0.26652,0.32768,0.76412],[0.26816,0.33825,0.78050],[0.26967,0.34878,0.79631],[0.27103,0.35926,0.81156],[0.27226,0.36970,0.82624],[0.27334,0.38008,0.84037],[0.27429,0.39043,0.85393],[0.27509,0.40072,0.86692],[0.27576,0.41097,0.87936],[0.27628,0.42118,0.89123],[0.27667,0.43134,0.90254],[0.27691,0.44145,0.91328],[0.27701,0.45152,0.92347],[0.27698,0.46153,0.93309],[0.27680,0.47151,0.94214],[0.27648,0.48144,0.95064],[0.27603,0.49132,0.95857],[0.27543,0.50115,0.96594],[0.27469,0.51094,0.97275],[0.27381,0.52069,0.97899],[0.27273,0.53040,0.98461],[0.27106,0.54015,0.98930],[0.26878,0.54995,0.99303],[0.26592,0.55979,0.99583],[0.26252,0.56967,0.99773],[0.25862,0.57958,0.99876],[0.25425,0.58950,0.99896],[0.24946,0.59943,0.99835],[0.24427,0.60937,0.99697],[0.23874,0.61931,0.99485],[0.23288,0.62923,0.99202],[0.22676,0.63913,0.98851],[0.22039,0.64901,0.98436],[0.21382,0.65886,0.97959],[0.20708,0.66866,0.97423],[0.20021,0.67842,0.96833],[0.19326,0.68812,0.96190],[0.18625,0.69775,0.95498],[0.17923,0.70732,0.94761],[0.17223,0.71680,0.93981],[0.16529,0.72620,0.93161],[0.15844,0.73551,0.92305],[0.15173,0.74472,0.91416],[0.14519,0.75381,0.90496],[0.13886,0.76279,0.89550],[0.13278,0.77165,0.88580],[0.12698,0.78037,0.87590],[0.12151,0.78896,0.86581],[0.11639,0.79740,0.85559],[0.11167,0.80569,0.84525],[0.10738,0.81381,0.83484],[0.10357,0.82177,0.82437],[0.10026,0.82955,0.81389],[0.09750,0.83714,0.80342],[0.09532,0.84455,0.79299],[0.09377,0.85175,0.78264],[0.09287,0.85875,0.77240],[0.09267,0.86554,0.76230],[0.09320,0.87211,0.75237],[0.09451,0.87844,0.74265],[0.09662,0.88454,0.73316],[0.09958,0.89040,0.72393],[0.10342,0.89600,0.71500],[0.10815,0.90142,0.70599],[0.11374,0.90673,0.69651],[0.12014,0.91193,0.68660],[0.12733,0.91701,0.67627],[0.13526,0.92197,0.66556],[0.14391,0.92680,0.65448],[0.15323,0.93151,0.64308],[0.16319,0.93609,0.63137],[0.17377,0.94053,0.61938],[0.18491,0.94484,0.60713],[0.19659,0.94901,0.59466],[0.20877,0.95304,0.58199],[0.22142,0.95692,0.56914],[0.23449,0.96065,0.55614],[0.24797,0.96423,0.54303],[0.26180,0.96765,0.52981],[0.27597,0.97092,0.51653],[0.29042,0.97403,0.50321],[0.30513,0.97697,0.48987],[0.32006,0.97974,0.47654],[0.33517,0.98234,0.46325],[0.35043,0.98477,0.45002],[0.36581,0.98702,0.43688],[0.38127,0.98909,0.42386],[0.39678,0.99098,0.41098],[0.41229,0.99268,0.39826],[0.42778,0.99419,0.38575],[0.44321,0.99551,0.37345],[0.45854,0.99663,0.36140],[0.47375,0.99755,0.34963],[0.48879,0.99828,0.33816],[0.50362,0.99879,0.32701],[0.51822,0.99910,0.31622],[0.53255,0.99919,0.30581],[0.54658,0.99907,0.29581],[0.56026,0.99873,0.28623],[0.57357,0.99817,0.27712],[0.58646,0.99739,0.26849],[0.59891,0.99638,0.26038],[0.61088,0.99514,0.25280],[0.62233,0.99366,0.24579],[0.63323,0.99195,0.23937],[0.64362,0.98999,0.23356],[0.65394,0.98775,0.22835],[0.66428,0.98524,0.22370],[0.67462,0.98246,0.21960],[0.68494,0.97941,0.21602],[0.69525,0.97610,0.21294],[0.70553,0.97255,0.21032],[0.71577,0.96875,0.20815],[0.72596,0.96470,0.20640],[0.73610,0.96043,0.20504],[0.74617,0.95593,0.20406],[0.75617,0.95121,0.20343],[0.76608,0.94627,0.20311],[0.77591,0.94113,0.20310],[0.78563,0.93579,0.20336],[0.79524,0.93025,0.20386],[0.80473,0.92452,0.20459],[0.81410,0.91861,0.20552],[0.82333,0.91253,0.20663],[0.83241,0.90627,0.20788],[0.84133,0.89986,0.20926],[0.85010,0.89328,0.21074],[0.85868,0.88655,0.21230],[0.86709,0.87968,0.21391],[0.87530,0.87267,0.21555],[0.88331,0.86553,0.21719],[0.89112,0.85826,0.21880],[0.89870,0.85087,0.22038],[0.90605,0.84337,0.22188],[0.91317,0.83576,0.22328],[0.92004,0.82806,0.22456],[0.92666,0.82025,0.22570],[0.93301,0.81236,0.22667],[0.93909,0.80439,0.22744],[0.94489,0.79634,0.22800],[0.95039,0.78823,0.22831],[0.95560,0.78005,0.22836],[0.96049,0.77181,0.22811],[0.96507,0.76352,0.22754],[0.96931,0.75519,0.22663],[0.97323,0.74682,0.22536],[0.97679,0.73842,0.22369],[0.98000,0.73000,0.22161],[0.98289,0.72140,0.21918],[0.98549,0.71250,0.21650],[0.98781,0.70330,0.21358],[0.98986,0.69382,0.21043],[0.99163,0.68408,0.20706],[0.99314,0.67408,0.20348],[0.99438,0.66386,0.19971],[0.99535,0.65341,0.19577],[0.99607,0.64277,0.19165],[0.99654,0.63193,0.18738],[0.99675,0.62093,0.18297],[0.99672,0.60977,0.17842],[0.99644,0.59846,0.17376],[0.99593,0.58703,0.16899],[0.99517,0.57549,0.16412],[0.99419,0.56386,0.15918],[0.99297,0.55214,0.15417],[0.99153,0.54036,0.14910],[0.98987,0.52854,0.14398],[0.98799,0.51667,0.13883],[0.98590,0.50479,0.13367],[0.98360,0.49291,0.12849],[0.98108,0.48104,0.12332],[0.97837,0.46920,0.11817],[0.97545,0.45740,0.11305],[0.97234,0.44565,0.10797],[0.96904,0.43399,0.10294],[0.96555,0.42241,0.09798],[0.96187,0.41093,0.09310],[0.95801,0.39958,0.08831],[0.95398,0.38836,0.08362],[0.94977,0.37729,0.07905],[0.94538,0.36638,0.07461],[0.94084,0.35566,0.07031],[0.93612,0.34513,0.06616],[0.93125,0.33482,0.06218],[0.92623,0.32473,0.05837],[0.92105,0.31489,0.05475],[0.91572,0.30530,0.05134],[0.91024,0.29599,0.04814],[0.90463,0.28696,0.04516],[0.89888,0.27824,0.04243],[0.89298,0.26981,0.03993],[0.88691,0.26152,0.03753],[0.88066,0.25334,0.03521],[0.87422,0.24526,0.03297],[0.86760,0.23730,0.03082],[0.86079,0.22945,0.02875],[0.85380,0.22170,0.02677],[0.84662,0.21407,0.02487],[0.83926,0.20654,0.02305],[0.83172,0.19912,0.02131],[0.82399,0.19182,0.01966],[0.81608,0.18462,0.01809],[0.80799,0.17753,0.01660],[0.79971,0.17055,0.01520],[0.79125,0.16368,0.01387],[0.78260,0.15693,0.01264],[0.77377,0.15028,0.01148],[0.76476,0.14374,0.01041],[0.75556,0.13731,0.00942],[0.74617,0.13098,0.00851],[0.73661,0.12477,0.00769],[0.72686,0.11867,0.00695],[0.71692,0.11268,0.00629],[0.70680,0.10680,0.00571],[0.69650,0.10102,0.00522],[0.68602,0.09536,0.00481],[0.67535,0.08980,0.00449],[0.66449,0.08436,0.00424],[0.65345,0.07902,0.00408],[0.64223,0.07380,0.00401],[0.63082,0.06868,0.00401],[0.61923,0.06367,0.00410],[0.60746,0.05878,0.00427],[0.59550,0.05399,0.00453],[0.58336,0.04931,0.00486],[0.57103,0.04474,0.00529],[0.55852,0.04028,0.00579],[0.54583,0.03593,0.00638],[0.53295,0.03169,0.00705],[0.51989,0.02756,0.00780],[0.50664,0.02354,0.00863],[0.49321,0.01963,0.00955],[0.47960,0.01583,0.01055]]
        colormap = turbo_colormap_data
        x = max(0.0, min(1.0, x))
        a = int(x*255.0)
        b = min(255, a + 1)
        f = x*255.0 - a
        return [colormap[a][0] + (colormap[b][0] - colormap[a][0]) * f,
                colormap[a][1] + (colormap[b][1] - colormap[a][1]) * f,
                colormap[a][2] + (colormap[b][2] - colormap[a][2]) * f]

    
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

    def find_closest_lidar(self, lidar_start_token, stamp_nsec):
        candidates = []

        next_lidar_token = self.nusc.get('sample_data', lidar_start_token)['next']
        while next_lidar_token != '':
            lidar_data = self.nusc.get('sample_data', next_lidar_token)
            if lidar_data['is_key_frame']:
                break

            # dist_abs = abs(stamp_nsec - get_time(lidar_data).to_nsec())
            dist_abs = abs(stamp_nsec - self.unix_us2time(lidar_data['timestamp']).nanoseconds)
            candidates.append((dist_abs, lidar_data))
            next_lidar_token = lidar_data['next']

        if len(candidates) == 0:
            return None

        return min(candidates, key=lambda x: x[0])[1]

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

    def get_pose(self, data):
        p = Pose()
        p.position.x = data['translation'][0]
        p.position.y = data['translation'][1]
        p.position.z = data['translation'][2]
        
        p.orientation.w = data['rotation'][0]
        p.orientation.x = data['rotation'][1]
        p.orientation.y = data['rotation'][2]
        p.orientation.z = data['rotation'][3]
        
        return p

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

    def get_radar(self, sample_data, frame_id):
        pc_filename = 'data/' + sample_data['filename']
        # pc = pypcd.PointCloud.from_path(pc_filename)
        # msg = numpy_pc2.array_to_pointcloud2(pc.pc_data)
        # msg.header.frame_id = frame_id
        # msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        # return msg
        pass

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

    def get_camera_compressed(self, sample_data, frame_id):
        jpg_filename = 'data/' + sample_data['filename']
        msg = CompressedImage()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        msg.format = "jpeg"
        with open(jpg_filename, 'rb') as jpg_file:
            msg.data = jpg_file.read()
        return msg

    # RGB Image
    def get_camera(self, sample_data, frame_id):
        jpg_filename = 'data/' + sample_data['filename']
        img = cv2.imread(jpg_filename)

        msg = Image()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        msg.height, msg.width = img.shape[:2]
        msg.step = msg.width*3
        msg.encoding = "bgr8"
        msg.data = np.array(img).tostring()
        
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

    def get_lidar_imagemarkers(self, sample_lidar, sample_data, frame_id):
        # lidar image markers in camera frame
        points, coloring, _ = self.nusc.explorer.map_pointcloud_to_image(
            pointsensor_token=sample_lidar['token'],
            camera_token=sample_data['token'],
            render_intensity=True)
        points = points.transpose()
        coloring = [self.turbomap(c) for c in coloring]

        marker = ImageMarker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.unix_us2time(sample_data['timestamp']).to_msg()
        marker.ns = 'LIDAR_TOP'
        marker.id = 0
        marker.type = ImageMarker.POINTS
        marker.action = ImageMarker.ADD
        marker.scale = 2.0
        marker.points = [self.make_point2d(p) for p in points]
        marker.outline_colors = [self.make_color(c) for c in coloring]
        return marker
    
    def get_remove_imagemarkers(self, frame_id, ns, stamp):
        marker = ImageMarker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = 0
        marker.action = ImageMarker.REMOVE
        return marker

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

    # write ##############################################
    def write_occupancy_grid(self, nusc_map, ego_pose, stamp):
        translation = ego_pose['translation']
        rotation = Quaternion(ego_pose['rotation'])
        yaw = quaternion_yaw(rotation) / np.pi * 180
        patch_box = (translation[0], translation[1], 32, 32)
        canvas_size = (patch_box[2] * 10, patch_box[3] * 10)

        drivable_area = nusc_map.get_map_mask(patch_box, yaw, ['drivable_area'], canvas_size)[0]
        drivable_area = (drivable_area * 100).astype(np.int8)

        msg = OccupancyGrid()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = stamp.to_msg()
        msg.info.map_load_time = stamp.to_msg()
        msg.info.resolution = 0.1
        msg.info.width = drivable_area.shape[1]
        msg.info.height = drivable_area.shape[0]
        msg.info.origin.position.x = -16.0
        msg.info.origin.position.y = -16.0
        msg.info.origin.orientation.w = 1.0
        msg.data = drivable_area.flatten().tolist()

        # bag.write('/drivable_area', msg, stamp)
        self.writer.write(
            '/drivable_area',
            serialize_message(msg),
            stamp.nanoseconds
        )

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
                type='nav_msgs/msg/OccupancyGrid',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # TODO: RADAR
        # /LIDAR_TOP
        topic_info = rosbag2_py._storage.TopicMetadata(
            name='/LIDAR_TOP',
            type='sensor_msgs/msg/PointCloud2',
            serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # camera
        channels = ['CAM_BACK',  'CAM_FRONT', 'CAM_FRONT_LEFT',
                    'CAM_FRONT_RIGHT', 'CAM_BACK_RIGHT', 'CAM_BACK_LEFT']
        for channel in channels:
            # CompressedImage
            topic_info = rosbag2_py._storage.TopicMetadata(
                name='/' + channel + '/image_rect_compressed',
                type='sensor_msgs/msg/CompressedImage',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            # Image
            topic_info = rosbag2_py._storage.TopicMetadata(
                name='/' + channel + '/image',
                type='sensor_msgs/msg/Image',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            # /CameraInfo
            topic_info = rosbag2_py._storage.TopicMetadata(
                name='/' + channel + '/camera_info',
                type='sensor_msgs/msg/CameraInfo',
                serialization_format='cdr')
            self.writer.create_topic(topic_info)
            # /{channel}/image_markers_lidar
            topic_info = rosbag2_py._storage.TopicMetadata(
                    name='/' + channel + '/image_markers_lidar',
                    type='visualization_msgs/msg/ImageMarker',
                    serialization_format='cdr')
            self.writer.create_topic(topic_info)
        # /gps
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/gps',
                type='sensor_msgs/msg/NavSatFix',
                serialization_format='cdr')
        self.writer.create_topic(topic_info)
        # /markers/annotations
        topic_info = rosbag2_py._storage.TopicMetadata(
                name='/markers/annotations',
                type='visualization_msgs/msg/MarkerArray',
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

            # TODO: write CAN messages to /pose, /odom, and /diagnostics

            # publish /tf
            tf_array = self.get_tfmessage(cur_sample)
            self.writer.write(
                '/tf',
                serialize_message(tf_array),
                stamp.nanoseconds
            )

            # /driveable_area occupancy grid
            self.write_occupancy_grid(nusc_map, ego_pose, stamp)
            
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
                    msg = self.get_camera_compressed(sample_data, sensor_id)
                    # bag.write(topic + '/image_rect_compressed', msg, stamp)
                    self.writer.write(
                        topic + '/image_rect_compressed',
                        serialize_message(msg),
                        stamp.nanoseconds
                    )
                    msg = self.get_camera(sample_data, sensor_id)
                    self.writer.write(
                        topic + '/image',
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

            # publish /markers/annotations
            marker_array = MarkerArray()
            for annotation_id in cur_sample['anns']:
                ann = self.nusc.get('sample_annotation', annotation_id)
                marker_id = int(ann['instance_token'][:4], 16)
                c = np.array(self.nusc.explorer.get_color(ann['category_name'])) / 255.0

                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = stamp.to_msg()
                marker.id = marker_id
                marker.text = ann['instance_token'][:4]
                marker.type = Marker.CUBE
                marker.pose = self.get_pose(ann)
                marker.frame_locked = True
                marker.scale.x = ann['size'][1]
                marker.scale.y = ann['size'][0]
                marker.scale.z = ann['size'][2]
                marker.color = self.make_color(c, 0.5)
                marker_array.markers.append(marker)
            # bag.write('/markers/annotations', marker_array, stamp)
            self.writer.write(
                '/markers/annotations',
                serialize_message(marker_array),
                stamp.nanoseconds
            )

            # collect all sensor frames after this sample but before the next sample
            non_keyframe_sensor_msgs = []
            for (sensor_id, sample_token) in cur_sample['data'].items():
                topic = '/' + sensor_id

                next_sample_token = self.nusc.get('sample_data', sample_token)['next']
                while next_sample_token != '':
                    next_sample_data = self.nusc.get('sample_data', next_sample_token)
                    # if next_sample_data['is_key_frame'] or get_time(next_sample_data).to_nsec() > next_stamp.to_nsec():
                    #     break
                    if next_sample_data['is_key_frame']:
                        break

                    if next_sample_data['sensor_modality'] == 'radar':
                        msg = self.get_radar(next_sample_data, sensor_id)
                        # non_keyframe_sensor_msgs.append((msg.header.stamp.to_nsec(), topic, msg))
                    elif next_sample_data['sensor_modality'] == 'lidar':
                        msg = self.get_lidar(next_sample_data, sensor_id)
                        non_keyframe_sensor_msgs.append((msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec, topic, msg))
                    elif next_sample_data['sensor_modality'] == 'camera':
                        # CompressedImage
                        msg = self.get_camera_compressed(next_sample_data, sensor_id)
                        camera_stamp_nsec = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                        non_keyframe_sensor_msgs.append((camera_stamp_nsec, topic + '/image_rect_compressed', msg))
                        # Image
                        msg = self.get_camera(next_sample_data, sensor_id)
                        camera_stamp_nsec = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                        non_keyframe_sensor_msgs.append((camera_stamp_nsec, topic + '/image', msg))

                        msg = self.get_camera_info(next_sample_data, sensor_id)
                        non_keyframe_sensor_msgs.append((camera_stamp_nsec, topic + '/camera_info', msg))

                        closest_lidar = self.find_closest_lidar(cur_sample['data']['LIDAR_TOP'], camera_stamp_nsec)
                        if closest_lidar is not None:
                            msg = self.get_lidar_imagemarkers(closest_lidar, next_sample_data, sensor_id)
                            # non_keyframe_sensor_msgs.append((msg.header.stamp.to_nsec(), topic + '/image_markers_lidar', msg))
                        else:
                            msg = self.get_remove_imagemarkers(sensor_id, 'LIDAR_TOP', msg.header.stamp)
                        non_keyframe_sensor_msgs.append((msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec, topic + '/image_markers_lidar', msg))

                        # Delete all image markers on non-keyframe camera images
                        # msg = get_remove_imagemarkers(sensor_id, 'LIDAR_TOP', msg.header.stamp)
                        # non_keyframe_sensor_msgs.append((camera_stamp_nsec, topic + '/image_markers_lidar', msg))
                        # msg = get_remove_imagemarkers(sensor_id, 'annotations', msg.header.stamp)
                        # non_keyframe_sensor_msgs.append((camera_stamp_nsec, topic + '/image_markers_annotations', msg))

                    next_sample_token = next_sample_data['next']

            # sort and publish the non-keyframe sensor msgs
            non_keyframe_sensor_msgs.sort(key=lambda x: x[0])
            for (_, topic, msg) in non_keyframe_sensor_msgs:
                # bag.write(topic, msg, msg.header.stamp)
                self.writer.write(
                    topic,
                    serialize_message(msg),
                    msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                )

            # move to the next sample
            cur_sample = self.nusc.get('sample', cur_sample['next']) if cur_sample.get('next') != '' else None

        self.get_logger().info(f'Finished writing {bag_name}')