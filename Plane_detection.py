import sys
import rospy
import numpy as np
import open3d as o3d

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcd2


class Plane_detection():
    '''
    Subscribe scene point cloud from kinect driver ros topics and detect a plane from it.

    Parameters
    ----------
    z_threshold : float, pass through threshold along the z-axis.

    plane_thickness : float, maximum distance a point can be from the plane model.

    Attributes
    ----------
    pcd_sub : ROS subscriber of the point cloud message.

    scene_cloud : list, list type of the subscribed scene point cloud.

    Author : ou.

    Data : 2022-4-21
    '''

    def __init__(self, z_threshold=1.0, plane_thickness=0.01):
        self.pcd_sub = rospy.Subscriber('/points2', PointCloud2, self.__callback)
        self.z_threshold = float(z_threshold)
        self.plane_thickness = float(plane_thickness)
        self.scene_cloud = []
        rospy.sleep(1.0)

    def __callback(self, cloud):
        for data in pcd2.read_points(cloud, skip_nans=True):
            self.scene_cloud.append([data[0], data[1], data[2]])

    def __numpy2pcd(self, array):
        # Numpy to PointCloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(array)
        return pcd

    def detecter(self):
        # Convert to numpy
        SCENE_CLOUD = np.array(self.scene_cloud)
        try:
            pcd = self.__numpy2pcd(SCENE_CLOUD)
        except RuntimeError as e:
            print('\nScene cloud is empty, did the rosbag stop?\n')

        # Voxel downsampling
        pcd_down = pcd.voxel_down_sample(voxel_size=0.005)
        PCD_DOWN = np.asarray(pcd_down.points)
        scene_pass = []
        # Pass through
        for point in PCD_DOWN.tolist():
            if point[2] < self.z_threshold:
                scene_pass.append(point)
            else:
                pass
        pcd_scene_pass = self.__numpy2pcd(scene_pass)
        # Detect a plan using RANSAC algorithm
        _, inliers = pcd_scene_pass.segment_plane(distance_threshold=self.plane_thickness,
                                                  ransac_n = 3,
                                                  num_iterations = 1000)
        pcd_plane = pcd_scene_pass.select_down_sample(inliers, invert=False)
        pcd_remove_plane = pcd_scene_pass.select_down_sample(inliers, invert=True)

        # Paint the plane component
        pcd_plane.paint_uniform_color([0., 1., 0.])
        pcd_remove_plane.paint_uniform_color([0., 0, 1.])
        # Visualize point cloud
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        o3d.visualization.draw_geometries([pcd_remove_plane, pcd_plane, frame])


if __name__ == '__main__':
    rospy.init_node('Plane_detection_node', anonymous=True)
    Plane_detection(z_threshold=sys.argv[1], plane_thickness=sys.argv[2]).detecter()