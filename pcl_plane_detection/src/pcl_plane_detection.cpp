# include <ros/ros.h>
# include <sensor_msgs/PointCloud2.h>

# include <pcl/point_cloud.h>
# include <pcl/point_types.h>
# include <pcl_conversions/pcl_conversions.h>
# include <pcl/filters/voxel_grid.h>
# include <pcl/filters/passthrough.h>
# include <pcl/filters/extract_indices.h>
# include <pcl/ModelCoefficients.h>
# include <pcl/sample_consensus/method_types.h>
# include <pcl/sample_consensus/model_types.h>
# include <pcl/segmentation/sac_segmentation.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub;
float z_threshold;
float plane_thickness;

void callback(const sensor_msgs::PointCloud2ConstPtr& msg_cloud)
{
    // Convert sensor_msgs to pcl data type
    PointCloudXYZRGB::Ptr scene_cloud (new PointCloudXYZRGB);
    pcl::fromROSMsg(*msg_cloud, *scene_cloud);

    // Voxel downsampling
    PointCloudXYZRGB::Ptr down_cloud (new PointCloudXYZRGB);
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
    voxel.setInputCloud (scene_cloud);
    voxel.setLeafSize (0.005, 0.005, 0.005);
    voxel.filter (*down_cloud);

    // Pass through
    PointCloudXYZRGB::Ptr pass_cloud (new PointCloudXYZRGB);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (down_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, z_threshold);
    pass.filter (*pass_cloud);

    // Detect tha largest plane use RANSAC algorithm
    PointCloudXYZRGB::Ptr plane_cloud (new PointCloudXYZRGB);
    pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (plane_thickness);
    seg.setInputCloud (pass_cloud);
    seg.segment (*inliers, *coeff);

    // Extract the points belong to the detected plane
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (pass_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);

    // Change the color of the points on the plane
    for (auto& point: *plane_cloud) {
        point.r = 0;
        point.g = 255;
        point.b = 0;
    }

    // Convert pcl data type to sensor msgs and publish it the "/Plane_region" topics.
    sensor_msgs::PointCloud2 msg_plane_cloud;
    pcl::toROSMsg (*plane_cloud, msg_plane_cloud);
    pub.publish (msg_plane_cloud);
}   

int main (int argc, char** argv)
{
    ros::init (argc, argv, "Plane_detection_node");
    z_threshold = atof(argv[1]);
    plane_thickness = atof(argv[2]);

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/points2", 1, callback);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_region", 1);

    ros::spin ();
}