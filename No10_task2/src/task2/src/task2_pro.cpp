// fusion_of_camera_and_lidar
#include <vector>
#include <iostream>
#include <math.h>
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_conversions/pcl_conversions.h>/* /opt/ros/kinetic/include/pcl_conversions */
#include <cv_bridge/cv_bridge.h>            /* /opt/ros/kinetic/include/cv_bridge */

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>                                //提供各种点云数据类型
#include <pcl/visualization/cloud_viewer.h>                 //提供pcl可视化窗口

#include <opencv2/opencv.hpp>

static const std::string ORIGIN_IMAGE = "origin image";
static const std::string FUSED_IMAGE = "fused image";

// ROS相关
ros::Publisher image_pub;
ros::Publisher cloud_pub;

// 相机相关
cv::Mat camera_image;
cv::Mat extrinsic_mat, camera_mat, dist_coeff, rotate_mat, transform_vec;

// 点云相关
pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
const float COLOR_DIS = 1.2;
void projection();

// 颜色映射
const int COLOR_MAP[][3] = {
    {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, {255, 140, 0}, {255, 165, 0},
    {238, 173, 14}, {255, 193, 37}, {255, 255, 0}, {255, 236, 139}, {202, 255, 112},
    {0, 255, 0}, {84, 255, 159}, {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
    {178, 223, 238}, {126, 192, 238}, {28, 134, 238}, {0, 0, 255}, {72, 118, 255},
    {122, 103, 238}
};

// 显示图像
void Image_Show(const cv::Mat &cv_image, const std::string &title) {
    cv::imshow(title, cv_image);
    if (cv::waitKey(1) == 27) {
        ros::shutdown();
        cv::destroyAllWindows();
    }
}

// 处理激光雷达数据
void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg) {
    ROS_INFO("lidarCallback");
    pcl::fromROSMsg(*msg, *lidar_cloud);
}

// 处理相机图像数据
void cameraCallback(const sensor_msgs::ImageConstPtr &msg) {
    ROS_INFO("cameraCallback");

    try {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg);
        camera_image = cv_image->image;
        Image_Show(cv_image->image, ORIGIN_IMAGE);
        projection();
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

// 预处理函数
void pre_process() {
    cv::FileStorage fs_read("./src/task2/config/comb_bd_1018.yml", cv::FileStorage::READ);

    if (fs_read.isOpened()) {
        fs_read["CameraExtrinsicMat"] >> extrinsic_mat;
        fs_read["CameraMat"] >> camera_mat;
        fs_read["DistCoeff"] >> dist_coeff;
        fs_read.release();

        // 计算旋转矩阵和平移向量
        cv::Mat temp_extrinsic_mat = extrinsic_mat.t();
        rotate_mat = temp_extrinsic_mat.colRange(0, 3).rowRange(0, 3);
        transform_vec = temp_extrinsic_mat.col(3).rowRange(0, 3);
    } else {
        ROS_ERROR("Failed to open .yml file.");
    }
}

// 投影函数
void projection() {
    if (lidar_cloud->empty() || camera_image.empty()) {
        return;
    }

    std::vector<cv::Point3f> points3d;
    std::vector<cv::Point2f> projectedPoints;
    std::vector<cv::Scalar> dis_color;

    for (const pcl::PointXYZ& point : lidar_cloud->points) {
        // cv::Point3f p3d(point.x, point.y, point.z);
        cv::Point3f p3d(point.x, point.y, point.z);
        points3d.push_back(p3d);
    }

    cv::projectPoints(points3d, rotate_mat, transform_vec, camera_mat, dist_coeff, projectedPoints);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    dis_color.reserve(projectedPoints.size());

    for (size_t i = 0; i < projectedPoints.size(); i++) {
        const cv::Point2f& p = projectedPoints[i];
        pcl::PointXYZRGB point_rgb;
        point_rgb.x = lidar_cloud->points[i].x;
        point_rgb.y = lidar_cloud->points[i].y;
        point_rgb.z = lidar_cloud->points[i].z;
        point_rgb.r = 0;
        point_rgb.g = 0;
        point_rgb.b = 0;

        if (p.y < 480 && p.y >= 0 && p.x < 640 && p.x >= 0) {
            point_rgb.r = static_cast<uint8_t>(camera_image.at<cv::Vec3b>(p.y, p.x)[2]);
            point_rgb.g = static_cast<uint8_t>(camera_image.at<cv::Vec3b>(p.y, p.x)[1]);
            point_rgb.b = static_cast<uint8_t>(camera_image.at<cv::Vec3b>(p.y, p.x)[0]);

            int color_order = std::abs(point_rgb.z) / (COLOR_DIS + 0.00001);
            if (color_order > 20) {
                color_order = 20;
            }
            dis_color.push_back(cv::Scalar(COLOR_MAP[color_order][2], COLOR_MAP[color_order][1], COLOR_MAP[color_order][0]));
        }

        rgb_cloud->push_back(point_rgb);
    }

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*rgb_cloud, ros_cloud);
    ros_cloud.header.frame_id = "rslidar";
    cloud_pub.publish(ros_cloud);

    for (size_t i = 0; i < projectedPoints.size(); i++) {
        const cv::Point2f& p = projectedPoints[i];
        if (p.y < 480 && p.y >= 0 && p.x < 640 && p.x >= 0) {
            cv::circle(camera_image, p, 1, dis_color[i], 1, 8, 0);
        }
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", camera_image).toImageMsg();
    image_pub.publish(msg);
    ROS_INFO("Fusion image published!");
    Image_Show(camera_image, FUSED_IMAGE);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_camera_fusion");
    ros::NodeHandle n;
    cv::namedWindow(ORIGIN_IMAGE);
    cv::namedWindow(FUSED_IMAGE);

    image_pub = n.advertise<sensor_msgs::Image>("fusion", 100);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("rgb_cloud", 100);
    ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, lidarCallback);
    ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, cameraCallback);

    pre_process();
    
    ros::spin();

    return 0;
}
