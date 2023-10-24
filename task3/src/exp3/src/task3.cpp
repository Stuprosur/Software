// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <std_msgs/String.h>
// #include "ugv_sdk/mobile_robot/scout_robot.hpp"
// #include "ugv_sdk/utilities/protocol_detector.hpp"

// using namespace westonrobot;

// std::unique_ptr<ScoutMiniOmniRobot> scout;

// void controlCallback(const geometry_msgs::Twist::ConstPtr& msg) {
//     // 控制小车运动
//     scout->SetMotionCommand(1.0, 0.0, 0.0);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "scout_control_node");
//     ros::NodeHandle nh;

//     std::string device_name;

//     if (argc == 2) {
//         device_name = {argv[1]};
//         ROS_INFO("Selected interface: %s", device_name.c_str());
//     } else {
//         ROS_ERROR("Usage: scout_control_node <interface>");
//         ROS_ERROR("Example: rosrun your_package_name scout_control_node can0");
//         return -1;
//     }

//     ProtocolDetector detector;
//     detector.Connect(device_name);
//     auto proto = detector.DetectProtocolVersion(5);

//     if (proto == ProtocolVersion::AGX_V1) {
//         ROS_INFO("Detected protocol: AGX_V1");
//         scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V1);
//     } else if (proto == ProtocolVersion::AGX_V2) {
//         ROS_INFO("Detected protocol: AGX_V2");
//         scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V2);
//     } else {
//         ROS_ERROR("Detected protocol: UNKNOWN");
//         return -1;
//     }

//     if (scout == nullptr) {
//         ROS_ERROR("Failed to create robot object");
//         return -1;
//     }

//     scout->Connect(device_name);

//     if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
//         scout->EnableCommandedMode();
//     }

//     // 订阅控制命令的话题
//     ros::Subscriber control_sub = nh.subscribe("motion_cmd_topic", 1, controlCallback);

//     // 创建用于发布小车状态信息的话题
//     ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("motion_cmd_topic", 1);

//     ros::Rate loop_rate(10); // 10 Hz

//     while (ros::ok()) {
//         // 发布小车状态信息
//         geometry_msgs::Twist cmd_msg;
//         cmd_msg.linear.x = 0.5;   // 设置线速度
//         cmd_msg.linear.y = 0.0;
//         cmd_msg.angular.z = 0.0;  // 设置角速度
//         control_pub.publish(cmd_msg);


//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

using namespace westonrobot;

std::unique_ptr<ScoutMiniOmniRobot> scout;

int getKey() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void controlCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 控制小车运动
    scout->SetMotionCommand(msg->linear.x, msg->angular.z, 0.0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scout_control_node");
    ros::NodeHandle nh;

    std::string device_name;

    if (argc == 2) {
        device_name = {argv[1]};
        ROS_INFO("Selected interface: %s", device_name.c_str());
    } else {
        ROS_ERROR("Usage: scout_control_node <interface>");
        ROS_ERROR("Example: rosrun your_package_name scout_control_node can0");
        return -1;
    }

    ProtocolDetector detector;
    detector.Connect(device_name);
    auto proto = detector.DetectProtocolVersion(5);

    if (proto == ProtocolVersion::AGX_V1) {
        ROS_INFO("Detected protocol: AGX_V1");
        scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V1);
    } else if (proto == ProtocolVersion::AGX_V2) {
        ROS_INFO("Detected protocol: AGX_V2");
        scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V2);
    } else {
        ROS_ERROR("Detected protocol: UNKNOWN");
        return -1;
    }

    if (scout == nullptr) {
        ROS_ERROR("Failed to create robot object");
        return -1;
    }

    scout->Connect(device_name);

    if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
        scout->EnableCommandedMode();
    }

    // 订阅控制命令的话题
    ros::Subscriber control_sub = nh.subscribe("motion_cmd_topic", 1, controlCallback);
    // 创建用于发布小车控制命令的话题
    ros::Publisher control_pub = nh.advertise<geometry_msgs::Twist>("motion_cmd_topic", 1);
    ros::Rate loop_rate(10); // 10 Hz

    while (ros::ok()) {
        int key = getKey();
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = 0.0;   // 初始化线速度
        cmd_msg.angular.z = 0.0;  // 初始化角速度

        if (key == 'w') {
            cmd_msg.linear.x = 1.0;   // 前进
        } else if (key == 's') {
            cmd_msg.linear.x = -1.0;  // 后退
        } else if (key == 'a') {
            cmd_msg.angular.z = 1.0;  // 左转
        } else if (key == 'd') {
            cmd_msg.angular.z = -1.0;  // 右转
        } else if (key == ' ') {
            // 空格键停止小车
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
        }

        control_pub.publish(cmd_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
