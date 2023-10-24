#include <iostream>
#include <chrono>
#include <thread>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

#include <iomanip>
using namespace westonrobot;

void dispalay_motion(MotionStateMessage motion_state)
{
    //读取运动状态(线速度，角速度，横向速度，转向速度)
    std::cout << "velocity (linear, angular, lateral, steering): "
            << std::setw(6) << motion_state.linear_velocity << ", "
            << std::setw(6) << motion_state.angular_velocity << ", "
            << std::setw(6) << motion_state.lateral_velocity << ", "
            << std::setw(6) << motion_state.steering_angle << std::endl;
}

void msleep(unsigned int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int main(int argc, char** argv) {
    std::string device_name;

    if (argc == 2) {
        device_name = {argv[1]};
        std::cout << "Selected interface: " << device_name << std::endl;
    } else {
        std::cout << "Usage: control_scout_mini_omni_robot <interface>" << std::endl
                  << "Example: ./control_scout_mini_omni_robot can0" << std::endl;
        return -1;
    }

    std::unique_ptr<ScoutMiniOmniRobot> scout;

    ProtocolDetector detector;
    detector.Connect(device_name);
    auto proto = detector.DetectProtocolVersion(5);

    if (proto == ProtocolVersion::AGX_V1) {
        std::cout << "Detected protocol: AGX_V1" << std::endl;
        scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V1);
    } else if (proto == ProtocolVersion::AGX_V2) {
        std::cout << "Detected protocol: AGX_V2" << std::endl;
        scout = std::make_unique<ScoutMiniOmniRobot>(ProtocolVersion::AGX_V2);
    } else {
        std::cout << "Detected protocol: UNKNOWN" << std::endl;
        return -1;
    }

    if (scout == nullptr) {
        std::cout << "Failed to create robot object" << std::endl;
        return -1;
    }

    scout->Connect(device_name);

    if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2) {
        scout->EnableCommandedMode();
    }


    /*
    TASK1
    */
    std::cout<<"----------Start reading the vehicle status information----------"<<std::endl;
    //读取车灯状态 读取前后车灯的模式，CONST_OFF = 0x00(关),CONST_ON = 0x01(开),BREATH = 0x02(呼吸),CUSTOM = 0x03(自定义值)
    ScoutCoreState scout_state = scout->GetRobotState();
    LightStateMessage light_state = scout_state.light_state;
    // 定义映射字典
    std::map<AgxLightMode, std::string> light_mode_mapping = {
        {CONST_OFF, "OFF"},
        {CONST_ON, "ON"},
        {BREATH, "BREATH"}
    };
    sleep(1);
    scout->SetLightCommand(BREATH, 0, BREATH, 0);
    std::cout << "Light State: Front - " << light_mode_mapping[scout->GetRobotState().light_state.front_light.mode]
                << ", Rear - " << light_mode_mapping[scout->GetRobotState().light_state.rear_light.mode] << std::endl;
    sleep(1);

    //读取电池状态(电池电压)
    SystemStateMessage system_state = scout_state.system_state;
    std::cout << "Battery Voltage: " << scout->GetRobotState().system_state.battery_voltage << " V" << std::endl;


    while (true) {
        //读取电机状态(电流，转速，驱动器温度，电机温度)
        auto actuator = scout->GetActuatorState();
        if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V1) {
        for (int i = 0; i < 3; ++i) {
            printf("motor %d: current %f, rpm %d, driver temp %f, motor temp %f\n",
                scout->GetActuatorState().actuator_state[i].motor_id,
                scout->GetActuatorState().actuator_state[i].current,
                scout->GetActuatorState().actuator_state[i].rpm,
                scout->GetActuatorState().actuator_state[i].driver_temp,
                scout->GetActuatorState().actuator_state[i].motor_temp);
        }
        std::cout << "actuator state age (ms): "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                        SdkClock::now() - actuator.time_stamp)
                        .count()
                  << std::endl;
        } else {
            //非AGX_V1协议
            for (int i = 0; i < 4; ++i) {
                printf("motor %d: current %.2f, rpm %d, driver temp %.2f, motor temp %2.f\n",
                scout->GetActuatorState().actuator_hs_state[i].motor_id,
                scout->GetActuatorState().actuator_hs_state[i].current,
                scout->GetActuatorState().actuator_hs_state[i].rpm,
                scout->GetActuatorState().actuator_ls_state[i].driver_temp,
                scout->GetActuatorState().actuator_ls_state[i].motor_temp);
            }
            std::cout << "actuator state age (ms): "  
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                    SdkClock::now() - actuator.time_stamp)
                    .count()
            << std::endl;
        }

        MotionStateMessage motion_state = scout->GetRobotState().motion_state;
        //读取运动状态(线速度，角速度，横向速度，转向速度)
        dispalay_motion(motion_state);


        // /*
        // TASK2
        // */
        std::cout<<"----------Start controlling the vehicle's movement----------"<<std::endl;

        // 控制小车前进
        std::cout << "Control: Forward" << std::endl;
        scout->SetMotionCommand(3.0, 0.0, 0.0);
        //读取运动状态(线速度，角速度，横向速度，转向速度)
        msleep(400);
        motion_state = scout->GetRobotState().motion_state;

        dispalay_motion(motion_state);
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 控制小车后退
        std::cout << "Control: Backward" << std::endl;
        scout->SetMotionCommand(-3.0, 0.0, 0.0);
        //读取运动状态(线速度，角速度，横向速度，转向速度)
        msleep(400);
        motion_state = scout->GetRobotState().motion_state;
        dispalay_motion(motion_state);
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 控制小车旋转
        std::cout << "Control: Rotate" << std::endl;
        scout->SetMotionCommand(0.0, 1.0, 0.0);
        msleep(400);
        //读取运动状态(线速度，角速度，横向速度，转向速度)
        motion_state = scout->GetRobotState().motion_state;
        dispalay_motion(motion_state);
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 控制小车旋转
        std::cout << "Control: Rotate" << std::endl;
        scout->SetMotionCommand(0.0, -1.0, 0.0);
        msleep(400);
        //读取运动状态(线速度，角速度，横向速度，转向速度)
        motion_state = scout->GetRobotState().motion_state;
        dispalay_motion(motion_state);
        // 等待一段时间
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // // 停止小车运动
        // std::cout << "Control: Stop" << std::endl;
        // scout->SetMotionCommand(0.0, 0.0, 0.0);
        // //读取运动状态(线速度，角速度，横向速度，转向速度)
        // motion_state = scout->GetRobotState().motion_state;
        // dispalay_motion(motion_state);
        // // 等待一段时间
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    return 0;
}
