#ifndef ROBOTIS_ARM_H
#define ROBOTIS_ARM_H

/** ROS */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
/** Dynamixel */
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
// Servos: AX-18A (ID=2), MX-12W (ID=1)
#include "dynamixel_workbench_msgs/AX.h"
#include "dynamixel_workbench_msgs/MX.h"


class ArmControl {
private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber joint_cmd_sub_;
    ros::Publisher joint_state_pub_;
    ros::Publisher dynamixel_state_pub_;

    // Dynamixel
    DynamixelWorkbench* dxl_wb_;
    uint8_t dxl_id_[16];
    uint8_t dxl_cnt_;
    
    bool debug_;

public:
    ArmControl(ros::NodeHandle n, bool debug);
    ~ArmControl();

    void controlLoop(void);
};
#endif //ROBOTIS_ARM_H
