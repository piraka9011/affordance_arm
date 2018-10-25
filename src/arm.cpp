#include <robotis_arm/arm.h>

ArmControl::ArmControl(ros::NodeHandle n, bool debug=true) :
                        nh_(n), debug_(debug)
{
    if (debug_) {
        if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
            ros::console::notifyLoggerLevelsChanged();
    }
    
    // Param setup
    std::string device_name = nh_.param<std::string>("device_name", "/dev/ttyUSB0");
    uint32_t dxl_baud_rate  = nh_.param<int>("baud_rate", 1000000);
    uint8_t scan_range      = nh_.param<int>("scan_range", 100);
    uint32_t profile_velocity     = nh_.param<int>("profile_velocity", 200);
    uint32_t profile_acceleration = nh_.param<int>("profile_acceleration", 50);
    ROS_DEBUG_STREAM("Arm Control Params:" << "\n" <<
                     "Device Name: " << device_name << "\n" <<
                     "Baud Rate: " << dxl_baud_rate << "\n" <<
                     "Scan Range: " << unsigned(scan_range) << "\n");
    // Servo setup
    dxl_wb_ = new DynamixelWorkbench;
    dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

    // Check if they're available
    if (!dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range)) {
        ROS_ERROR("Servos not found, check scan range or baud rate");
        ros::shutdown();
        return;
    }
    else
        ROS_INFO("Arm Controller Initialized!");
}

ArmControl::~ArmControl() {
    for (int index = 0; index < dxl_cnt_; index++)
        dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle n;
    ArmControl ac(n);
    
    return 0;
}
