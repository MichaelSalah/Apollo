#ifndef APOLLO_BASE_APOLLO_HARDWARE_H
#define APOLLO_BASE_APOLLO_HARDWARE_H
#include "hardware_interface/hardware_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/callback_queue.h"
#include "controller_manager/controller_manager.h"
#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>
#include "std_msgs/Int16MultiArray.h"

#define RAD(rpm)(rpm * 0.1047197551196667)
#define RPM(rad)(rad * 9.549296585513089)
using namespace std;

class ApolloHardware : public hardware_interface::RobotHW {
public:
    explicit ApolloHardware(ros::NodeHandle& nh);
    void read(const ros::Duration& period);
    void write();
    void control_loop(controller_manager::ControllerManager& cm);
    void feedback_cb(const std_msgs::Int16MultiArray::ConstPtr& ptr);
    ros::NodeHandle nh, private_nh;
    realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>* RT_pub;

private:
    hardware_interface::JointStateInterface _joint_state_interface;
    hardware_interface::VelocityJointInterface _velocity_joint_interface;
    hardware_interface::PositionJointInterface _position_joint_interface;
    vector<string> _joint_names = {"fl_joint", "rl_joint", "fr_joint", "rr_joint", "fork_joint"};
    vector<double> _joint_pos = {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> _joint_vel = {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> _joint_eff = {0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> _joint_cmd = {0.0, 0.0, 0.0, 0.0, 0.0};
    ros::Subscriber motors_sub;
    std::vector<int16_t> incoming_data = {0, 0, 0, 0, 0};
};


#endif //APOLLO_BASE_APOLLO_HARDWARE_H
