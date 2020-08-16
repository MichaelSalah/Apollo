#include "apollo_base/apollo_hardware.h"

 ApolloHardware::ApolloHardware(ros::NodeHandle& nh) {
    for(uint8_t i = 0; i < 5; i++){
        hardware_interface::JointStateHandle _jnt_state(_joint_names[i], &_joint_pos[i], &_joint_vel[i], &_joint_eff[i]);
        _joint_state_interface.registerHandle(_jnt_state);
        hardware_interface::JointHandle _jnt(_jnt_state, &_joint_cmd[i]);
        if(_joint_names[i] == "fork_joint"){
            _position_joint_interface.registerHandle(_jnt);
            break;
        }
        _velocity_joint_interface.registerHandle(_jnt);
    }
    registerInterface(&_velocity_joint_interface);
    registerInterface(&_position_joint_interface);
    registerInterface(&_joint_state_interface);
    RT_pub = new realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>(nh, "commands", 5);
    motors_sub = nh.subscribe("motor_feedback", 5, &ApolloHardware::feedback_cb, this);
}

void ApolloHardware::feedback_cb(const std_msgs::Int16MultiArray::ConstPtr& ptr){
    incoming_data = ptr->data;
}

void ApolloHardware::read(const ros::Duration &period) {
    for(uint8_t i = 0; i < 4; i++){
        _joint_pos[i] += 0.5 * period.toSec() * (RAD(incoming_data[i]) + _joint_vel[i]);
        _joint_vel[i] = RAD(incoming_data[i]);
    }
    _joint_pos[4] = (double)incoming_data[4] / 100.0;
}

void ApolloHardware::write(){
    vector<int16_t> cmd;
    for(uint8_t i = 0; i < 5; i++){
        cmd.push_back((i == 4)?(int16_t)(_joint_cmd[i]*100):(int16_t)RPM(_joint_cmd[i])); //use when fork is position controlled
        //cmd.push_back((int16_t)RPM(_joint_cmd[i])); //use when fork is velocity controlled
        //ROS_INFO("%d: %f", i, _joint_cmd[i]);
    }
    if(RT_pub->trylock()){
        RT_pub->msg_.data.resize(5);
        RT_pub->msg_.data = cmd;
        RT_pub->unlockAndPublish();
    }
}

void ApolloHardware::control_loop(controller_manager::ControllerManager& cm) {
    static ros::Time last_time = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_time;
    read(period);
    cm.update(ros::Time::now(), period);
    write();
    last_time = ros::Time::now();
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "apollo_interface");
    ros::NodeHandle nh, private_nh("~");

    double freq;
    private_nh.param<double>("control_frequency", freq, 10.0);
    ApolloHardware robot(nh);
    controller_manager::ControllerManager cm(&robot, nh);
    ros::CallbackQueue control_queue;
    ros::AsyncSpinner spinner(1, &control_queue);
    ros::TimerOptions t_opt(ros::Duration(1/freq), boost::bind(&ApolloHardware::control_loop, boost::ref(robot), boost::ref(cm)), &control_queue);
    ros::Timer timer = nh.createTimer(t_opt);
    spinner.start();
    //Consider using a rated spinOnce
    ros::spin();
    return 0;
}
