#ifndef __CCV_SIM_H
#define __CCV_SIM_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sq2_ccv_roll_pitch_msgs/RollPitch.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>

class ccvSim
{
public:
    ccvSim();

    void process(void);
    void joy_callback(const sensor_msgs::Joy::ConstPtr&);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
    void cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr &msg);

private:
    static constexpr int L_STICK_H_ = 0;
    static constexpr int L_STICK_V_ = 1;
    static constexpr int L2_STICK_ = 2;
    static constexpr int R_STICK_H_ = 3;
    static constexpr int R_STICK_V_ = 4;
    static constexpr int R2_STICK_ = 5;
    // buttons
    static constexpr int CROSS_ = 0;
    static constexpr int CIRCLE_ = 1;
    static constexpr int TRIANGLE_ = 2;
    static constexpr int SQUARE_ = 3;
    static constexpr int L1_ = 4;
    static constexpr int R1_ = 5;
    static constexpr int L2_ = 6;
    static constexpr int R2_ = 7;

    double MAX_VELOCITY_;
    double MAX_ANGULAR_VELOCITY_;
    double MAX_STEERING_ANGLE_;
    double MAX_PITCH_ANGLE_;
    double MAX_ROLL_ANGLE_;
    double PITCH_OFFSET_;
    double STEER_R_OFFSET_;
    double STEER_L_OFFSET_;
    double TREAD_;
    double HZ_;

    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cmd_pos_pub_;
    ros::Publisher roll_pitch_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber cmd_pos_sub_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Twist cmd_vel_;
    ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos_;
    sq2_ccv_roll_pitch_msgs::RollPitch roll_pitch_;

    bool is_joy_subscribe_;
    bool auto_flag_, move_flag_, joy_flag_;
    int mode;
};

#endif // __CCV_SIM_H

