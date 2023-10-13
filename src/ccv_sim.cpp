#include "ccv_sim/ccv_sim.h"

ccvSim::ccvSim(void) : local_nh_("~")
{
    ROS_INFO_STREAM("=== ccv_sim ===");

    local_nh_.param<double>("MAX_VELOCITY", MAX_VELOCITY_, {1.5});
    local_nh_.param<double>("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY_, {M_PI});
    local_nh_.param<double>("MAX_STEERING_ANGLE", MAX_STEERING_ANGLE_, {20*M_PI/180});
    local_nh_.param<double>("MAX_PITCH_ANGLE", MAX_PITCH_ANGLE_, {M_PI/12.0});
    local_nh_.param<double>("MAX_ROLL_ANGLE", MAX_ROLL_ANGLE_, {M_PI/24.0});
    local_nh_.param<double>("PITCH_OFFSET", PITCH_OFFSET_, 0.0);
    local_nh_.param<double>("STEER_R_OFFSET", STEER_R_OFFSET_, {3.25*M_PI/180});
    local_nh_.param<double>("STEER_L_OFFSET", STEER_L_OFFSET_, 0.0);
    local_nh_.param<double>("TREAD", TREAD_, 0.5);
    local_nh_.param<double>("HZ", HZ_, 50.0);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    cmd_pos_pub_ = nh_.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("cmd_pos", 1);
    roll_pitch_pub_ = nh_.advertise<sq2_ccv_roll_pitch_msgs::RollPitch>("roll_pitch", 1);

    joy_sub_ = nh_.subscribe("/joy", 1, &ccvSim::joy_callback, this, ros::TransportHints().tcpNoDelay());
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &ccvSim::cmd_vel_callback, this, ros::TransportHints().tcpNoDelay());
    cmd_pos_sub_ = nh_.subscribe("cmd_pos", 1, &ccvSim::cmd_pos_callback, this, ros::TransportHints().tcpNoDelay());

    is_joy_subscribe_ = false;
    auto_flag_ = false;
    move_flag_ = false;
    joy_flag_ = false;
    mode = 0;
    ROS_INFO_STREAM("mode: " << mode);
}

void ccvSim::joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    joy_ = *msg;
    is_joy_subscribe_ = true;
    // ROS_INFO_STREAM("connected to joy");
}

void ccvSim::cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    cmd_vel_ = *msg;
}

void ccvSim::cmd_pos_callback(const ccv_dynamixel_msgs::CmdPoseByRadian::ConstPtr &msg)
{
    cmd_pos_ = *msg;
    roll_pitch_.roll = cmd_pos_.roll;
    roll_pitch_.pitch = cmd_pos_.rear - PITCH_OFFSET_;
}

void ccvSim::process(void)
{
    ros::Rate loop_rate(HZ_);
    while(ros::ok())
    {
        if(is_joy_subscribe_)
        {
            double v = 0.0;
            double w = 0.0;
            double direction = 0.0;
            double d_i = 0.0;
            double d_o = 0.0;
            double pitch = 0.0;
            double roll = 0.0;
            if(joy_.buttons[CROSS_]) mode = -1;   // stop
            if(joy_.buttons[CIRCLE_]) mode = 0;   // manual: L = velocity, R = yawrate
            if(joy_.buttons[SQUARE_]) mode = 1;   // manual: L = velocity and yawrate, R = steering angle
            if(joy_.buttons[TRIANGLE_]) mode = 2; // manual: L = velocity and yawrate, R = roll and pitch
            ROS_INFO_STREAM("mode: " << mode);
            if(mode == -1)
            {
                v = 0.0;
                w = 0.0;
                d_o = 0.0;
                d_i = 0.0;
            }
            if(joy_.buttons[L1_])
            {
                if(mode == 0)
                {
                    v = joy_.axes[L_STICK_V_]*MAX_VELOCITY_;
                    w = joy_.axes[L_STICK_H_]*MAX_ANGULAR_VELOCITY_;
                    double r_h = joy_.axes[R_STICK_H_];
                    double r_v = joy_.axes[R_STICK_V_];
                    double stick_angle = 0;
                    if(sqrt(r_h*r_h + r_v*r_v) > 0.5) stick_angle = atan2(r_v, r_h);
                    direction = stick_angle/2.0;
                }
                else if(mode == 1)
                {
                    v = joy_.axes[L_STICK_V_] * MAX_VELOCITY_;
                    w = joy_.axes[L_STICK_H_] * MAX_ANGULAR_VELOCITY_;
                }

                else if(mode == 2)
                {
                    v = joy_.axes[L_STICK_V_] * MAX_VELOCITY_;
                    w = joy_.axes[L_STICK_H_] * MAX_ANGULAR_VELOCITY_;
                    // if(joy_.buttons[L2_] && !joy_.buttons[R2_]){
                    //     // w = 0.0;
                    //     direction = MAX_STEERING_ANGLE_ * (1.0 - (joy_.axes[L2_STICK_] + 1.0) * 0.5);
                    // }else if(joy_.buttons[R2_] && !joy_.buttons[L2_]){
                    //     // w = 0.0;
                    //     direction = -MAX_STEERING_ANGLE_ * (1.0 - (joy_.axes[L2_STICK_] + 1.0) * 0.5);
                    // }else if(joy_.buttons[R2_] && joy_.buttons[L2_]){
                    //     // std::cout << "brake" << std::endl;
                    //     v = 0.0;
                    //     w = 0.0;
                    // }
                    pitch = joy_.axes[R_STICK_V_] * MAX_PITCH_ANGLE_;
                    pitch = std::max(-MAX_PITCH_ANGLE_, std::min(MAX_PITCH_ANGLE_, pitch));
                    roll = joy_.axes[R_STICK_H_] * MAX_ROLL_ANGLE_;
                    roll = std::max(-MAX_ROLL_ANGLE_, std::min(MAX_ROLL_ANGLE_, roll));
                }

                direction = std::max(-MAX_STEERING_ANGLE_, std::min(MAX_STEERING_ANGLE_, direction));
                if(abs(w) > 1e-2)
                {
                    double r = abs(v/w);
                    d_i = atan(r*sin(direction)/(r*cos(direction) - TREAD_/2.0));
                    d_o = atan(r*sin(direction)/(r*cos(direction) + TREAD_/2.0));
                    if(w<0.0)
                    {
                        double buf = d_i;
                        d_i = d_o;
                        d_o = buf;
                    }
                }
                else
                {
                    d_i = direction;
                    d_o = direction;
                }
            }
            std::cout << "v: " << v << ", " << "w: " << w << ", " << "direction: " << direction << std::endl;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v;
            cmd_vel.angular.z = w;

            ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
            sq2_ccv_roll_pitch_msgs::RollPitch roll_pitch;
            cmd_pos.fore = -pitch + PITCH_OFFSET_;
            cmd_pos.rear = pitch + PITCH_OFFSET_;
            cmd_pos.roll = -roll;
            roll_pitch.roll = roll;
            roll_pitch.pitch = pitch;

            cmd_vel_pub_.publish(cmd_vel);
            cmd_pos_pub_.publish(cmd_pos);
            roll_pitch_pub_.publish(roll_pitch);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}