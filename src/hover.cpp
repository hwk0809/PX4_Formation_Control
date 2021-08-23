//
// Created by cc on 2020/8/5.
//

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

using namespace Eigen;

Vector3d current_p;
mavros_msgs::State current_state;
ros::Publisher pva_pub;

void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    /// ENU frame to NWU
    current_p << msg->pose.position.y, -msg->pose.position.x, msg->pose.position.z;

    //ROS_INFO("X:%f Y:%f  Z:%f",-current_p(0),current_p(1),current_p(2));
}


void stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;

}


void setPVA(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double yaw=0.0)
{
    trajectory_msgs::JointTrajectoryPoint pva_setpoint;

    pva_setpoint.positions.push_back(p(0)); //x
    pva_setpoint.positions.push_back(p(1)); //y
    pva_setpoint.positions.push_back(p(2)); //z
    pva_setpoint.positions.push_back(yaw);

    pva_setpoint.velocities.push_back(v(0));
    pva_setpoint.velocities.push_back(v(1));
    pva_setpoint.velocities.push_back(v(2));

    pva_setpoint.accelerations.push_back(a(0));
    pva_setpoint.accelerations.push_back(a(1));
    pva_setpoint.accelerations.push_back(a(2));

    pva_pub.publish(pva_setpoint);

//    ROS_INFO_THROTTLE(1.0, "P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO_THROTTLE(1.0, "V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO_THROTTLE(1.0, "A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
//
//    ROS_INFO("P x=%f, y=%f, z=%f", pva_setpoint.positions[0], pva_setpoint.positions[1], pva_setpoint.positions[2]);
//    ROS_INFO("V x=%f, y=%f, z=%f", pva_setpoint.velocities[0], pva_setpoint.velocities[1], pva_setpoint.velocities[2]);
//    ROS_INFO("A x=%f, y=%f, z=%f", pva_setpoint.accelerations[0], pva_setpoint.accelerations[1], pva_setpoint.accelerations[2]);
}

/** This function is to generate state to state trajectory **/
void motion_primitives(Eigen::Vector3d p0, Eigen::Vector3d v0, Eigen::Vector3d a0,
                       Eigen::Vector3d pf, Eigen::Vector3d vf, Eigen::Vector3d af, double v_max, double delt_t,
                       Eigen::MatrixXd &p, Eigen::MatrixXd &v, Eigen::MatrixXd &a, Eigen::VectorXd &t)
{
    // % Choose the time as running in average velocity
    // double decay_parameter = 0.5;
    // double T = 0.2;

    double j_limit = 5;
    double a_limit = 3;
    double v_limit = v_max;

//    double T1 = fabs(af(0)-a0(0))/j_limit > fabs(af(1)-a0(1))/j_limit ? fabs(af(0)-a0(0))/j_limit : fabs(af(1)-a0(1))/j_limit;
//    T1 = T1 > fabs(af(2)-a0(2))/j_limit ? T1 : fabs(af(2)-a0(2))/j_limit;
    double T2 = fabs(vf(0)-v0(0))/a_limit > fabs(vf(1)-v0(1))/a_limit ? fabs(vf(0)-v0(0))/a_limit : fabs(vf(1)-v0(1))/a_limit;
    T2 = T2 > fabs(vf(2)-v0(2))/a_limit ? T2 : fabs(vf(2)-v0(2))/a_limit;
    double T3 = fabs(pf(0)-p0(0))/v_limit > fabs(pf(1)-p0(1))/v_limit ? fabs(pf(0)-p0(0))/v_limit : fabs(pf(1)-p0(1))/v_limit;
    T3 = T3 > fabs(pf(2)-p0(2))/v_limit ? T3 : fabs(pf(2)-p0(2))/v_limit;

//    double T = T1 > T2 ? T1 : T2;
//    T = T > T3 ? T : T3;
    double T = T2;
    T = T > T3 ? T : T3;
    T = T < 0.3 ? 0.3 : T;

//    ROS_INFO_THROTTLE(2, "T=%lf", T);

    int times = T / delt_t;

    p = Eigen::MatrixXd::Zero(times, 3);
    v = Eigen::MatrixXd::Zero(times, 3);
    a = Eigen::MatrixXd::Zero(times, 3);
    t = Eigen::VectorXd::Zero(times);

    // % calculate optimal jerk controls by Mark W. Miller
    for(int ii=0; ii<3; ii++)
    {
        double delt_a = af(ii) - a0(ii);
        double delt_v = vf(ii) - v0(ii) - a0(ii)*T;
        double delt_p = pf(ii) - p0(ii) - v0(ii)*T - 0.5*a0(ii)*T*T;

        //%  if vf is not free
        double alpha = delt_a*60/pow(T,3) - delt_v*360/pow(T,4) + delt_p*720/pow(T,5);
        double beta = -delt_a*24/pow(T,2) + delt_v*168/pow(T,3) - delt_p*360/pow(T,4);
        double gamma = delt_a*3/T - delt_v*24/pow(T,2) + delt_p*60/pow(T,3);

        for(int jj=0; jj<times; jj++)
        {
            double tt = (jj + 1)*delt_t;
            t(jj) = tt;
            p(jj,ii) = alpha/120*pow(tt,5) + beta/24*pow(tt,4) + gamma/6*pow(tt,3) + a0(ii)/2*pow(tt,2) + v0(ii)*tt + p0(ii);
            v(jj,ii) = alpha/24*pow(tt,4) + beta/6*pow(tt,3) + gamma/2*pow(tt,2) + a0(ii)*tt + v0(ii);
            a(jj,ii) = alpha/6*pow(tt,3) + beta/2*pow(tt,2) + gamma*tt + a0(ii);
        }
    }
}

void compute_circular_traj(const double r, const double vel, const Eigen::Vector3d p0, const double t,
                           Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a)
//@requires r > 0 && vel > 0 && t >= 0;
{
    const double theta = vel*t/r;

    p(0) = r*cos(theta) + p0(0) - r;
    p(1) = r*sin(theta) + p0(1);
    p(2) = p0(2);

    v(0) = -vel*sin(theta);
    v(1) = vel*cos(theta);
    v(2) = 0.0;

    a(0) = -vel*vel/r*cos(theta);
    a(1) = -vel*vel/r*sin(theta);
    a(2) = 0.0;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "hover");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, stateCallback);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, positionCallback);

    pva_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/pva_setpoint", 1);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);

    const int LOOPRATE = 40;
    ros::Rate loop_rate(LOOPRATE);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        loop_rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    /// Take off with constant acceleration
    double take_off_height = 2.0;
    double take_off_acc = 1.0;

    double take_off_time_half = sqrt(take_off_height/take_off_acc);
    double delt_t = 1.0 / LOOPRATE;
    double take_off_send_times = take_off_time_half / delt_t * 2;
    int counter = 0;
    Vector3d recorded_takeoff_position(current_p(0), current_p(1), current_p(2));

    double yaw_set = 3.1415926;

    ROS_INFO("Arm and ta1111keoff");
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        trajectory_msgs::JointTrajectoryPoint pva_setpoint;


        static unsigned int count=0;
        count++;
        if(count>1000)
        {
            break;
        }
        double theta=180.0/180.0*3.14;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z =3;
        pose.pose.orientation.w=cos(theta/2);
//        ROS_INFO("theta:%f",theta);
//        ROS_INFO("W:%f",pose.pose.orientation.w);
        pose.pose.orientation.x=-0.0;
        pose.pose.orientation.y=0;
        pose.pose.orientation.z=sin(theta/2);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        loop_rate.sleep();

    }


    /** Take off complete. Go to a point with minimum jerk trajectory **/


    /** Now draw circle **/

    double init_t = 0.0;
    int mode=1;
    ROS_INFO("HOVER!!");
    while(ros::ok())
    {
//        Eigen::Vector3d p, v ,a;
//        compute_circular_traj(circle_radius, circle_speed, circle_p0, init_t, p, v, a);
        static int count=0;
        count++;
        if(count==800)
        {
            mode=-mode;
            count=0;
        }
        if(mode==1)
        {
            Eigen::Vector3d p(0,0,3);
            Eigen::Vector3d v(0,0,0);
            Eigen::Vector3d a(0,0,0);
            yaw_set=180.0/180*3.1415926;
            setPVA(p, v, a, yaw_set);//a_t.row(last_index));

//            geometry_msgs::PoseStamped pose;
//            pose.pose.position.x = 3;
//            pose.pose.position.y = 0;
//            pose.pose.position.z =3;
//            double theta=180.0/180.0*3.14;
//            pose.pose.orientation.w=cos(theta/2);
////        ROS_INFO("theta:%f",theta);
////        ROS_INFO("W:%f",pose.pose.orientation.w);
//            pose.pose.orientation.x=-0.0;
//            pose.pose.orientation.y=0;
//            pose.pose.orientation.z=sin(theta/2);
//            local_pos_pub.publish(pose);

        }
        else {
            Eigen::Vector3d p(0, 0, 2);
            Eigen::Vector3d v(0, 0, 0);
            Eigen::Vector3d a(0, 0, 0);
            yaw_set = 180.0 / 180 * 3.1415926;
            setPVA(p, v, a, yaw_set);//a_t.row(last_index));

//            geometry_msgs::PoseStamped pose;
//            pose.pose.position.x = 6;
//            pose.pose.position.y = 0;
//            pose.pose.position.z =3;
//            double theta=180.0/180.0*3.14;
//            pose.pose.orientation.w=cos(theta/2);
////        ROS_INFO("theta:%f",theta);
////        ROS_INFO("W:%f",pose.pose.orientation.w);
//            pose.pose.orientation.x=-0.0;
//            pose.pose.orientation.y=0;
//            pose.pose.orientation.z=sin(theta/2);
//            local_pos_pub.publish(pose);
//        }
        }


        init_t += delt_t;
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
