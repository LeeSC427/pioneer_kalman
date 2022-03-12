#include "pioneer_kalman/function.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pioneer_kalman");
 
    ros::NodeHandle nh;

    Kalman kalman;

    // kalman.landmark_location();

    // kalman.init_matrix(kalman.mean, kalman.cov);

    kalman.pub_goal = nh.advertise<std_msgs::Bool>("/chk_goal", 1);
    kalman.pub_pose = nh.advertise<geometry_msgs::Pose>("/kalman_mean",1);
    kalman.pub_robot_pose = nh.advertise<geometry_msgs::Pose>("/robot_pose", 1);
    
    ros::Subscriber sub_cmd_vel = nh.subscribe("/RosAria/cmd_vel", 10, &Kalman::get_cmd_vel, &kalman);
    ros::Subscriber sub_distance = nh.subscribe("/Marker_distance", 10, &Kalman::get_distance2marker, &kalman);
    ros::Subscriber sub_index = nh.subscribe("/Marker_pass", 10, &Kalman::get_index_info, &kalman);
    ros::Subscriber sub_see_marker = nh.subscribe("/Marker_onsight", 10, &Kalman::is_seen, &kalman);
    // kalman.pub_pose = nh.advertise<

    kalman.kalman_exec();

    ros::spin();

    return 0;
}