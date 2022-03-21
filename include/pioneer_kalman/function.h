#include "pioneer_kalman/header.h"

class Kalman
{
private:
    ros::NodeHandle nh;

public:

    struct landmark
    {
        double x, y;
        int index;

        landmark(): x(0.0), y(0.0), index(0)
        {
        }
        ~landmark()
        {
        }
    };

    ros::Publisher pub_pose;
    ros::Publisher pub_goal;
    ros::Publisher pub_robot_pose;
    
    ros::Subscriber sub_prev_turn;
    ros::Subscriber sub_after_turn;
    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_distance;
    ros::Subscriber sub_index;
    ros::Subscriber sub_see_marker;

    std::vector<landmark> lm_vec;
    
    bool initial_cmd_vel;
    bool is_goal;
    bool chk_marker; //to increase landmark index
    bool see_marker;
    bool is_prev;
    bool is_after;
    bool index_flag;
    bool next_index;
    bool turning;
    bool prev_flag;
    bool after_flag;
    bool cam_prev_flag;
    bool cam_after_flag;

    int landmark_num;
    int size;
    int cur_dest_landmark;
    
    double lin_vel, ang_vel; //subscribe
    double interval; //time duration
    double x_dist2landmark, y_dist2landmark, ang2landmark; //subscribe
    double robot_width;
    double robot2camera;
    double Kr, Kl;
    double prev_time;
    double dist_ref;
    double obs_error;

    cv::Mat mean, cov;
    cv::Mat prev_robot_pos;

    void kalman_exec()
    {
        double time = 0.0;

        // std::cout << "kalman_exec()" << std::endl;

        landmark_location();
        
        cv::Mat mean = cv::Mat::zeros(size, 1, CV_64F);
        cv::Mat cov = cv::Mat::zeros(size, size, CV_64F);

        init_matrix(mean, cov);

        ros::Rate rate(10);

        while(ros::ok())
        {
            kalman_loop(mean, cov);

            time += interval;

            // std::cout << "time: " << time << std::endl;

            ros::spinOnce();

            rate.sleep();
        }

    }

    void kalman_loop(cv::Mat &mean, cv::Mat &cov)
    {
        // double state2landmark;
        
        std::cout << "==================================" << turning << std::endl;
        update_index();

        landmark lm;

        // if(cur_dest_landmark <= landmark_num)
        // {
            for(int i = 0; i < landmark_num; i++)
            {
                if(cur_dest_landmark == lm_vec[i].index)
                {
                    lm = lm_vec[i];
                }
            }
            
            cv::Mat Fx = get_Fx();
            
            cv::Mat predicted_mean = mean_prediction(mean, Fx);
            
            // std::cout << predicted_mean.at<double>(0,0) << std::endl;

            cv::Mat predicted_cov = cov_prediction(mean, cov, Fx);
            
            if(see_marker)
            {
                double robot2marker = dist2landmark(lm.x - predicted_mean.at<double>(0,0), lm.y - predicted_mean.at<double>(1,0));
                
                cv::Mat h = measurement_model(predicted_mean, robot2marker);
                
                cv::Mat J = jacobian(robot2marker, predicted_mean, lm);
                
                cv::Mat K = get_gain(J, predicted_cov);
                
                double camera2marker = dist2landmark(x_dist2landmark, y_dist2landmark);
                
                mean = mean_correction(camera2marker, h, predicted_mean, K);
                
                cov = cov_correction(K, J, predicted_cov);
            }

            else
            {
                mean = predicted_mean;
                cov = predicted_cov;
            }

            final_mean(mean); 

            cv::Mat rb_pose = robot_mean(mean);

            // check_goal(mean, cov, lm);

            publish_odom(mean);

            publish_robot_pose(rb_pose);
        // }
        // else
        // {
            std::cout << "=================== MEAN ===================" << std::endl << mean << std::endl;
            // std::cout << "=================== COVARIANCE ===================" << std::endl << cov << std::endl;
        // }

    }

// =====================================================================//
// =========================== SUBSCRIBERS =============================//
// =====================================================================//

    void get_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
    {
        double cur_time = ros::Time::now().toSec();

        lin_vel = msg->linear.x;
        ang_vel = msg->angular.z;

        // std::cout << lin_vel << std::endl;

        if(!initial_cmd_vel)
            if(cur_time - prev_time > 0.05)
            {
                interval = cur_time - prev_time;

                std::cout << "TIME INTERVAL: " << interval << std::endl;
            }
            else
            {
                std::cout << "HI" << std::endl;
            }
        else
            initial_cmd_vel = false;

        prev_time = cur_time;

    }

    void get_distance2marker(const geometry_msgs::Twist::ConstPtr& msg)
    {
        x_dist2landmark = msg->linear.x;
        y_dist2landmark = msg->linear.y;
        ang2landmark = msg->angular.z;
    }

    void get_index_info(const std_msgs::Bool::ConstPtr& msg)
    {
        // bool next_index = msg->data;
        next_index = msg->data;
        if(next_index)
        {
            index_flag = true;
        //     update_index();
        }
    }

    void is_seen(const std_msgs::Bool::ConstPtr& msg)
    {
        see_marker = msg->data;
    }

    void prev_turn(const std_msgs::Bool::ConstPtr& msg)
    {
        is_prev = msg->data;

        if(is_prev)
        {
            prev_flag = true;
            cam_prev_flag = true;
        }
    }

    void after_turn(const std_msgs::Bool::ConstPtr& msg)
    {
        is_after = msg->data;
        
        if(is_after)
        {
            after_flag = true;
            cam_after_flag = true;
        }
    }

// =====================================================================//
// ======================== SET MARKER LOCATION ========================//
// =====================================================================//

    void update_index()
    {
        // if(next_index)
        // {
        //     cur_dest_landmark++;
        // }
        if(index_flag)
        {
            cur_dest_landmark++;
            index_flag = false;
        }
    }

// =====================================================================//
// ======================== SET MARKER LOCATION ========================//
// =====================================================================//
// =================== PLEASE MODIFY LOCATION LATER ====================//

    void landmark_location()
    {
        for(int i = 1; i <= landmark_num; i++)
        {
            landmark lm;
            lm.index = i;
            if(i == 1)
            {
                lm.x = 0.6;
                lm.y = 0.0;
            }
            else if(i == 2)
            {
                lm.x = 1.2;
                lm.y = 0.0;
            }
            else if(i == 3)
            {
                lm.x = 1.8;
                lm.y = 0.0;
            }
            else if(i == 4)
            {
                lm.x = 2.4;
                lm.y = 0.0;
            }
            else if(i == 5)
            {
                lm.x = 2.4;
                lm.y = -1.2;
            }
            else if(i == 6)
            {
                lm.x = 2.4;
                lm.y = -1.8;
            }
            else if(i == 7)
            {
                lm.x = 1.8;
                lm.y = -1.8;
            }
            else if(i == 8)
            {
                lm.x = 1.2;
                lm.y = -1.8;
            }
            else if(i == 9)
            {
                lm.x = 1.2;
                lm.y = -2.4;
            }
            else if(i == 10)
            {
                lm.x = 1.2;
                lm.y = -3.0;
            }
            else if(i == 11)
            {
                lm.x = 1.2;
                lm.y = -3.6;
            }

            lm_vec.push_back(lm);
        }
        // std::cout << "landmark_location" << std::endl;
    }

// =====================================================================//
// ===================== INITIATE MEAN & COVARIANCE ====================//
// =====================================================================//


    void init_matrix(cv::Mat &mean, cv::Mat &cov)
    {
        for(int i = 0; i < landmark_num; i++)
        {
            mean.at<double>(3 + 2 * i, 0) = lm_vec[i].x;
            mean.at<double>(4 + 2 * i, 0) = lm_vec[i].y;
        }

        for(int i = 3; i < size; i++)
        {
            cov.at<double>(i, i) = 0.025;
        }

        mean.at<double>(0,0) = robot2camera;
    }

// =====================================================================//
// ================== GET Fx TO MATCH MATRIX SIZE ======================//
// =====================================================================//

    cv::Mat get_Fx()
    {
        cv::Mat Fx = cv::Mat::zeros(3, size, CV_64F);

        Fx.at<double>(0,0) = 1.0;
        Fx.at<double>(1,1) = 1.0;
        Fx.at<double>(2,2) = 1.0;

        // std::cout << "get_Fx" << std::endl;

        return Fx;
    }

// =====================================================================//
// ========================== PREDICTIOIN STEP =========================//
// =====================================================================//

    cv::Mat mean_prediction(cv::Mat &mean, cv::Mat Fx)
    {
        // std::cout << mean.at<double>(2,0) << std::endl;

        cv::Mat mat_temp = cv::Mat::zeros(3, 1, CV_64F);

        mat_temp.at<double>(0,0) = lin_vel * interval * cos(mean.at<double>(2,0) + ang_vel * interval / 2.0);
        mat_temp.at<double>(1,0) = lin_vel * interval * sin(mean.at<double>(2,0) + ang_vel * interval / 2.0);
        mat_temp.at<double>(2,0) = ang_vel * interval;

        // std::cout << "mean_prediction" << std::endl;

        return mean + Fx.t() * mat_temp;
    }

    cv::Mat cov_prediction(cv::Mat &mean, cv::Mat &cov, cv::Mat &Fx)
    {
        cv::Mat Axk = cv::Mat::zeros(3, 3, CV_64F);
        Axk.at<double>(0,2) = -lin_vel * interval * sin(mean.at<double>(2,0) + ang_vel * interval /2.0);
        Axk.at<double>(1,2) = lin_vel * interval * cos(mean.at<double>(2,0) + ang_vel * interval /2.0);
        
        cv::Mat Ak = cv::Mat::eye(size, size, CV_64F) + Fx.t() * Axk * Fx;
        
        cv::Mat Wk = cv::Mat::zeros(size, 2, CV_64F);
        Wk.at<double>(0,0) = cos(mean.at<double>(2,0) + ang_vel * interval / 2.0) / 2.0 - (lin_vel * sin(mean.at<double>(2,0) + ang_vel * interval / 2.0)) / (2.0 * robot_width);
        Wk.at<double>(1,0) = sin(mean.at<double>(2,0) + ang_vel * interval / 2.0) / 2.0 + (lin_vel * cos(mean.at<double>(2,0) + ang_vel * interval / 2.0)) / (2.0 * robot_width);
        Wk.at<double>(2,0) = 1.0 / robot_width;
        Wk.at<double>(0,1) = cos(mean.at<double>(2,0) + ang_vel * interval / 2.0) / 2.0 + (lin_vel * sin(mean.at<double>(2,0) + ang_vel * interval / 2.0)) / (2.0 * robot_width);        
        Wk.at<double>(1,1) = sin(mean.at<double>(2,0) + ang_vel * interval / 2.0) / 2.0 - (lin_vel * cos(mean.at<double>(2,0) + ang_vel * interval / 2.0)) / (2.0 * robot_width);
        Wk.at<double>(2,1) = -1.0 / robot_width;

        cv::Mat Q = cv::Mat::zeros(2, 2, CV_64F);
        Q.at<double>(0,0) = Kr * abs(interval * (2.0 * lin_vel + robot_width * ang_vel)) / 2.0;
        Q.at<double>(1,1) = Kl * abs(interval * (2.0 * lin_vel - robot_width * ang_vel)) / 2.0;

        return Ak * cov * Ak.t() + Wk * Q * Wk.t();
    }
    
// =====================================================================//
// ========================= MEASUREMENT MODEL==========================//
// =====================================================================//

    cv::Mat measurement_model(cv::Mat &mean, double dist)
    {
        cv::Mat h = cv::Mat::zeros(2, 1, CV_64F);
        h.at<double>(0,0) = dist;
        h.at<double>(1,0) = atan2(y_dist2landmark, x_dist2landmark) - mean.at<double>(2,0);

        // std::cout << "measurement_model" << std::endl;

        return h;
    }

// =====================================================================//
// ===================== GET JACOBIAN MATRIX ===========================//
// =====================================================================//


    cv::Mat jacobian(double dist, cv::Mat &predict_mean, landmark lm)
    {
        cv::Mat low_Ht = cv::Mat(2, 5, CV_64F);
        
        low_Ht.at<double>(0,0) = -(lm.x - predict_mean.at<double>(0,0)) / dist;
        low_Ht.at<double>(0,1) = -(lm.y - predict_mean.at<double>(1,0)) / dist;
        low_Ht.at<double>(0,2) = 0.0;
        low_Ht.at<double>(0,3) = (lm.x - predict_mean.at<double>(0,0)) / dist;
        low_Ht.at<double>(0,4) = (lm.y - predict_mean.at<double>(1,0)) / dist;

        low_Ht.at<double>(1,0) = (lm.y - predict_mean.at<double>(1,0)) / pow(dist,2);
        low_Ht.at<double>(1,1) = -(lm.x - predict_mean.at<double>(0,0)) / pow(dist,2);
        low_Ht.at<double>(1,2) = -1.0;
        low_Ht.at<double>(1,3) = -(lm.y - predict_mean.at<double>(1,0)) / pow(dist,2);
        low_Ht.at<double>(1,4) = (lm.y - predict_mean.at<double>(0,0)) / pow(dist,2);

        cv::Mat Fxj = cv::Mat::zeros(5, size, CV_64F);

        for(int i = 0; i < 3; i++)
        {
            Fxj.at<double>(i,i) = 1.0;
        }

        Fxj.at<double>(3, 2 * cur_dest_landmark + 1) = 1.0;
        Fxj.at<double>(4, 2 * cur_dest_landmark + 2) = 1.0;

        // std::cout << "jacobian" << std::endl;

        return low_Ht * Fxj;
    }

// =====================================================================//
// ========================== GET  KALMAN GAIN =========================//
// =====================================================================//

    cv::Mat get_gain(cv::Mat &H, cv::Mat &predict_cov)
    {
        cv::Mat R = cv::Mat::zeros(2, 2, CV_64F);
        R.at<double>(0,0) = obs_error;
        R.at<double>(1,1) = obs_error;

        cv::Mat S = H * predict_cov * H.t() + R;

        // std::cout << "get_gain" << std::endl;

        return predict_cov * H.t() * S.inv();
    }

// =====================================================================//
// ========================= CORRECTION STEP ===========================//
// =====================================================================//

    cv::Mat mean_correction(double dist, cv::Mat &h, cv::Mat &predict_mean, cv::Mat &K)
    {
        cv::Mat Z = cv::Mat::zeros(2, 1, CV_64F);
        Z.at<double>(0,0) = dist;
        Z.at<double>(1,0) = ang2landmark;

        cv::Mat v = Z - h;

        // std::cout << "mean_correction" << std::endl;

        return predict_mean + K * v;
    }

    cv::Mat cov_correction(cv::Mat &K, cv::Mat &J, cv::Mat &predict_cov)
    {
        cv::Mat I = cv::Mat::eye(size,size,CV_64F);

        // std::cout << "cov_correction" << std::endl;

        return (I - K * J) * predict_cov;
    }

// =====================================================================//
// =========================== GET DISTANCE ============================//
// =====================================================================//

    double dist2landmark(double x_dist, double y_dist)
    {
        // std::cout << "dist2landmark" << std::endl;

        return sqrt(pow(x_dist, 2) + pow(y_dist, 2));
    }

// =====================================================================//
// ===================== CHECK GOAL & PUBLISH IS_GOAL =================//
// =====================================================================//

    // void check_goal(cv::Mat &mean, cv::Mat &cov, landmark lm)
    // {
    //     double x_dist = lm.x - mean.at<double>(0, 0);
    //     double y_dist = lm.y - mean.at<double>(1, 0);
    //     double dist = dist2landmark(x_dist, y_dist);

    //     std::cout << "mean to landmark: " << dist << std::endl;

    //     if(dist < dist_ref)
    //     {
    //         is_goal = true;
    //         std::cout << "ROBOT_POSITION: " << std::endl << mean << std::endl;
    //         std::cout << "POSITION_COVARIANCE: " << std::endl << cov << std::endl;
    //     }
    //     else
    //         is_goal = false;
        
    //     std_msgs::Bool boolean_msg;

    //     boolean_msg.data = is_goal;

    //     pub_goal.publish(boolean_msg);
    // }

// =====================================================================//
// =========================== PUBLISH POSES ===========================//
// =====================================================================//

    void publish_odom(cv::Mat &mean)
    {
        geometry_msgs::Pose odom;

        double x = mean.at<double>(0,0);
        double y = mean.at<double>(1,0);
        double yaw = mean.at<double>(2,0);

        tf::Quaternion Q_temp;
        Q_temp.setRPY(0.0, 0.0, yaw);

        geometry_msgs::Quaternion Q;

        tf::quaternionTFToMsg(Q_temp, Q);

        odom.position.x = x;
        odom.position.y = y;
        odom.position.z = 0.0;
        odom.orientation = Q;

        pub_pose.publish(odom);
    }

    void publish_robot_pose(cv::Mat &rb_pose)
    {
        geometry_msgs::Pose robot_pose;

        double x = rb_pose.at<double>(0,0);
        double y = rb_pose.at<double>(1,0);
        double yaw = rb_pose.at<double>(2,0);

        tf::Quaternion Q_temp;
        Q_temp.setRPY(0.0, 0.0, yaw);

        geometry_msgs::Quaternion Q;

        tf::quaternionTFToMsg(Q_temp, Q);

        robot_pose.position.x = x;
        robot_pose.position.y = y;
        robot_pose.position.z = 0.0;
        robot_pose.orientation = Q;

        pub_robot_pose.publish(robot_pose);
    }

// =====================================================================//
// ======================= REVISE MEAN TH ==============================//
// =====================================================================//

    void final_mean(cv::Mat &mean)
    {
        double temp_ang = 0.0;
        
        if(mean.at<double>(2,0) > CV_PI)
        {
            temp_ang = mean.at<double>(2,0) - CV_PI;
            mean.at<double>(2,0) = -(CV_PI - temp_ang);
        }
        else if(mean.at<double>(2,0) < -CV_PI)
        {
            temp_ang = -mean.at<double>(2,0) - CV_PI;
            mean.at<double>(2,0) = CV_PI - temp_ang;
        }

        cv::Mat temp_mean = cv::Mat::zeros(size, 1, CV_64F);
        //BEFORE TURN
        if(cam_prev_flag)
        {
            if( -CV_PI / 4.0 <= mean.at<double>(2,0) && mean.at<double>(2,0) <= CV_PI / 4.0)
                mean.at<double>(0,0) -= robot2camera;
            else if(-CV_PI / 4.0 > mean.at<double>(2,0) && mean.at<double>(2,0) >= -3.0 * CV_PI / 4.0)
                mean.at<double>(1,0) += robot2camera;
            else if(-3.0 * CV_PI / 4.0 > mean.at<double>(2,0) || 3.0 * CV_PI / 4.0 <= mean.at<double>(2,0))
                mean.at<double>(0,0) += robot2camera;

            cam_prev_flag = false;
            std::cout << "============= BEFORE TURN ==============" << std::endl;
        }
        //AFTER TURN
        if(cam_after_flag)
        {
            if( -CV_PI / 4.0 > mean.at<double>(2,0) && mean.at<double>(2,0) >= -3.0 * CV_PI / 4.0)
                mean.at<double>(1,0) -= robot2camera;
            else if(-3.0 * CV_PI / 4.0 > mean.at<double>(2,0) || 3.0 * CV_PI / 4.0 <= mean.at<double>(2,0))
                mean.at<double>(0,0) -= robot2camera;

            cam_after_flag = false;
            std::cout << "============= AFTER TURN ==============" << std::endl;
        }
    }

// =====================================================================//
// ===================== MOVE MEAN TO ROBOT CENTER =====================//
// =====================================================================//

    cv::Mat robot_mean(cv::Mat &mean)
    {
        cv::Mat rb_mean = cv::Mat::zeros(3, 1, CV_64F);

        cv::Mat temp_mean = cv::Mat::zeros(3, 1, CV_64F);
        
        if(prev_flag)
        {
            turning = true;
            prev_flag = false;
        }

        if(after_flag)
        {
            turning = false;
            after_flag = false;
        }
        
        if(!turning)
        {
            rb_mean.at<double>(0,0) = mean.at<double>(0,0) - robot2camera * cos(mean.at<double>(2,0));
            rb_mean.at<double>(1,0) = mean.at<double>(1,0) + robot2camera * sin(mean.at<double>(2,0));
            rb_mean.at<double>(2,0) = mean.at<double>(2,0);
        }
        else
        {
            rb_mean = prev_robot_pos;
            rb_mean.at<double>(2,0) = mean.at<double>(2,0);
        }


        prev_robot_pos = rb_mean;

        return rb_mean;
    }

    Kalman()
    {
        pub_goal = nh.advertise<std_msgs::Bool>("/chk_goal", 1);
        pub_pose = nh.advertise<geometry_msgs::Pose>("/kalman_mean",1);
        pub_robot_pose = nh.advertise<geometry_msgs::Pose>("/robot_pose", 1);

        sub_prev_turn = nh.subscribe("/pos_spin_prev", 10, &Kalman::prev_turn, this);
        sub_after_turn = nh.subscribe("/pos_spin_after", 10, &Kalman::after_turn, this);
        sub_cmd_vel = nh.subscribe("/RosAria/cmd_vel", 10, &Kalman::get_cmd_vel, this);
        sub_distance = nh.subscribe("/Marker_distance", 10, &Kalman::get_distance2marker, this);
        sub_index = nh.subscribe("/Marker_center", 10, &Kalman::get_index_info, this);
        sub_see_marker = nh.subscribe("/Marker_onsight", 10, &Kalman::is_seen, this);

        initial_cmd_vel = true;
        is_goal = false;
        chk_marker = false;
        see_marker = false;
        is_prev = false;
        is_after = false;
        index_flag = false;
        next_index = false;
        turning = false;
        prev_flag = false;
        after_flag = false;
        cam_prev_flag = false;
        cam_after_flag = false;

        landmark_num = 11;
        size = 3 + 2 * landmark_num;
        cur_dest_landmark = 1;
    
        lin_vel = 0.0;
        ang_vel = 0.0;
        interval = 0.0;
        x_dist2landmark = 0.0;
        y_dist2landmark = 0.0;
        ang2landmark = 0.0;
        robot_width = 0.4;
        robot2camera = 0.4;
        Kr = 0.1;
        Kl = 0.1;
        prev_time = ros::Time::now().toSec();
        dist_ref = 0.1;
        obs_error = 0.01;

        prev_robot_pos = cv::Mat::zeros(3,1,CV_64F);
    }
    ~Kalman()
    {
    }
};
