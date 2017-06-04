#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h> 
using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    VideoCapture cap(0);
    cap.set(CAP_PROP_FPS, 60);
    cap.set(CAP_PROP_BUFFERSIZE, 1);
    Mat src, hsv, threshold_blue;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    short int old_i = 0;
    Point2f center;
    float radius;
    bool ifExist = false;

    ros::init(argc, argv, "middlePoint_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg_pid_xy;
    ros::Publisher pub_msg_pid_xy = nh.advertise<geometry_msgs::Twist>("pid_xy", 1000000);
    double kp_roll = 0, ki_roll = 0, kd_roll = 0, kp_pitch = 0, ki_pitch = 0, kd_pitch = 0;
    double error_x = 0, error_y = 0;
    double previous_error_x = 0, previous_error_y = 0, integral_x = 0, integral_y = 0, derivative_x = 0, derivative_y = 0;
    double control_value_x = 0, control_value_y = 0;
    double finish_time0 = 0.0;

    double total_x = 0, total_y = 0, temp_x = 0, temp_y = 0;
    double th_x = 0, th_y = 0;
    int b = 0;

    ros::param::set("pid_kp_roll", 0); ros::param::set("pid_kp_pitch", 0);
    ros::param::set("pid_ki_roll", 0); ros::param::set("pid_ki_pitch", 0);
    ros::param::set("pid_kd_roll", 0); ros::param::set("pid_kd_pitch", 0);
    ros::Rate r(100000);

    while (ros::ok())
    {
        ros::param::getCached("pid_kp_roll", kp_roll); ros::param::getCached("pid_kp_pitch", kp_pitch);
        ros::param::getCached("pid_ki_roll", ki_roll); ros::param::getCached("pid_ki_pitch", ki_pitch);
        ros::param::getCached("pid_kd_roll", kd_roll); ros::param::getCached("pid_kd_pitch", kd_pitch);
        cap >> src;
        hsv = src.clone();
        cvtColor(hsv, hsv, CV_BGR2HSV);
        if (!hsv.empty())
        {
            inRange(hsv, Scalar(105, 100, 0), Scalar(120, 255, 255), threshold_blue);
            if (!threshold_blue.empty())
            {
                findContours(threshold_blue, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
                if (!contours.empty())
                {
                    for (int i = 0; i < contours.size(); i++)
                    {
                        total_x += contours[i][0].x;
                        total_y += contours[i][0].y;
                        b = i;
                        if (hierarchy[i][0] == -1)
                        {
                            if (hierarchy[i][1] == -1)
                            {
                                if (hierarchy[i][2] != -1)
                                {
                                    if (hierarchy[i][3] != -1)
                                    {
                                        line(src, Point(300, 240), Point(340, 240), Scalar(0, 255, 0), 3);
                                        line(src, Point(320, 220), Point(320, 260), Scalar(0, 255, 0), 3);
                                        old_i = i;
                                        ifExist = true;
                                    }
                                }
                            }
                        }
                    }
                    if (ifExist)
                    {
                        minEnclosingCircle(contours[old_i], center, radius);
                        line(src, center, center, Scalar(0, 0, 255), 10);
                        if (!contours[old_i].empty())
                        {
                            if (radius <= 11)
                            {
                                error_x = center.x - 320.0;
                                error_y = 240.0 - center.y;
                            }
                            else if (radius <= 16)
                            {
                                error_x = (center.x - 320.0) / 1.25;
                                error_y = (240.0 - center.y) / 1.25;
                            }
                            else if (radius <= 21)
                            {
                                error_x = (center.x - 320.0) / 2;
                                error_y = (240.0 - center.y) / 2;
                            }
                            else
                            {
                                error_x = (center.x - 320.0) / 10;
                                error_y = (240.0 - center.y) / 10;
                            }
                        }
                    }
                    else
                    {
                        line(src, contours[0][0], contours[0][0], Scalar(255, 0, 0), 15);
                        total_x = total_x / b;
                        total_y = total_y / b;
                        double delta_x = abs(total_x - temp_x);
                        double delta_y = abs(total_y - temp_y);
                        double dt_x = abs(temp_x - th_x);
                        double dt_y = abs(temp_y - th_y);
                        if ((delta_x <10.0) && (delta_y < 10.0) && (dt_x <10.0) && (dt_y<10.0))
                        {
                            //imshow("there", threshold_blue);
                            cout << "avr x " << total_x << " avr y " << total_y << " temp_x " << temp_x << " temp_y " << temp_y << " th_x " << th_x << " th_y " << th_y << endl;;
                            circle(src, Point(total_x, total_y), 3, Scalar(255, 255, 255), 5);

                            error_x = total_x - 320.0;
                            error_y = 240.0 - total_y;
                        }
                        temp_x = total_x;
                        temp_y = total_y;
                        th_x = temp_x;
                        th_y = temp_y;
                        total_x = 0;
                        total_y = 0;
                    }
                    ifExist = false;

                }
            }
        }

        double finish_time1 = clock() / 1000000.0;
        double dt = finish_time1 - finish_time0;
        finish_time0 = finish_time1;
        integral_x = integral_x + error_x*dt; integral_y = integral_y + error_y*dt;
        derivative_x = (error_x - previous_error_x) / dt; derivative_y = (error_y - previous_error_y) / dt;
        control_value_x = kp_roll*error_x + ki_roll*integral_x + kd_roll*derivative_x;
        control_value_y = kp_pitch*error_y + ki_pitch*integral_y + kd_pitch*derivative_y;
        if (control_value_x > 30.0) control_value_x = 30.0;
        if (control_value_y > 30.0) control_value_y = 30.0;
        msg_pid_xy.linear.x = control_value_x;
        msg_pid_xy.linear.y = control_value_y;
        //double cpp_count = clock()/1000000.0;
        //ROS_INFO_STREAM("  "<<cpp_count<<"    "<<ros::Time::now());
        pub_msg_pid_xy.publish(msg_pid_xy);
        previous_error_x = error_x;
        previous_error_y = error_y;
        integral_x = 0.0; integral_y = 0.0;
        imshow("src", src);
        waitKey(5);
        r.sleep();
    }
    return 0;
}

