#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
////#include <std_msgs/Bool.h>
#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
#define pi 3.14159265358979323846

int main(int argc, char** argv)
{
    VideoCapture cap(0);
    VideoWriter writer;
    writer.open("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 15, Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT)));
    Mat src, hsv, threshold_blue;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    short int old_i = 0;
    Point2f center;
    float radius;
    bool ifExist = false;
    

    ros::init(argc, argv, "middlePoint_node");
    ros::NodeHandle nh;
    geometry_msgs::Twist msg_xy;
    ros::Publisher pub_msg_xy = nh.advertise<geometry_msgs::Twist>("mid_xy", 1000);

    ////std_msgs::Bool msg_if_image;

    while (1)
    {
        cap >> src;
        hsv = src.clone();
        cvtColor(hsv, hsv, CV_BGR2HSV);
        if (!hsv.empty())
        {
            inRange(hsv, Scalar(72, 119, 75), Scalar(138, 255, 255), threshold_blue);
            //imshow("threshold_blue", threshold_blue);
            if (!threshold_blue.empty())
            {
                findContours(threshold_blue, contours, hierarchy, CV_RETR_TREE, CHAIN_APPROX_NONE, Point(0, 0));
                if (!contours.empty())
                {
                    for (int i = 0; i < contours.size(); i++)
                    {
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
                        //cout << "center:  " << center << endl;                       
                        //contours[0][0].x = -contours[0][0].x;
                        //contours[0][0].y = -contours[0][0].y;
                        msg_xy.linear.x = center.x; msg_xy.linear.y = center.y;
                        msg_xy.angular.x = double(320.0); msg_xy.angular.y = double(240.0);

                    }
                    else
                    {
                        //cout <<contours[0][0].x << contours[0][0].y << endl;
                        //center = -center;
                        msg_xy.angular.x = contours[0][0].x; msg_xy.angular.y = contours[0][0].y;
                        msg_xy.linear.x = double(320.0); msg_xy.linear.y = double(240.0);
                        line(src, Point(contours[0][0].x, contours[0][0].y), Point(contours[0][0].x, contours[0][0].y), Scalar(0, 255, 0), 5);
                    }
                    line(src, center, center, Scalar(0, 0, 255), 10);
                    //contours[0][0].x = 0; contours[0][0].y = 0;                                                                

                    ifExist = false;
                }
            }
            
        }
        pub_msg_xy.publish(msg_xy);
        imshow("src", src);
        //imshow("threshold_contour", threshold);
        writer.write(src);
        if (waitKey(16) == 27) break;
    }
    return 0;
}

