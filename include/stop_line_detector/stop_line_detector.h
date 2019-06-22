#ifndef __STOP_LINE_DETECTOR_H
#define __STOP_LINE_DETECTOR_H

#include <ros/ros.h>

#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

class StopLineDetector
{
public:
    StopLineDetector(void);

    void process(void);
    void image_callback(const sensor_msgs::ImageConstPtr&);
    void detect_stop_line(const cv::Mat&);
    double get_angle(const cv::Vec4i&);
    double get_length(const cv::Vec4i&);

private:
    double UP_LEFT_U;
    double UP_LEFT_V;
    double DOWN_LEFT_U;
    double DOWN_LEFT_V;
    double DOWN_RIGHT_U;
    double DOWN_RIGHT_V;
    double UP_RIGHT_U;
    double UP_RIGHT_V;
    double UP_LEFT;
    double TRANSFORM_WIDTH;
    double TRANSFORM_HEIGHT;
    double H_MIN;
    double H_MAX;
    double S_MIN;
    double S_MAX;
    double V_MIN;
    double V_MAX;
    int HOUGH_THRESHOLD;
    double MIN_LINE_LENGTH;
    double MAX_LINE_GAP;
    double ANGLE_DIFF_THRESHOLD;
    double MIN_DISTANCE_LIMIT;
    double MAX_DISTANCE_LIMIT;
    bool SHOW_IMAGE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    cv::Mat homography_matrix;
};

#endif// __STOP_LINE_DETECTOR_H
