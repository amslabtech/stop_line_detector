#include "stop_line_detector/stop_line_detector.h"

StopLineDetector::StopLineDetector(void)
:local_nh("~"), it(nh)
{
    image_sub = it.subscribe("/camera/color/image_raw", 1, &StopLineDetector::image_callback, this);
    local_nh.param("UP_LEFT_U", UP_LEFT_U, {234});
    local_nh.param("UP_LEFT_V", UP_LEFT_V, {152});
    local_nh.param("DOWN_LEFT_U", DOWN_LEFT_U, {192});
    local_nh.param("DOWN_LEFT_V", DOWN_LEFT_V, {370});
    local_nh.param("DOWN_RIGHT_U", DOWN_RIGHT_U, {560});
    local_nh.param("DOWN_RIGHT_V", DOWN_RIGHT_V, {375});
    local_nh.param("UP_RIGHT_U", UP_RIGHT_U, {492});
    local_nh.param("UP_RIGHT_V", UP_RIGHT_V, {152});
    local_nh.param("UP_LEFT", UP_LEFT, {50});
    local_nh.param("TRANSFORM_WIDTH", TRANSFORM_WIDTH, {100});
    local_nh.param("TRANSFORM_HEIGHT", TRANSFORM_HEIGHT, {100});

    std::cout << "stop line detector" << std::endl;
    std::cout << "UP_LEFT_U: " << UP_LEFT_U << std::endl;
    std::cout << "UP_LEFT_V: " << UP_LEFT_V << std::endl;
    std::cout << "DOWN_LEFT_U: " << DOWN_LEFT_U << std::endl;
    std::cout << "DOWN_LEFT_V: " << DOWN_LEFT_V << std::endl;
    std::cout << "DOWN_RIGHT_U: " << DOWN_RIGHT_U << std::endl;
    std::cout << "DOWN_RIGHT_V: " << DOWN_RIGHT_V << std::endl;
    std::cout << "UP_RIGHT_U: " << UP_RIGHT_U << std::endl;
    std::cout << "UP_RIGHT_V: " << UP_RIGHT_V << std::endl;
    std::cout << "UP_LEFT: " << UP_LEFT << std::endl;
    std::cout << "TRANSFORM_WIDTH: " << TRANSFORM_WIDTH << std::endl;
    std::cout << "TRANSFORM_HEIGHT: " << TRANSFORM_HEIGHT << std::endl;

    std::vector<cv::Point2f> src_pts = {cv::Point2f(UP_LEFT_U, UP_LEFT_V),
                                         cv::Point2f(DOWN_LEFT_U, DOWN_LEFT_V),
                                         cv::Point2f(DOWN_RIGHT_U, DOWN_RIGHT_V),
                                         cv::Point2f(UP_RIGHT_U, UP_RIGHT_V)};

    std::vector<cv::Point2f> dst_pts = {cv::Point2f(UP_LEFT, UP_LEFT),
                                           cv::Point2f(UP_LEFT, UP_LEFT + TRANSFORM_HEIGHT),
                                           cv::Point2f(UP_LEFT + TRANSFORM_WIDTH, UP_LEFT + TRANSFORM_HEIGHT),
                                           cv::Point2f(UP_LEFT + TRANSFORM_WIDTH, UP_LEFT)};

    homography_matrix = cv::getPerspectiveTransform(src_pts, dst_pts);
}

void StopLineDetector::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
    try{
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }catch(cv_bridge::Exception& ex){
        ROS_ERROR("cv_bridge exception: %s", ex.what());
    }

    cv::Mat dst_image = image;
    cv::warpPerspective(image, dst_image, homography_matrix, dst_image.size());

    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", dst_image);
    cv::waitKey(1);
}

void StopLineDetector::process(void)
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_line_detector");
    StopLineDetector sd;
    sd.process();
    return 0;
}
