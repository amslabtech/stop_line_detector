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
    local_nh.param("H_MIN", H_MIN, {0});
    local_nh.param("H_MAX", H_MAX, {180});
    local_nh.param("S_MIN", S_MIN, {0});
    local_nh.param("S_MAX", S_MAX, {255});
    local_nh.param("V_MIN", V_MIN, {0});
    local_nh.param("V_MAX", V_MAX, {255});
    local_nh.param("HOUGH_THRESHOLD", HOUGH_THRESHOLD, {50});
    local_nh.param("MIN_LINE_LENGTH", MIN_LINE_LENGTH, {40});
    local_nh.param("MAX_LINE_GAP", MAX_LINE_GAP, {100});

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
    std::cout << "H_MIN: " << H_MIN << std::endl;
    std::cout << "H_MAX: " << H_MAX << std::endl;
    std::cout << "S_MIN: " << S_MIN << std::endl;
    std::cout << "S_MAX: " << S_MAX << std::endl;
    std::cout << "V_MIN: " << V_MIN << std::endl;
    std::cout << "V_MAX: " << V_MAX << std::endl;
    std::cout << "HOUGH_THRESHOLD: " << HOUGH_THRESHOLD << std::endl;
    std::cout << "MIN_LINE_LENGTH: " << MIN_LINE_LENGTH << std::endl;
    std::cout << "MAX_LINE_GAP: " << MAX_LINE_GAP << std::endl;

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
    double start = ros::Time::now().toSec();
    cv::Mat image;
    try{
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }catch(cv_bridge::Exception& ex){
        ROS_ERROR("cv_bridge exception: %s", ex.what());
    }

    cv::Mat dst_image = image;
    cv::warpPerspective(image, dst_image, homography_matrix, dst_image.size());
    cv::medianBlur(dst_image, dst_image, 3);

    cv::Mat hsv_image;
    cv::cvtColor(dst_image, hsv_image, CV_BGR2HSV);

    cv::Scalar hsv_min(H_MIN, S_MIN, V_MIN);
    cv::Scalar hsv_max(H_MAX, S_MAX, V_MAX);
    cv::Mat mask_image;
    cv::inRange(hsv_image, hsv_min, hsv_max, mask_image);

    cv::Mat filtered_image;
    cv::medianBlur(mask_image, filtered_image, 3);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(filtered_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    cv::Mat contour_image;
    cv::cvtColor(filtered_image, contour_image, CV_GRAY2BGR);
    //--------------//
    cv::Scalar contour_color(255, 0, 0);
    cv::drawContours(contour_image, contours, -1, contour_color, 1, 8, hierarchy);

    int contour_num = contours.size();
    for(int i=0;i<contour_num;i++){
        double area = cv::contourArea(contours[i]);
        if(area < 100){
            continue;
        }
        cv::Rect bb = cv::boundingRect(contours[i]);
        cv::Scalar bb_color(0, 255, 0);
        cv::rectangle(contour_image, bb.tl(), bb.br(), bb_color, 1);
    }
    //--------------//
    cv::Mat canny_image;
    cv::Canny(filtered_image, canny_image, filtered_image.rows*0.1, filtered_image.rows*0.1, 3, false);
    std::vector<cv::Vec4i> hough_lines;
    cv::HoughLinesP(canny_image, hough_lines, 1, M_PI / 180, HOUGH_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);
    std::vector<cv::Vec4i>::iterator it = hough_lines.begin();
    cv::Mat line_image;
    cv::cvtColor(filtered_image, line_image, CV_GRAY2BGR);
    for (; it != hough_lines.end(); ++it) {
        cv::Vec4i l = *it;
        cv::line(line_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 1, CV_AA);
    }
    std::cout << hough_lines.size() << " lines detected" << std::endl;

    std::cout << ros::Time::now().toSec() - start << "[s]" << std::endl;
    /*
    cv::namedWindow("transformed_image", cv::WINDOW_NORMAL);
    cv::imshow("transformed_image", dst_image);
    cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
    cv::imshow("filtered_image", filtered_image);
    cv::namedWindow("hsv_image", cv::WINDOW_NORMAL);
    cv::imshow("hsv_image", hsv_image);
    cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
    cv::imshow("filtered_image", filtered_image);
    */
    cv::namedWindow("contour_image", cv::WINDOW_NORMAL);
    cv::imshow("contour_image", contour_image);
    cv::namedWindow("line_image", cv::WINDOW_NORMAL);
    cv::imshow("line_image", line_image);
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
