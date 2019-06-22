#include "stop_line_detector/stop_line_detector.h"

StopLineDetector::StopLineDetector(void)
:local_nh("~"), it(nh)
{
    image_sub = it.subscribe("/camera/color/image_raw", 1, &StopLineDetector::image_callback, this);
    line_flag_pub = nh.advertise<std_msgs::Bool>("/recognition/stop_line", 1);

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
    local_nh.param("ANGLE_DIFF_THRESHOLD", ANGLE_DIFF_THRESHOLD, {0.01});
    local_nh.param("MIN_DISTANCE_LIMIT", MIN_DISTANCE_LIMIT, {8});
    local_nh.param("MAX_DISTANCE_LIMIT", MAX_DISTANCE_LIMIT, {16});
    local_nh.param("SHOW_IMAGE", SHOW_IMAGE, {false});
    local_nh.param("LINE_POSITION_V_THRESHOLD", LINE_POSITION_V_THRESHOLD, {240});

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
    std::cout << "ANGLE_DIFF_THRESHOLD: " << ANGLE_DIFF_THRESHOLD << std::endl;
    std::cout << "MAX_DISTANCE_LIMIT: " << MAX_DISTANCE_LIMIT << std::endl;
    std::cout << "MIN_DISTANCE_LIMIT: " << MIN_DISTANCE_LIMIT << std::endl;
    std::cout << "SHOW_IMAGE: " << SHOW_IMAGE << std::endl;
    std::cout << "LINE_POSITION_V_THRESHOLD: " << LINE_POSITION_V_THRESHOLD << std::endl;

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
    std::cout << "--- callback ---" << std::endl;
    cv::Mat image;
    try{
        image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    }catch(cv_bridge::Exception& ex){
        ROS_ERROR("cv_bridge exception: %s", ex.what());
    }
    try{
        detect_stop_line(image);
    }catch(cv::Exception& ex){
        std::cout << ex.what() << std::endl;
    }
}

void StopLineDetector::detect_stop_line(const cv::Mat& image)
{
    double start = ros::Time::now().toSec();
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

    cv::Mat canny_image;
    cv::Canny(filtered_image, canny_image, filtered_image.rows*0.1, filtered_image.rows*0.1, 3, false);
    std::vector<cv::Vec4i> hough_lines;
    cv::HoughLinesP(canny_image, hough_lines, 1, M_PI / 180, HOUGH_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);
    cv::Mat line_image = dst_image;

    std::vector<cv::Vec4i> lines;
    std::vector<cv::Point> centers;

    for(auto it=hough_lines.begin();it!=hough_lines.end();++it){
        cv::Vec4i l = *it;
        double angle = get_angle(l);
        for(auto it2=it+1;it2!= hough_lines.end();++it2){
            cv::Vec4i l2 = *it2;
            double angle2 = get_angle(l2);
            double angle_diff = fabs(angle - angle2);
            if(angle_diff < ANGLE_DIFF_THRESHOLD){
                //std::cout << "angle_diff: " << angle_diff << "[rad]" << std::endl;
                double distance = 100;
                if(fabs(l[2] - l[0]) < 1e-6){
                    distance = fabs(l[3] - l[1]);
                }else{
                    double a = (l[3] - l[1]) / double(l[2] - l[0]);
                    double b = -1;
                    double c = l[1] - a * l[0];
                    distance = fabs(a * l2[0] + b * l2[1] + c) / sqrt(a * a + b * b);
                }
                if(MIN_DISTANCE_LIMIT < distance && distance < MAX_DISTANCE_LIMIT){
                    bool registered_flag = false;
                    for(auto line : lines){
                        if(fabs(get_angle(line) - get_angle(l2)) < ANGLE_DIFF_THRESHOLD){
                            registered_flag = true;
                        }
                    }
                    if(registered_flag){
                        continue;
                    }else{
                        lines.push_back(l2);
                    }
                    std::cout << "distance: " << distance << "[px]" << std::endl;
                    std::cout << "stop line" << std::endl;
                    std::cout << l << std::endl;
                    std::cout << get_length(l) << std::endl;
                    std::cout << l2 << std::endl;
                    std::cout << get_length(l2) << std::endl;
                    cv::Scalar color(l[0] / (double)image.cols * 255, l[1] / (double)image.rows * 255, 0);
                    cv::line(line_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 1, CV_AA);
                    cv::line(line_image, cv::Point(l2[0], l2[1]), cv::Point(l2[2], l2[3]), color, 1, CV_AA);
                    cv::Point center((l[0] + l[2] + l2[0] + l2[2]) / 4.0, (l[1] + l[3] + l2[1] + l2[3]) / 4.0);
                    centers.push_back(center);
                    cv::putText(line_image, "line", center, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, CV_AA);
                    break;
                }
            }
        }
    }
    int n = lines.size();
    std::cout << "detected lines: " << n << std::endl;
    for(int i=0;i<n;i++){
        std::cout << "detected line " << i << ": " << std::endl;
        std::cout << lines[i] << std::endl;
        std::cout << centers[i] << std::endl;
        std::cout << centers[i].y << std::endl;
        std::cout << LINE_POSITION_V_THRESHOLD << std::endl;
        if(LINE_POSITION_V_THRESHOLD < centers[i].y){
            std::cout << "!!! line !!!" << std::endl;
            std_msgs::Bool flag;
            flag.data = true;
            line_flag_pub.publish(flag);
        }
    }
    cv::Mat result_image;
    cv::warpPerspective(line_image, result_image, homography_matrix.inv(), result_image.size());

    std::cout << ros::Time::now().toSec() - start << "[s]" << std::endl;
    if(SHOW_IMAGE){
        /*
        cv::namedWindow("transformed_image", cv::WINDOW_NORMAL);
        cv::imshow("transformed_image", dst_image);
        cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
        cv::imshow("filtered_image", filtered_image);
        cv::namedWindow("hsv_image", cv::WINDOW_NORMAL);
        cv::imshow("hsv_image", hsv_image);
        cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
        cv::imshow("filtered_image", filtered_image);
        cv::namedWindow("line_image", cv::WINDOW_NORMAL);
        cv::imshow("line_image", line_image);
        */
        cv::namedWindow("result_image", cv::WINDOW_NORMAL);
        cv::imshow("result_image", result_image);
        cv::waitKey(1);
    }
}

double StopLineDetector::get_angle(const cv::Vec4i& l)
{
    return atan2(l[3] - l[1], l[2] - l[0]);
}

double StopLineDetector::get_length(const cv::Vec4i& l)
{
    double du = l[0] - l[2];
    double dv = l[1] - l[3];
    return sqrt(du * du + dv * dv);
}

void StopLineDetector::process(void)
{
    std::cout << "=== stop line detector ===" << std::endl;
    std::cout << "waiting for image..." << std::endl;
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop_line_detector");
    StopLineDetector sd;
    sd.process();
    return 0;
}
