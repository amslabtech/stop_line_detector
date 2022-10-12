#include "stop_line_detector/stop_line_detector.h"

StopLineDetector::StopLineDetector(void)
:local_nh("~"), it(nh)
{
    image_sub = it.subscribe("/camera/color/image_raw", 1, &StopLineDetector::image_callback, this);
    pose_sub = nh.subscribe("/estimated_pose/pose", 1, &StopLineDetector::pose_callback, this);
    edge_sub = nh.subscribe("/estimated_pose/edge", 1, &StopLineDetector::edge_callback, this);

    line_pub = nh.advertise<amsl_navigation_msgs::StopLine>("/recognition/stop_line", 1);

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
    local_nh.param("APPROX_EPSILON", APPROX_EPSILON, {4.0});
    local_nh.param("POLY_THICKNESS", POLY_THICKNESS, {2.0});
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
    std::cout << "APPROX_EPSILON: " << APPROX_EPSILON << std::endl;
    std::cout << "POLY_THICKNESS: " << POLY_THICKNESS << std::endl;
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
    //std::cout << "--- callback ---" << std::endl;
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

void StopLineDetector::pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
    robot_direction = tf::getYaw(msg->pose.pose.orientation);
}

void StopLineDetector::edge_callback(const amsl_navigation_msgs::EdgeConstPtr& msg)
{
    edge_direction = msg->direction;
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
    cv::threshold(mask_image, mask_image, 0, 255, cv::THRESH_BINARY);

    cv::Mat filtered_image;
    cv::morphologyEx(mask_image, filtered_image, cv::MORPH_CLOSE, cv::Mat());
    cv::morphologyEx(filtered_image, filtered_image, cv::MORPH_OPEN, cv::Mat());

    cv::medianBlur(filtered_image, filtered_image, 3);
    cv::Mat canny_image;
    cv::Canny(filtered_image, canny_image, filtered_image.rows*0.1, filtered_image.rows*0.1, 3, false);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat contour_image = cv::Mat::zeros(filtered_image.size(), CV_8U);
    cv::findContours(canny_image, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    cv::Mat hough_image = contour_image.clone();
    std::vector<std::vector<cv::Point>> approx(contours.size());
    for(size_t i=0; i<contours.size(); i++){
        cv::approxPolyDP(cv::Mat(contours[i]), approx[i], APPROX_EPSILON, false);
        cv::polylines(contour_image, approx, false, cv::Scalar(255, 255, 255), POLY_THICKNESS);
    }
    std::vector<cv::Vec4i> hough_lines;
    cv::HoughLinesP(contour_image, hough_lines, 1, M_PI / 180, HOUGH_THRESHOLD, MIN_LINE_LENGTH, MAX_LINE_GAP);
    cv::Mat line_image = dst_image;
    std::vector<std::vector<cv::Vec4i>> lines(2);
    std::vector<cv::Point> centers;
    for(auto it=hough_lines.begin();it!=hough_lines.end();++it){
        cv::Vec4i l = *it;
        cv::line(hough_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
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
                // std::cout << "distance: " << distance << "[px]" << std::endl;
                if(MIN_DISTANCE_LIMIT < distance && distance < MAX_DISTANCE_LIMIT){
                    bool registered_flag = false;
                    for(auto line : lines[1]){
                        if(fabs(get_angle(line) - get_angle(l2)) < ANGLE_DIFF_THRESHOLD){
                            registered_flag = true;
                        }
                    }
                    if(registered_flag){
                        continue;
                    }
                    //std::cout << "distance: " << distance << "[px]" << std::endl;
                    //std::cout << "stop line" << std::endl;
                    //std::cout << l << std::endl;
                    //std::cout << get_length(l) << std::endl;
                    //std::cout << l2 << std::endl;
                    //std::cout << get_length(l2) << std::endl;
                    //cv::Scalar color(l[0] / (double)image.cols * 255, l[1] / (double)image.rows * 255, 0);
                    cv::Point center((l[0] + l[2] + l2[0] + l2[2]) / 4.0, (l[1] + l[3] + l2[1] + l2[3]) / 4.0);
                    //std::cout << center << std::endl;
                    //std::cout << (int)filtered_image.at<unsigned char>(center.y, center.x) << std::endl;
                    if(filtered_image.at<unsigned char>(center.y, center.x) == 0){
                        // not white line
                        break;
                    }
                    centers.push_back(center);
                    lines[0].push_back(l);
                    lines[1].push_back(l2);
                    //cv::line(line_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), color, 1, cv::LINE_AA);
                    //cv::line(line_image, cv::Point(l2[0], l2[1]), cv::Point(l2[2], l2[3]), color, 1, cv::LINE_AA);
                    //cv::putText(line_image, "line", center, cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
                    break;
                }
            }
        }
    }
    int n = lines[0].size();
	bool t_flag = false;
    double t_angle;
    std::vector<double> line_angles;
    std::vector<cv::Scalar> colors;
	for(int i=0;i<n;i++){
        double line_angle1 = get_angle(lines[0][i]);
        for(int j=i+1;j<n;j++){
            double line_angle2 = get_angle(lines[0][j]);
            if(fabs(line_angle1 - line_angle2) > M_PI*0.25 && fabs(line_angle1 - line_angle2) < M_PI*0.75){
                line_angles.clear();
                if(fabs(line_angle1) < M_PI*0.25 || fabs(line_angle1) > M_PI*0.75){
                    line_angles.push_back(line_angle1);
                }else{
                    line_angles.push_back(line_angle2);
                }
                t_flag = true;
                break;
            }
        }
        double direction_diff = robot_direction - edge_direction;
        double angle_diff = line_angle1 - M_PI*0.5 + direction_diff;
        angle_diff = fabs(atan2(sin(angle_diff), cos(angle_diff)));
        if(angle_diff > M_PI*0.25 && angle_diff < M_PI*0.75){
            /*horizontal*/
            cv::Scalar color = CV_RGB(0,0,255);//blue
            colors.push_back(color);
        }else{
            /*vertical*/
            cv::Scalar color = CV_RGB(107,255,0);//green
            colors.push_back(color);
        }
        line_angles.push_back(line_angle1);

    }
    //std::cout << "detected lines: " << n << std::endl;
    for(int i=0;i<n;i++){
        if(t_flag){
            cv::Scalar color = CV_RGB(255,0,0);//red
            cv::line(line_image, cv::Point(lines[0][i][0], lines[0][i][1]), cv::Point(lines[0][i][2], lines[0][i][3]), color, 1, cv::LINE_AA);
            cv::line(line_image, cv::Point(lines[1][i][0], lines[1][i][1]), cv::Point(lines[1][i][2], lines[1][i][3]), color, 1, cv::LINE_AA);
            cv::putText(line_image, "T_line", centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
        }else{
            // cv::Scalar color = CV_RGB(0,0,255);
            cv::line(line_image, cv::Point(lines[0][i][0], lines[0][i][1]), cv::Point(lines[0][i][2], lines[0][i][3]), colors[i], 1, cv::LINE_AA);
            cv::line(line_image, cv::Point(lines[1][i][0], lines[1][i][1]), cv::Point(lines[1][i][2], lines[1][i][3]), colors[i], 1, cv::LINE_AA);
            cv::putText(line_image, "line", centers[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 1, cv::LINE_AA);
        }
        //std::cout << "detected line " << i << ": " << std::endl;
        //std::cout << lines[0][i] << std::endl;
        //std::cout << "center: " << centers[i] << std::endl;
        if(LINE_POSITION_V_THRESHOLD < centers[i].y){
            // std::cout << "!!! line !!!" << std::endl;
            amsl_navigation_msgs::StopLine line_info;
            if(t_flag){
                std::cout << "\033[31m----- line ----- \033[m" << std::endl;
                line_info.is_t_shape = true;
                line_info.angle = line_angles[0];
            }else{
                std::cout << "\033[33m----- line ----- \033[m" << std::endl;
                line_info.is_t_shape = false;
                line_info.angle = line_angles[i];
            }
            if(fabs(line_angles[i])>0.25*M_PI){
                std::cout << "line trace" << std::endl;
                line_info.center_point = calc_center_point(centers[i]);
            }else{
                line_info.center_point.x = 0.0;
                line_info.center_point.y = 0.0;
            }
            std::cout << line_info << std::endl;
            line_pub.publish(line_info);
        }
    }
    cv::Mat result_image;
    cv::warpPerspective(line_image, result_image, homography_matrix.inv(), result_image.size());
    //std::cout << ros::Time::now().toSec() - start << "[s]" << std::endl;
    if(SHOW_IMAGE){
        // cv::namedWindow("transformed_image", cv::WINDOW_NORMAL);
        // cv::imshow("transformed_image", dst_image);
        //cv::namedWindow("hsv_image", cv::WINDOW_NORMAL);
        //cv::imshow("hsv_image", hsv_image);
        //cv::namedWindow("mask_image", cv::WINDOW_NORMAL);
        //cv::imshow("mask_image", mask_image);
        // cv::namedWindow("hough_image", cv::WINDOW_NORMAL);
        // cv::imshow("hough_image", hough_image);
        cv::namedWindow("filtered_image", cv::WINDOW_NORMAL);
        cv::imshow("filtered_image", filtered_image);
        // cv::namedWindow("contour_image", cv::WINDOW_NORMAL);
        // cv::imshow("contour_image", contour_image);
        //cv::namedWindow("canny_image", cv::WINDOW_NORMAL);
        //cv::imshow("canny_image", canny_image);
        cv::namedWindow("line_image", cv::WINDOW_NORMAL);
        cv::imshow("line_image", line_image);
        // cv::namedWindow("result_image", cv::WINDOW_NORMAL);
        // cv::imshow("result_image", result_image);
        cv::waitKey(1);
    }
}

geometry_msgs::Point StopLineDetector::calc_center_point(cv::Point center_point)
{
    geometry_msgs::Point point;
    double x = (1.0-(center_point.y)/640.0)*0.5+0.3;
    double y = -(center_point.x-320)/320.0*0.5;
    point.x = x;
    point.y = y;
    return point;
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
