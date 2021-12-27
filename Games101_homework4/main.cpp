#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
            << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f>& points, cv::Mat& window)
{
    auto& p_0 = points[0];
    auto& p_1 = points[1];
    auto& p_2 = points[2];
    auto& p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
            3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2) {
        auto p = control_points[0] * (1 - t) + control_points[1] * t;
        return cv::Point2f(p);
    }
    std::vector<cv::Point2f> next_control_points;
    for (int i = 1; i < control_points.size(); i++) {
        cv::Point2f p1{ control_points[i - 1] };
        cv::Point2f p2{ control_points[i] };

        cv::Point2f ncp = p1 * (1 - t) + p2 * t;
        next_control_points.push_back(ncp);
    }
    return recursive_bezier(next_control_points, t);
}

//void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
//{
//    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
//    // recursive Bezier algorithm.
//    for (double t = 0.0; t <= 1.0; t += 0.001)
//    {
//        auto point = recursive_bezier(control_points, t);
//        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
//    }
//}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    // 2x2 ·´×ßÑù
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);

        int p0_x = std::floor(point.x);
        int p0_y = std::floor(point.y);
        int p1_x = p0_x + 1;
        int p1_y = p0_y;
        int p2_x = p0_x;
        int p2_y = p0_y + 1;
        int p3_x = p0_x + 1;
        int p3_y = p0_y + 1;

        float max_d = sqrt(8);
        float d0 = max_d - sqrt(pow(point.x - (p0_x + 0.5), 2) + pow(point.y - (p0_y + 0.5), 2));
        float d1 = max_d - sqrt(pow(point.x - (p1_x + 0.5), 2) + pow(point.y - (p1_y + 0.5), 2));
        float d2 = max_d - sqrt(pow(point.x - (p2_x + 0.5), 2) + pow(point.y - (p2_y + 0.5), 2));
        float d3 = max_d - sqrt(pow(point.x - (p3_x + 0.5), 2) + pow(point.y - (p3_y + 0.5), 2));
        float d_sum = d0 + d1 + d2 + d3;

        float k0 = d0 / d_sum;
        float k1 = d1 / d_sum;
        float k2 = d2 / d_sum;
        float k3 = d3 / d_sum;

        window.at<cv::Vec3b>(p0_y, p0_x)[1] = std::min(255.0f, window.at<cv::Vec3b>(p0_y, p0_x)[1] + k0 * 255);
        window.at<cv::Vec3b>(p1_y, p1_x)[1] = std::min(255.0f, window.at<cv::Vec3b>(p1_y, p1_x)[1] + k1 * 255);
        window.at<cv::Vec3b>(p2_y, p2_x)[1] = std::min(255.0f, window.at<cv::Vec3b>(p2_y, p2_x)[1] + k2 * 255);
        window.at<cv::Vec3b>(p3_y, p3_x)[1] = std::min(255.0f, window.at<cv::Vec3b>(p3_y, p3_x)[1] + k3 * 255);
    }
}


int main()
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    //cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);


    std::vector<cv::Point2f> control_points{ {100, 455}, {182, 315}, {390, 257}, {549, 393} };

    int key = -1;
    while (key != 27)
    {
        for (auto& point : control_points)
        {
            cv::circle(window, point, 3, { 255, 255, 255 }, 3);
        }

        if (control_points.size() == 4)
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
