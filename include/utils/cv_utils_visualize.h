//
// Created by vincentlv on 17-3-16.
//

#ifndef VISION_CV_UTILS_VISUALIZE_H
#define VISION_CV_UTILS_VISUALIZE_H

#include <opencv2/opencv.hpp>

inline void visualize_boxes(cv::Mat& img_mat, const std::vector<cv::Rect>& boxes, const cv::Scalar& color = cv::Scalar(0, 0, 255), bool wait=false, std::string window_name="Boxes")
{
    for (int i = 0; i < boxes.size(); ++i) {
        // const cv::Rect &rect_ = boxes[i];

        // cv::Point2f kp_box_top_left(rect_.x, rect_.y);
        // cv::Point2f kp_box_bottom_right(rect_.x + rect_.width,
        //                               rect_.y + rect_.height);
        cv::rectangle(img_mat, boxes[i], color);
    }
    cv::imshow(window_name, img_mat);
    if (wait)
        cv::waitKey(0);
}

template <typename T>
inline void visualize_points(cv::Mat& img_mat, const std::vector<T>& pts, float radius = 2.0f, const cv::Scalar& color = cv::Scalar(0, 0, 255), bool wait=false, std::string window_name="Points")
{
    for (int i = 0; i < pts.size(); ++i) {
        cv::circle(img_mat, pts[i], radius, color);
    }
    cv::imshow(window_name, img_mat);

    if (wait)
        cv::waitKey(0);
}

#endif //VISION_CV_UTILS_VISUALIZE_H
