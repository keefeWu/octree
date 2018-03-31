//
// Created by vincentlv on 17-3-20.
//

#ifndef VISION_CV_UTILS_H
#define VISION_CV_UTILS_H

#include <opencv2/opencv.hpp>
#include <math.h>

#ifndef PI
#define PI M_PI
#endif

#define DEG2RAD(deg) deg * PI / 180
#define RAD2DEG(rad) rad * 180 / PI

template <typename T>
inline void randomize_vector(T& v){
    for (int i = 0; i < v.rows; ++i) {
        v[i] = rand() % 255 + 1;
    }
}

template <typename PointT>
struct PointYSort {
    bool operator() (PointT pt1, PointT pt2) { return (pt1.y < pt2.y);}
};
template <typename PointT>
struct PointXSort {
    bool operator() (PointT pt1, PointT pt2) { return (pt1.x < pt2.x);}
};

template <typename PointT>
inline void sort_points_by_y(std::vector<PointT>& pts)
{
    std::sort(pts.begin(), pts.end(), PointYSort<PointT>());
}
template <typename PointT>
inline void sort_points_by_x(std::vector<PointT>& pts)
{
    std::sort(pts.begin(), pts.end(), PointXSort<PointT>());
}

//bool isGrayImage( const cv::Mat& img ) // returns true if the given 3 channel image is B = G = R
//{
////    cv::Mat dst;
////    cv::Mat bgr[3];
////    cv::split( img, bgr );
////    cv::absdiff( bgr[0], bgr[1], dst );
////
////    if(cv::countNonZero( dst ))
////        return false;
////
////    cv::absdiff( bgr[0], bgr[2], dst );
////    return !cv::countNonZero( dst );
//    return img.channels() == 1;
//}

// usage: std::sort(contours.begin(), contours.end(), compareContourAreas);
// where contours is std::vector<std::vector<Point>>
inline bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}


// taken from opencv 
// https://github.com/kipr/opencv/blob/master/samples/cpp/bagofwords_classification.cpp
inline float intersection_over_union(const cv::Rect& detection, const cv::Rect& ground_truth)
{
    int detection_x2 = detection.x + detection.width;
    int detection_y2 = detection.y + detection.height;
    int ground_truth_x2 = ground_truth.x + ground_truth.width;
    int ground_truth_y2 = ground_truth.y + ground_truth.height;
    //first calculate the boundaries of the intersection of the rectangles
    int intersection_x = std::max(detection.x, ground_truth.x); //rightmost left
    int intersection_y = std::max(detection.y, ground_truth.y); //bottommost top
    int intersection_x2 = std::min(detection_x2, ground_truth_x2); //leftmost right
    int intersection_y2 = std::min(detection_y2, ground_truth_y2); //topmost bottom
    //then calculate the width and height of the intersection rect
    int intersection_width = intersection_x2 - intersection_x + 1;
    int intersection_height = intersection_y2 - intersection_y + 1;
    //if there is no overlap then return false straight away
    if ((intersection_width <= 0) || (intersection_height <= 0)) return -1.0;
    //otherwise calculate the intersection
    int intersection_area = intersection_width*intersection_height;

    //now calculate the union
    int union_area = (detection.width+1)*(detection.height+1) + (ground_truth.width+1)*(ground_truth.height+1) - intersection_area;

    //calculate the intersection over union and use as threshold as per VOC documentation
    float overlap = static_cast<float>(intersection_area)/static_cast<float>(union_area);
    return overlap;
}


#endif //VISION_CV_UTILS_H
