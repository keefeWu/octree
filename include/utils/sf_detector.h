//
// Created by vincentlv on 17-3-15.
//

#ifndef VISION_SF_DETECTOR_H
#define VISION_SF_DETECTOR_H


#include <opencv2/opencv.hpp>

// features
#if CV_MAJOR_VERSION >= 3
#include <opencv2/xfeatures2d.hpp>
#else CV_MAJOR_VERSION >= 3
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#define MIN_HESSIAN 3000  // 3000+ gives similar result as opencv3 default SIFT 
#endif CV_MAJOR_VERSION >= 3
//

#include "sf_detector_config.h"
#include "DbScan.h"


class SfDetector
{

public:
    // constructors
    explicit SfDetector();
    explicit SfDetector(const std::string sf_detector_config_path);
    explicit SfDetector(const SfDetectorConfig& sf_detector_config);
    // destructors
    ~SfDetector();

    // config getter/setter
    void set_config(const std::string sf_detector_config_path);
    void set_config(const SfDetectorConfig& sf_detector_config);
    SfDetectorConfig get_config();

    void clear();

    bool key_exists(const std::string key) const;

    // print params
    void print_object_params(std::string object_key) const;

    // public detect functions
    bool detect_object(const cv::Mat& img, std::string object_key, std::string& msg);

    void resize_data(const float resize_width_ratio, const float resize_height_ratio);

    // expose data
    std::vector<std::vector<cv::Point2f>> cluster_;
    std::vector<cv::Point2f> cluster_means_;
    std::vector<cv::Rect> cluster_bounding_boxes_;
    // std::vector<int> min_points_cluster_indices_; // index of clusters with min cluster points
    // std::vector<cv::Rect> min_points_cluster_bounding_boxes_; 
    std::vector<int> good_cluster_indices_; // index of good clusters boxes
    std::vector<cv::Rect> good_cluster_bounding_boxes_;

private:
    // init function
    void init();

    // helper functions
    void load_object_image_map();

    void cluster_keypoints(const std::vector<cv::Point2f>& keypoints, const cv::Mat& img);

    // member variables
    cv::Ptr<cv::Feature2D> sift_feature_detector_;
    cv::BFMatcher bf_feature_matcher_;
//    cv::FlannBasedMatcher flann_feature_matcher;

    // server configuration
    SfDetectorConfig sf_detector_config_;

    // hashmap for storing template images
    std::unordered_map<std::string, cv::Mat> sf_object_image_map_;

    // used for keypoint clustering
    DbScan<cv::Point2f> db_scan_;

    std::string current_query_key_ = "";

};

#endif //VISION_SF_DETECTOR_H
