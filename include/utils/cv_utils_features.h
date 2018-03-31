
#ifndef VISION_CV_UTILS_FEATURES_H
#define VISION_CV_UTILS_FEATURES_H

#include <opencv2/opencv.hpp>

inline void extract_good_feature_matches(std::vector<cv::DMatch> &good_matches,
                                  const std::vector<std::vector<cv::DMatch> > &matches,
                                  const float distance_ratio) {
    good_matches.clear();

    int matches_sz = matches.size();
    if (matches_sz == 0) {
        return;
    }

    for (int i = 0; i < matches_sz; ++i) {
        if (matches[i][0].distance < distance_ratio * matches[i][1].distance) {
            good_matches.push_back(matches[i][0]);
        }
    }
}

#endif //VISION_CV_UTILS_FEATURES_H
