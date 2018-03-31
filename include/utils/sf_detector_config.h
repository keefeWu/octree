//
// Created by vincentlv on 17-3-15.
//

#ifndef VISION_SF_DETECTOR_CONFIG_H
#define VISION_SF_DETECTOR_CONFIG_H

#include <string>
#include <unordered_map>


#include "json.hpp"


struct SfDetectorObjectConfig {

    // functions
    static SfDetectorObjectConfig default_config();
    static SfDetectorObjectConfig load_config_from_json(nlohmann::json& json_obj_conf);

    // variables
    std::string image_path_;

    int target_resize_width_;
    int target_resize_height_;

    int gaussian_blur_kernel_size_;   // for blurring the template image
    int target_gaussian_blur_kernel_size_;   // for blurring the target image

    /* layer 1 */
    int layer1_matcher_nearest_neighbors_;  // 3
    float layer1_features_distance_ratio_;  // 0.75
    float layer1_keypoints_cluster_distance_;  // 9 (numbers), 23 (sf)
    int layer1_keypoints_cluster_min_size_;  // 3
    int layer1_keypoints_cluster_eps_;  // 20
    bool layer1_keypoints_use_kmeans_;
    int layer1_keypoints_min_kmeans_centroid_distance_;

    // padding for each side
    int layer1_bounding_box_padding_;  // 2

    /* layer 2 */
    bool layer2_use_;
    // preprocessing
    // int layer2_gaussian_blur_size_;  // 3
    // int layer2_adaptive_threshold_block_size_;  // 11
    // float layer2_adaptive_threshold_constant_;  // 7

    int layer2_matcher_nearest_neighbors_;  // 3
    float layer2_features_distance_ratio_;  // 0.7

    int layer2_min_good_matches_;  // 1 (numbers), 4 (sf)
};

struct SfDetectorConfig {
    static SfDetectorConfig default_config();
    // the keys in the json file must exactly match the variable names in this ServerConfig struct
    static SfDetectorConfig load_config_from_json(const std::string json_file);

    std::unordered_map<std::string, SfDetectorObjectConfig> sf_object_config_map_;

};


#endif //VISION_SF_DETECTOR_CONFIG_H
