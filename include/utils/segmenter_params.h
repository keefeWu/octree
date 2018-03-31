//
// Created by vincentlv on 10-06-2017.
//

#ifndef SEGMENTER_PARAMS
#define SEGMENTER_PARAMS

#define HAVE_JSON

#ifdef HAVE_JSON
#include <json.hpp>
#endif HAVE_JSON

struct SupervoxelParams
{
	// Supervoxel Stuff
	float voxel_resolution = 0.004f;
	float seed_resolution = 0.03f;
	float color_importance = 0.0f;
	float spatial_importance = 1.0f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

    static SupervoxelParams default_config();
#ifdef HAVE_JSON
    static SupervoxelParams load_config_from_json(nlohmann::json& json_obj_conf);
#endif HAVE_JSON

};

struct SupervoxelSegmenterParams
{
	// constructor
	SupervoxelSegmenterParams();
	explicit SupervoxelSegmenterParams(const SupervoxelParams& params);

	SupervoxelParams supervoxel_params;

	// LCCPSegmentation Stuff
	bool use_extended_convexity = true;
	bool use_sanity_criterion = false;
	float concavity_tolerance_threshold = 5;
	float smoothness_threshold = 0.1;
	unsigned int min_segment_size = 10;

	// functions
    static SupervoxelSegmenterParams default_config();
#ifdef HAVE_JSON
    static SupervoxelSegmenterParams load_config_from_json(nlohmann::json& json_obj_conf);
#endif HAVE_JSON
};

struct LccpParams: public SupervoxelSegmenterParams
{
	// constructor
	LccpParams();
	explicit LccpParams(const SupervoxelParams& params);
};

struct CpcParams: public SupervoxelSegmenterParams
{
	// constructor
	CpcParams();
	explicit CpcParams(const SupervoxelParams& params);
	explicit CpcParams(const SupervoxelSegmenterParams& params);


	// CPCSegmentation Stuff
	float min_cut_score = 0.16;   // -cut min_cut_score,max_cuts,cutting_min_segments
	unsigned int max_cuts = 3;
	unsigned int cutting_min_segments = 400;
	unsigned int ransac_iterations = 10000;

	bool use_local_constrain = false;
	bool use_directed_cutting = false;
	bool use_clean_cutting = false;

    // functions
    static CpcParams default_config();
#ifdef HAVE_JSON
    static CpcParams load_config_from_json(nlohmann::json& json_obj_conf);
    static CpcParams load_config_from_json(const std::string json_file);
#endif HAVE_JSON
};


#endif
