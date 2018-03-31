//
// Created by vincentlv on 10-06-2017.
//

#ifndef CPC_SEGMENTER
#define CPC_SEGMENTER

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/cpc_segmentation.h>

#include "segmenter_params.h"

template <typename PointT>
class CpcSegmenter 
{
public:
	explicit CpcSegmenter(const CpcParams& params=CpcParams::default_config()) :
		params_(params), super_(params.supervoxel_params.voxel_resolution, params.supervoxel_params.seed_resolution, params.supervoxel_params.use_single_cam_transform)
	{
		update();
	}

	typedef typename pcl::PCLBase<PointT>::PointCloudConstPtr PointCloudConstPtr;
	typedef typename pcl::PCLBase<PointT>::PointCloudPtr PointCloudPtr;
	typedef typename pcl::PointCloud<pcl::Normal> NormalCloudT;
	typedef typename NormalCloudT::Ptr NormalCloudPtr;
	typedef typename NormalCloudT::ConstPtr NormalCloudConstPtr;

	void setInputCloud (const PointCloudConstPtr& cloud)
	{
		input_ = cloud;
		super_.setInputCloud (input_);
	}
  
	void setNormalCloud (const NormalCloudConstPtr& normal_cloud)
	{
		normal_ = normal_cloud;
		super_.setNormalCloud (normal_);
	}

	void setParams(const CpcParams& params)
	{
		params_ = params;
		update();
	}

	void segment()
	{
		PCL_INFO ("Starting Segmentation\n");

		// extract supervoxels
		super_.extract (supervoxel_clusters_);
		if (params_.supervoxel_params.use_supervoxel_refinement)
		{
			PCL_INFO ("Refining supervoxels\n");
			super_.refineSupervoxels (2, supervoxel_clusters_);
		}
		PCL_INFO ("Extracting adjacency\n");
		// extract adjacency graph
		super_.getSupervoxelAdjacency (supervoxel_adjacency_);

		PCL_INFO ("Setting input voxels\n");	
		// feed supervoxels and adjacency graph to cpc_ 
		cpc_.setInputSupervoxels (supervoxel_clusters_, supervoxel_adjacency_);

		PCL_INFO ("Segmenting\n");	
		// segment
		cpc_.segment ();

		PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
		sv_labeled_cloud_ = super_.getLabeledCloud ();
		cpc_labeled_cloud_ = sv_labeled_cloud_->makeShared ();

		cpc_.relabelCloud (*cpc_labeled_cloud_);

		reset();
	}

	pcl::PointCloud<pcl::PointXYZL>::Ptr getLabeledCloud() const
	{
		return cpc_labeled_cloud_;
	}

private:
	void reset()
	{
		// reset supervoxelclustering (otherwise seg fault!)
		const SupervoxelParams& sv_params = params_.supervoxel_params;
		pcl::SupervoxelClustering<PointT> super(sv_params.voxel_resolution, sv_params.seed_resolution, sv_params.use_single_cam_transform);
		super_ = super;
		updateSupervoxelParams();
	}

	void update()
	{
		updateSupervoxelParams();
		updateCpcParams();
	}

	void updateSupervoxelParams()
	{
		const SupervoxelParams& sv_params = params_.supervoxel_params;

		super_.setSeedResolution(sv_params.seed_resolution);
		super_.setVoxelResolution(sv_params.voxel_resolution);
		super_.setUseSingleCameraTransform (sv_params.use_single_cam_transform);

		super_.setColorImportance (sv_params.color_importance);
		super_.setSpatialImportance (sv_params.spatial_importance);
		super_.setNormalImportance (sv_params.normal_importance);
	}

	void updateCpcParams()
	{
		const SupervoxelParams& sv_params = params_.supervoxel_params;

		cpc_.setSmoothnessCheck (true, sv_params.voxel_resolution, sv_params.seed_resolution, params_.smoothness_threshold);

		cpc_.setConcavityToleranceThreshold (params_.concavity_tolerance_threshold);
		cpc_.setSanityCheck (params_.use_sanity_criterion);
		cpc_.setKFactor (params_.use_extended_convexity);
		cpc_.setMinSegmentSize (params_.min_segment_size);
		
		cpc_.setCutting (params_.max_cuts, params_.cutting_min_segments, params_.min_cut_score, params_.use_local_constrain, params_.use_directed_cutting, params_.use_clean_cutting);
		cpc_.setRANSACIterations (params_.ransac_iterations);

	}

	// supervoxels
	pcl::SupervoxelClustering<PointT> super_;

	std::map<uint32_t, typename pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters_;
	// extract adjacency graph
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency_;

	// cpc 
	pcl::CPCSegmentation<PointT> cpc_;

	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud_;
	pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud_;

	// input
	PointCloudConstPtr input_;
	NormalCloudConstPtr normal_;

	// params
	CpcParams params_;
};


#endif
