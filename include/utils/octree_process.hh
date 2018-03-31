#ifndef OCTREE_PROCESS_HH
#define OCTREE_PROCESS_HH

#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>
#include <vector>
#include <ctime>

// pcl io
#include <pcl/io/pcd_io.h>
// command parse
#include <pcl/console/parse.h>

// pcl vis
#include <pcl/visualization/cloud_viewer.h>

// passthrough
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>

#include "fileutils.hpp"
#include "stringutils.hpp"

using namespace std;
using namespace futils;
using namespace strutils;


typedef pcl::PointXYZRGB PointT;
#ifndef PRINT
#define PRINT(a) cout << #a << ": " << a << endl;
#endif

struct Point
{
    float x,y,z;
};
class Voxel
{
    int min_points_per_voxel_=20;
public:
    //vector<vector<vector<VoxelPoint>>> points_;
    int point_num;
    Point direction;
    bool has_direction;
    bool is_occupied;
    Point next_voxel;
    Point position;
    Voxel(Point direction);
    void reset();
    void calculate_direction();
    void set_occupied(); 
};
class Octree
{
public:
    float g_resolution;
    float g_minX,g_maxX,g_minY,g_maxY,g_minZ,g_maxZ;
    void updateCloud(pcl::PointCloud<PointT>::Ptr cloud_xyzrgb, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz);
    Point calculate_normal(pcl::PointCloud<PointT>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr compare_voxel(vector<vector<vector<Voxel>>>voxel1,
                                                    vector<vector<vector<Voxel>>>voxel2,
                                                    pcl::PointCloud<PointT>::Ptr cloud_xyzrgb);
    void calculate_octree(vector<vector<vector<Voxel>>>&voxel,pcl::PointCloud<PointT>::Ptr cloud);
};



#endif
