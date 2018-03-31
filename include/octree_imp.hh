#ifndef OCTREE_IMP_HH
#define OCTREE_IMP_HH

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


typedef pcl::PointXYZRGBA PointT;
#ifndef PRINT
#define PRINT(a) cout << #a << ": " << a << endl;
#endif

class Octree
{
public:
    Octree(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float length);
    bool set_bbox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, float length);
    bool set_points(pcl::PointCloud<PointT>::Ptr cloud);
private:
    float x_min_, float x_max_, float y_min_, float y_max_, float z_min_, float z_max_;
    float length_;
    std::vector<Voxel> voxel_vector_;

    bool create_octree();
};

class Voxel
{
public:
    Voxel Voxel(float x, float y, float z, length);
    std::vector<PointT> points;
}

#endif
