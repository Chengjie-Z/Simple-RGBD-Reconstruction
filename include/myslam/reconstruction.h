#ifndef SIMPLESLAM_RECONSTRUCTION_H
#define SIMPLESLAM_RECONSTRUCTION_H

#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h>
#include <pcl/io/vtk_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>

#include "myslam/mapping.h"

namespace simpleslam{


class Reconstruction {
    public:
    typedef std::shared_ptr<Reconstruction> Ptr;

    Reconstruction (Mapping::PointCloud::Ptr cloud);
    ~Reconstruction (){};

    void NormalEstimation(int k_search_num=100);

    void MovingLeastSquares(int polynomial_order=2, double search_rad = 0.03);

    pcl::PolygonMesh Poisson( int depth);

    pcl::PolygonMesh MarchingCubes(std::string option, double iso_level, int resolution, double percent_extend);

    private:
   
    bool use_camera_pos_ = false; 
    Vec3 pos_ = Vec3::Zero(); //position in world
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;
    //pcl::search::KdTree<pcl::PointNormal>::Ptr tree_;




};

}

#endif  // MYSLAM_RECONSTRUCTION_H
