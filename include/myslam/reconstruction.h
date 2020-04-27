#ifndef SIMPLESLAM_RECONSTRUCTION_H
#define SIMPLESLAM_RECONSTRUCTION_H

#include <iostream>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "myslam/mapping.h"

namespace simpleslam{


class Reconstruction {
    public:
    typedef std::shared_ptr<Reconstruction> Ptr;

    Reconstruction (Mapping::PointCloud::Ptr cloud);
    ~Reconstruction (){};

    void NormalEstimation(int k_search_num=100);

    void MovingLeastSquares(int polynomial_order=2, double search_rad = 0.03);

    void Poisson( int depth);

    void MarchingCubes(std::string option, double iso_level, int resolution, double percent_extend);

    void FilterLargeEdgeLength(double tol);

    pcl::PolygonMesh GetMesh();
    private:
   
    bool use_camera_pos_ = false; 
    Vec3 pos_ = Vec3::Zero(); //position in world
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz_;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;
    pcl::PolygonMesh mesh_;
    //pcl::search::KdTree<pcl::PointNormal>::Ptr tree_;




};

}

#endif  // MYSLAM_RECONSTRUCTION_H
