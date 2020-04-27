#include "myslam/reconstruction.h"

#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
namespace simpleslam{

    Reconstruction::Reconstruction(Mapping::PointCloud::Ptr cloud)
    {
        cloudxyz_= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        cloudxyz_->points.resize(cloud->size());
        for (size_t i = 0; i < cloudxyz_->points.size(); i++)        {
            cloudxyz_->points[i].x = cloud->points[i].x;
            cloudxyz_->points[i].y = cloud->points[i].y;
            cloudxyz_->points[i].z = cloud->points[i].z;
        }
        cloud_with_normals_ = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    }
    pcl::PolygonMesh Reconstruction::GetMesh()
    {
        return mesh_;
    }
    void Reconstruction::NormalEstimation(int k_search_num)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>(false));
        tree1->setInputCloud(cloudxyz_);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloudxyz_);
        ne.setSearchMethod(tree1);
        ne.setKSearch(k_search_num);

        Eigen::Vector4f view_pt;
        if (use_camera_pos_){
            view_pt[0] = pos_[0]; view_pt[1] = pos_[1];
            view_pt[2] = pos_[2]; view_pt[3] = 1.;
        }
        else{
            pcl::compute3DCentroid(*(cloudxyz_), view_pt); 
        }
        //ne.setViewPoint(view_pt[0], view_pt[1], view_pt[2]);
        //TO-DO: Set to camera pos instead when in realtime?
        ne.setViewPoint(0., 0., 0.);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*cloud_normals);
        pcl::concatenateFields(*(cloudxyz_), *cloud_normals, *cloud_with_normals_);
}
    void Reconstruction::MovingLeastSquares(int polynomial_order, double search_rad)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>(false));
        tree1->setInputCloud(cloudxyz_);
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> ne;
        ne.setInputCloud(cloudxyz_);
        ne.setSearchMethod(tree1);
        ne.setPolynomialOrder(polynomial_order);
        ne.setSearchRadius(search_rad);

        ne.process(*cloud_with_normals_);
}

    void Reconstruction::Poisson(int depth)
    {
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(depth);
        poisson.setSearchMethod(tree);
        poisson.setInputCloud(cloud_with_normals_);
        poisson.reconstruct(mesh_);
    }

    void Reconstruction::MarchingCubes(std::string option, double iso_level, int resolution, double percent_extend)
    {
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        pcl::MarchingCubes<pcl::PointNormal> *mc;
        if (option == "hoppe") {
            mc =  new pcl::MarchingCubesHoppe<pcl::PointNormal> (); 
        }
        else{
            mc = new pcl::MarchingCubesRBF<pcl::PointNormal> ();  
        }
        mc->setIsoLevel(iso_level);
        mc->setGridResolution(resolution, resolution,resolution);
        mc->setPercentageExtendGrid(percent_extend);
        mc->setInputCloud(cloud_with_normals_); 
        mc->setSearchMethod(tree); 
        mc->reconstruct(mesh_);   
    }

    void Reconstruction::FilterLargeEdgeLength(double tol)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(mesh_.cloud, *cloud1);
        std::vector<int> indices;
        std::vector<pcl::Vertices> polygons;

        std::vector<pcl::Vertices, std::allocator<pcl::Vertices>>::iterator it;
        for(it = mesh_.polygons.begin(); it != mesh_.polygons.end(); ++it)
        {
                
                Eigen::Vector3d e1(cloud1->points[it->vertices[0]].x, 
                        cloud1->points[it->vertices[0]].y,
                        cloud1->points[it->vertices[0]].z);
                Eigen::Vector3d e2(cloud1->points[it->vertices[1]].x,
                        cloud1->points[it->vertices[1]].y,
                        cloud1->points[it->vertices[1]].z);
                Eigen::Vector3d e3(cloud1->points[it->vertices[2]].x,
                        cloud1->points[it->vertices[2]].y,
                        cloud1->points[it->vertices[2]].z);
                double l1 = (e1-e2).norm(); double l2 = (e2-e3).norm(); double l3 = (e3-e1).norm();
                if(l1<tol && l2<tol && l3<tol)
                {
                        polygons.push_back(*it);
                }
        }
        // Dump vertices
        mesh_.polygons.clear();
        mesh_.polygons.insert(mesh_.polygons.begin(), polygons.begin(), polygons.end());
        pcl::PolygonMesh cleanedMesh(mesh_);
        pcl::surface::SimplificationRemoveUnusedVertices cleaner;
        cleaner.simplify(cleanedMesh,mesh_);
}

}
