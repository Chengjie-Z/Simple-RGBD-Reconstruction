#include "myslam/reconstruction.h"

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
        ne.setViewPoint(view_pt[0], view_pt[1], view_pt[2]);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*cloud_normals);
        pcl::concatenateFields(*(cloudxyz_), *cloud_normals, *cloud_with_normals_);
}
    void Reconstruction::NormalEstimation(int polynomial_order, double search_rad)
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

    pcl::PolygonMesh Reconstruction::Poisson(int depth)
    {
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        pcl::PolygonMesh mesh;
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth(depth);
        poisson.setSearchMethod(tree);
        poisson.setInputCloud(cloud_with_normals_);
        poisson.reconstruct(mesh);
        return mesh;
    }

    pcl::PolygonMesh Reconstruction::MarchingCubes(std::string option, double iso_level, int resolution, double percent_extend)
    {
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
        pcl::PolygonMesh mesh;
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
        mc->reconstruct(mesh);   
    }


}
