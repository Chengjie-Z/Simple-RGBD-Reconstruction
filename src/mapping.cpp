//
// Created by jerry on 4/18/20.
//

#include "myslam/mapping.h"

namespace simpleslam{
    Mapping::Mapping(double voxel_length, double sdf_trunc)
    {
        voxel_length_ = voxel_length;
        sdf_trunc_ = sdf_trunc;
        dense_map = Map_Ptr(new TSDF(voxel_length_ , sdf_trunc_, open3d::integration::TSDFVolumeColorType::RGB8));
    }

    bool Mapping::merge_with(Frame::Ptr frame,Camera::Ptr camera) {
        auto depth_ptr = std::make_shared<cv::Mat>(frame->depth_);
        auto color_ptr = std::make_shared<cv::Mat>(frame->color_);
        int height = depth_ptr->rows;
        int width = depth_ptr->cols;
        dense_map->Integrate(*to_o3d_RGBDImage(frame),*to_o3d_intrinsic(camera),frame->Pose().matrix());

        //LOG(INFO)<<"point cloud size:"<<dense_map->
    }



}
