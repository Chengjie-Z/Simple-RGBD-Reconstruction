//
// Created by jerry on 4/18/20.
//

#ifndef SIMPLESLAM_MAPPING_H
#define SIMPLESLAM_MAPPING_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/camera.h"
#include "myslam/conversion.h"
#include "Open3D/Integration/ScalableTSDFVolume.h"
#include "Open3D/Visualization/Utility/DrawGeometry.h"
#include "Open3D/IO/ClassIO/TriangleMeshIO.h"
#include "Open3D/IO/ClassIO/ImageIO.h"


namespace simpleslam{


class Mapping{
public:
    typedef std::shared_ptr<Mapping> Ptr;
    typedef open3d::integration::ScalableTSDFVolume TSDF;
    typedef std::shared_ptr<TSDF> Map_Ptr;

    Map_Ptr dense_map;

    Mapping(double voxel_length=0.04, double sdf_trunc=0.3);
    bool merge_with(Frame::Ptr frame, Camera::Ptr camera);
    double voxel_length_;
    double sdf_trunc_;
};





};

#endif //SIMPLESLAM_MAPPING_H
