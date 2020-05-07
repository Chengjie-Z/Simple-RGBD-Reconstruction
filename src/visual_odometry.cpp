//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include "myslam/config.h"
#include <boost/format.hpp>

namespace simpleslam {

inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); } //simple helper function

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    io_ =IO::Ptr(new IO(Config::Get<std::string>("dataset_dir")));
    io_->SetRealtime(realtime_);
    //CHECK_EQ(io_->Init(), true);
    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);
    mapping_ = Mapping::Ptr(new Mapping(Config::Get<double>("voxel_length"), Config::Get<double>("sdf_trunc")));
    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCamera(io_->GetCamera(0));
    backend_->SetMap(map_);
    backend_->SetCamera(io_->GetCamera(0));
    viewer_->SetMap(map_);
    std::cout << "TSDF, voxel_length: " << Config::Get<double>("voxel_length") << std::endl; 
    std::cout << "TSDF, sdf_trunc: " << Config::Get<double>("sdf_trunc") << std::endl; 
    return true;
}

void VisualOdometry::Run() {

    while (1) {
        //LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }
    backend_->Stop();
    viewer_->Close();

    //LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    auto t0 = std::chrono::steady_clock::now();
    Frame::Ptr new_frame = io_->NextFrame();
    auto t1 = std::chrono::steady_clock::now();
    if (new_frame == nullptr) return false;
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();

    if(save_pose_)
        io_->SavePose(new_frame);

    if(build_map_)
    {
        auto t3 = std::chrono::steady_clock::now();
        mapping_->merge_with(new_frame,io_->GetCamera(0));
        //mapping_->pcd_viewer->showCloud(mapping_->dense_map);
        auto t4 = std::chrono::steady_clock::now();
        timer2  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3).count();
        auto t5 = std::chrono::steady_clock::now();
        auto mesh = mapping_->dense_map->ExtractTriangleMesh();
        auto t6 = std::chrono::steady_clock::now();
        timer3 += std::chrono::duration_cast<std::chrono::duration<double>>(t6 - t5).count();
        if (io_->GetIndex()%2==0)
            io_->SaveMesh(mapping_);
    }
    auto t7 = std::chrono::steady_clock::now();

    timer0  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    timer1  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    timer4  +=  std::chrono::duration_cast<std::chrono::duration<double>>(t7 - t1).count();

    if(io_->GetIndex() % 30 == 0)  //every 50 frames
    {
        std::cout << "IO cost time: " << timer0/30 << " seconds." << std::endl;
        std::cout << "VO cost time: " << timer1/30 << " seconds."<< std::endl;
        if(save_point_cloud_||build_map_)
            std::cout << "Mapping cost time: " << timer2/30 << " seconds."<<std::endl;
        std::cout << "MC cost time: " << timer3/30 << " seconds." << std::endl;
        std::cout << "Time per frame: " << timer4/30 << " seconds." << std::endl;
        std:: cout << "Frame rate: " << 30/timer4 << std::endl;
        timer0=timer1=timer2=timer3=0;
    }
    return success;
}


}  // namespace myslam
