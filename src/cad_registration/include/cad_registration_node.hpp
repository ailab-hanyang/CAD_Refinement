/****************************************************************************/
// Module:      cad_registration_node.hpp
// Description: detection fusion node
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com), Soyeong Kim (soyeongkim@hanyang.ac.kr)
// Version: 0.1
//
// Revision History
//      April 2, 2024: Jaeyoung Jo - Created.
//      Jun 18, 2025: Jaehwan Lee - Add Point2Point registration and optimization.
//      Jan 5, 2026: Soyeong Kim - Remove ailab dependencies and clean up code
//                               - Switch input to jsk_recognition_msgs
/****************************************************************************/

#ifndef __CAD_REGISTRATION_HPP__
#define __CAD_REGISTRATION_HPP__
#pragma once

// STD header
#include <memory>
#include <mutex>
#include <utility>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <omp.h>
#include <unordered_map>
#include <fstream>
#include <sys/stat.h>

// ROS header
#include <ros/ros.h>

// PCL
#include <pcl/common/eigen.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <pcl_ros/point_cloud.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// Message header
#include <hyu_msgs/PointsObjects.h>
#include <hyu_msgs/PointsObject.h>
#include <hyu_msgs/DetectObjects.h>
#include <hyu_msgs/DetectObject.h>
#include <hyu_msgs/ObjectState.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Point.h>

// Utility
#include "rosbridge_objects.hpp"

// Algorithm header
#include "cad_registration_config.hpp"

typedef struct {
    double timestamp; 
    pcl::PointCloud<pcl::PointXYZI>::Ptr points;
    std::string frame_id;
} LidarDataStruct;

// Namespace
using namespace ros;
using namespace tf;
using namespace std;
using namespace hyu_types;

class CadRegistration {
    public:
        // Constructor
        CadRegistration();
        // Destructor
        ~CadRegistration();

        void InitSurfelModel();

        bool SyncLidarDetection(DetectObjects& i_objects, 
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& o_synced_point_cloud);
        bool ExtractPointsFromDetection(DetectObjects& i_objects, 
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& i_synced_point_cloud,
                                    PointsObjects& o_points_objects);

        void RegistrationRefinement(PointsObjects& point_objects);

        bool CheckBehind(ObjectState i_object_state);
        void AngleBasedTime(ObjectState& i_object_state);
        
        // Registration Util Functions
        bool RunRegistration(PointsObject& p_object);
        void SurfOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points);
        void SurfelOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points);
        void Point2PointOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points);
        void UpdatePointAssociateToMap();
        Eigen::Affine3f Trans2Affine3f(float transformIn[]);
        void PointAssociateToMap(pcl::PointXYZ const * const pi, pcl::PointXYZ * const po);
        void PointAssociateToMapSurfel(pcl::PointXYZ const * const pi, pcl::PointSurfel * const po);
        bool LMOptimization4DoF(int iterCount);
        bool LMOptimization4DoFP2P(int iterCount);

        void GenerateHashMap();
        void HashSearch(const pcl::PointSurfel p_source, int& nearest_ind, float& nearest_distance);
        std::vector<int> GetSurroundVoxelIndices(int cur_voxel_index);
        int GetVoxelIndex(const float p_x, const float p_y, const float p_z);
        void VoxelFilter(const pcl::PointCloud<pcl::PointXYZI> i_cloud, pcl::PointCloud<pcl::PointXYZI>& o_cloud);

        Eigen::Quaterniond NormalToQuaternion(double pa, double pb, double pc);
        std::array<int, 3> ValueToRainbowColor(float value);

        // Output Functions
        void UpdateJSKLidarObjects(const PointsObjects& point_objects);
        void UpdatePointsObjectsPc(const PointsObjects& point_objects);
        
    public:
        void Init();
        void Run();
        void Publish();
        void Terminate();
        void ProcessINI();

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // Callback Functions
        void CallbackPillarObjects(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg);
        void CallbackMainPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);

    private:
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
        //
        mutex mutex_main_point_cloud_;
        mutex mutex_lidar_objects_;
        mutex mutex_pillar_objects_;

        // Configs

        // Input
        DetectObjects i_pillar_objects_;
        sensor_msgs::PointCloud2 i_main_point_cloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr i_main_point_cloud_ptr_;
        std::tuple<pcl::PointCloud<pcl::PointXYZI>::Ptr, string, ros::Time> i_main_lidar_raw_tuple_;
        LidarDataStruct i_lidar_struct_;

        // Output
        hyu_msgs::DetectObjects o_refined_objects_;
        sensor_msgs::PointCloud2 o_points_objects_pc_;
        sensor_msgs::PointCloud2 o_refined_cad_pc_;
        jsk_recognition_msgs::BoundingBoxArray o_jsk_lidar_objects_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr o_refined_cad_pcptr_;
        pcl::PointCloud<pcl::PointSurfel>::Ptr o_refined_cad_surfel_pcptr_;

        // Variables
        string v_main_lidar_frame_id_;
        bool b_is_new_pillar_object_ = false;
        bool b_is_new_point_cloud_ = false;
        bool b_is_new_publish_ = false;
        double d_latest_pillar_object_time_ = 0.0;

        // Registration
        pcl::PointCloud<pcl::PointXYZ>::Ptr ioniq_cad_pcptr_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr ioniq_cad_surfel_sample_pcptr_;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr ioniq_kdtree_;

        pcl::PointCloud<pcl::PointSurfel>::Ptr ioniq_cad_surfel_pcptr_;
        pcl::KdTreeFLANN<pcl::PointSurfel>::Ptr ioniq_surfel_kdtree_;

        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_cluster_;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_ioniq_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudOri_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr coeffSel_;

        Eigen::MatrixXf matP;
        
        Eigen::Affine3f f_arr_clacluster_to_init_tfuster_to_init_;
        Eigen::Affine3f pc_to_cad_tf;
        Eigen::Affine3f cluster_tf_;

        bool b_isDegenerate_ = false;

        float f_arr_init_to_cad_[6];

        visualization_msgs::MarkerArray o_surfel_marker_;

        std::deque<LidarDataStruct> deq_lidar_struct_;

        ros::Subscriber s_pillar_objects_;
        ros::Subscriber s_main_point_cloud_;
        ros::Publisher p_points_objects_pc_;
        ros::Publisher p_refined_cad_pc_;
        ros::Publisher p_jsk_refined_box_;
        ros::Publisher p_surfels_;

        // Params
        CadRegistrationParams box_point_generator_params_;

        string cfg_pillar_objects_topic_ = "app/perc/jsk_pillar_objects";
        string cfg_lidar_points_topic_ = "velodyne_points";
        string cfg_ioniq_pcd_path_ = "/home/ailab/git/CAD_Refinement/pcd/ioniq5_low_above_outside.pcd";

        int cfg_max_iteration = 10;

        // Time Compensation
        double d_optimization_ms_ = 0.0;
        double d_surf_ms_ = 0.0;
        double d_kd_tree_ms_ = 0.0;
        double d_plane_equation_ms_ = 0.0;
        double d_point_to_map_ms_ = 0.0;

        int i_optimization_time_num_ = 0;
        int i_surf_time_num_ = 0;
        int i_kd_tree_time_num_ = 0;
        int i_plane_equation_time_num_ = 0;
        int i_point_to_map_time_num_ = 0;


        // pcl voxel grid
        float f_voxel_grid_size_ioniq_m_ = 0.05;
        float f_voexl_grid_size_cluster_m_ = 0.2;
        float f_voxel_grid_size_surfel_sample_m_ = 0.3;

        // planarity
        float f_min_planarity = 0.95;

        // Hashmap
        float f_hash_grid_size_m_ = 0.4;

        float f_min_hash_x_m_ = -3.0;
        float f_max_hash_x_m_ = 3.0;
        float f_min_hash_y_m_ = -3.0;
        float f_max_hash_y_m_ = 3.0;
        float f_min_hash_z_m_ = -3.0;
        float f_max_hash_z_m_ = 3.0;  

        int i_hash_x_num_;
        int i_hash_y_num_;
        int i_hash_z_num_;
        int i_hash_total_num_;

        bool b_use_hash_search_;

        bool b_use_p2p_registration_ = false;

        bool b_visualize_cad_points_ = true;
        bool b_visualize_sample_cad_ = true;
        bool b_visualize_surfel_ = true;

        std::string str_calc_time_csv_file_;

        // For CSV logging
        std::vector<double> v_d_extract_points_time_ms_;
        double d_voxel_filter_time_ms_;
        double d_registration_loop_time_ms_;

        std::vector<std::vector<int>> vec_voxel_index_;
        // std::unordered_map<int, std::vector<int>> cluster_map;
        std::vector<bool> vec_cluster_exist;
        float f_lm_lambda_ = 0.5;
        bool b_use_gm_kernel_;
        float f_gm_kernel_simga_;

        //
        int i_direct_search_num_ = 0;
        int i_rel_search_num_ = 0;
        int i_not_searched_num_ = 0;

};

#endif  // __CAD_REGISTRATION_HPP__