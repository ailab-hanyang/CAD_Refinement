/****************************************************************************/
// Module:      cad_registration_node.cpp
// Description: detection fusion node
//
// Authors: Jaeyoung Jo (wodud3743@gmail.com), Soyeong Kim (soyeongkim@hanyang.ac.kr)
// Version: 0.1
//
// Revision History
//      Jul 24, 2023: Jaeyoung Jo - Created.
//      Jun 18, 2025: Jaehwan Lee - Add Point2Point registration and optimization.
//      Jan 5, 2026: Soyeong Kim - Remove ailab dependencies and clean up code (remove atom task)
//                               - Switch input to jsk_recognition_msgs
/****************************************************************************/

#include "cad_registration_node.hpp"

CadRegistration::CadRegistration()
{         
    i_main_point_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    o_refined_cad_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    o_refined_cad_surfel_pcptr_.reset(new pcl::PointCloud<pcl::PointSurfel>());

    ioniq_cad_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ioniq_cad_surfel_sample_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    ioniq_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    ioniq_cad_surfel_pcptr_.reset(new pcl::PointCloud<pcl::PointSurfel>());
    ioniq_surfel_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointSurfel>());

    voxel_grid_cluster_.setLeafSize(f_voexl_grid_size_cluster_m_, f_voexl_grid_size_cluster_m_, f_voexl_grid_size_cluster_m_);
    voxel_grid_ioniq_.setLeafSize(f_voxel_grid_size_ioniq_m_, f_voxel_grid_size_ioniq_m_, f_voxel_grid_size_ioniq_m_);

    laserCloudOri_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    coeffSel_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    matP.resize(4, 4);
    matP.setZero();

    Init();
}

CadRegistration::~CadRegistration() {}

void CadRegistration::Init(){
    ros::NodeHandle nh;

    nh.param<float>("/cad_registration/f_voxel_grid_size_ioniq_m_", f_voxel_grid_size_ioniq_m_, 0.05);
    nh.param<float>("/cad_registration/f_voexl_grid_size_cluster_m_", f_voexl_grid_size_cluster_m_, 0.2);
    nh.param<float>("/cad_registration/f_voxel_grid_size_surfel_sample_m_", f_voxel_grid_size_surfel_sample_m_, 0.3);
    nh.param<float>("/cad_registration/f_min_planarity", f_min_planarity, 0.95);
    nh.param<float>("/cad_registration/f_hash_grid_size_m_", f_hash_grid_size_m_, 0.4);
    nh.param<float>("/cad_registration/f_lm_lambda_", f_lm_lambda_, 0.5);
    
    nh.param<bool>("/cad_registration/b_use_gm_kernel_", b_use_gm_kernel_, false);
    nh.param<float>("/cad_registration/f_gm_kernel_simga_", f_gm_kernel_simga_, 0.01);
    nh.param<bool>("/cad_registration/b_use_hash_search_", b_use_hash_search_, true);
    
    nh.param<bool>("/cad_registration/b_visualize_cad_points_", b_visualize_cad_points_, true);
    nh.param<bool>("/cad_registration/b_visualize_sample_cad_", b_visualize_sample_cad_, true);
    nh.param<bool>("/cad_registration/b_visualize_surfel_", b_visualize_surfel_, true);
    nh.param<bool>("/cad_registration/b_use_p2p_registration_", b_use_p2p_registration_, false);

    nh.param<string>("/cad_registration/ioniq_pcd_path", cfg_ioniq_pcd_path_, "/home/ailab/git/CAD_Refinement/pcd/ioniq5_low_above_outside.pcd");

    s_main_point_cloud_ = nh.subscribe(cfg_lidar_points_topic_, 10, 
                                    &CadRegistration::CallbackMainPointCloud, this, ros::TransportHints().tcpNoDelay());                         
    s_pillar_objects_ = nh.subscribe(cfg_pillar_objects_topic_, 10, 
                                    &CadRegistration::CallbackPillarObjects, this, ros::TransportHints().tcpNoDelay());

    // publisher init
    p_points_objects_pc_ = nh.advertise<sensor_msgs::PointCloud2>(
            "app/perc/points_objects_pc", 10);
    p_refined_cad_pc_ = nh.advertise<sensor_msgs::PointCloud2>(
            "app/perc/refined_cad_pc_", 10);
    p_jsk_refined_box_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "app/perc/jsk_refined_objects", 10);
    p_surfels_ = nh.advertise<visualization_msgs::MarkerArray>("ego_surfel", 1);

    // Load PCD
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (cfg_ioniq_pcd_path_,*ioniq_cad_pcptr_) == -1){
        ROS_ERROR_STREAM("[Shape Registration] Cannot Read file: " << cfg_ioniq_pcd_path_);
        return;
    }
    else{
        ROS_WARN_STREAM("[Shape Registration] File Loaded: " << cfg_ioniq_pcd_path_);
    }

    InitSurfelModel();
}

void CadRegistration::InitSurfelModel()
{
    ROS_WARN("ioniq_pcptr_ size %ld", ioniq_cad_pcptr_->points.size());
    ROS_WARN("ioniq_ds_pcptr size %ld", ioniq_cad_pcptr_->points.size());

    ioniq_kdtree_->setInputCloud(ioniq_cad_pcptr_);

    // Down sample sample point
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_large;
    voxel_grid_large.setLeafSize(f_voxel_grid_size_surfel_sample_m_, f_voxel_grid_size_surfel_sample_m_, f_voxel_grid_size_surfel_sample_m_);
    voxel_grid_large.setInputCloud(ioniq_cad_pcptr_);
    voxel_grid_large.filter(*ioniq_cad_surfel_sample_pcptr_);

    double initial_surfel_time = omp_get_wtime();

    int i_ioniq_ds_pcptr_num = ioniq_cad_surfel_sample_pcptr_->points.size();

    for(int i = 0 ; i < i_ioniq_ds_pcptr_num ; i++)
    {
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        pcl::PointXYZ pointSel;
        pcl::PointSurfel pointSurf;

        pointSel.x = ioniq_cad_surfel_sample_pcptr_->points[i].x;
        pointSel.y = ioniq_cad_surfel_sample_pcptr_->points[i].y;
        pointSel.z = ioniq_cad_surfel_sample_pcptr_->points[i].z;

        // ioniq_kdtree_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
        ioniq_kdtree_->radiusSearch(pointSel, f_voxel_grid_size_surfel_sample_m_ / 2.0 , pointSearchInd, pointSearchSqDis);

        int i_searched_point_num = pointSearchInd.size();

        if(i_searched_point_num < 5) continue;

        Eigen::MatrixXf matA0(i_searched_point_num, 3);
        Eigen::VectorXf matX0(i_searched_point_num);
        Eigen::MatrixXf matB0(i_searched_point_num, 1);

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();
        

        for (int j = 0; j < i_searched_point_num; j++) {
            matA0(j, 0) = ioniq_cad_pcptr_->points[pointSearchInd[j]].x;
            matA0(j, 1) = ioniq_cad_pcptr_->points[pointSearchInd[j]].y;
            matA0(j, 2) = ioniq_cad_pcptr_->points[pointSearchInd[j]].z;

        }


        matX0 = matA0.colPivHouseholderQr().solve(matB0);
        float pa = matX0(0, 0); // 평면의 방정식의 x계수
        float pb = matX0(1, 0); // 평면의 방정식의 y계수
        float pc = matX0(2, 0); // 평면의 방정식의 z계수
        float pd = 1;

        float ps = sqrt(pa * pa + pb * pb + pc * pc); // 이 값은 1이 아니기 때문에 normalization 해줌

        pa /= ps; pb /= ps; pc /= ps; pd /= ps; // 평면 법선 벡터 크기로 정규화 해줌

        // cov estimation
        Eigen::Vector3f mean = matA0.colwise().mean();
        Eigen::MatrixXf centered = matA0.rowwise() - mean.transpose();
        Eigen::MatrixXf cov = centered.transpose() * centered / float(centered.rows());
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        Eigen::Vector3f eigenvalues = eig.eigenvalues().reverse();
        Eigen::Matrix3f eigenvectors = eig.eigenvectors();

        int minIndex;
        eig.eigenvalues().minCoeff(&minIndex);
        Eigen::Vector3f minEigenvector = eigenvectors.col(minIndex);

        // 평면성 계산
        float planarity = (eigenvalues(0) + eigenvalues(1)) / eigenvalues.sum();
    
        if(planarity < f_min_planarity) planarity = f_min_planarity;

        pointSurf.x = mean(0);
        pointSurf.y = mean(1);
        pointSurf.z = mean(2);

        pointSurf.normal_x = pa;
        pointSurf.normal_y = pb;
        pointSurf.normal_z = pc;
        pointSurf.curvature = pd; // distance
        if(f_min_planarity < 1.0 - FLT_MIN)
            pointSurf.confidence =  (planarity - f_min_planarity) / (1.0 - f_min_planarity);
        else
            pointSurf.confidence = 1.0;
        // pointSurf.confidence = planarity;

        ioniq_cad_surfel_pcptr_->points.push_back(pointSurf);

        if(b_visualize_surfel_ == true){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "velodyne";
            marker.header.stamp = ros::Time::now();
            marker.ns = "surfel";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = pointSurf.x;
            marker.pose.position.y = pointSurf.y;
            marker.pose.position.z = pointSurf.z;

            // Eigen::Quaterniond quaternion = NormalToQuaternion(pa, pb, pc);

            Eigen::Quaternionf quaternion;
            quaternion.setFromTwoVectors(Eigen::Vector3f(0, 0, 1), minEigenvector);

            marker.pose.orientation.x = quaternion.x();
            marker.pose.orientation.y = quaternion.y();
            marker.pose.orientation.z = quaternion.z();
            marker.pose.orientation.w = quaternion.w();

            marker.scale.x = eigenvalues(0) / eigenvalues.sum() * f_voxel_grid_size_surfel_sample_m_ * 2;  // Diameter of the cylinder in x direction
            marker.scale.y = eigenvalues(1) / eigenvalues.sum() * f_voxel_grid_size_surfel_sample_m_ * 2;  // Diameter of the cylinder in y direction
            // marker.scale.x = f_voxel_grid_size_surfel_sample_m_;  // Diameter of the cylinder in x direction
            // marker.scale.y = f_voxel_grid_size_surfel_sample_m_;  // Diameter of the cylinder in y direction
            marker.scale.z = std::max((1.0f - planarity) * 0.5f, 0.01f);  // Height of the cylinder

            auto rgb = ValueToRainbowColor(pointSurf.confidence);

            marker.color.r = (float)(rgb[0]) / 255.0;
            marker.color.g = (float)(rgb[1]) / 255.0;
            marker.color.b = (float)(rgb[2]) / 255.0;
            marker.color.a = planarity;

            // marker.lifetime = 10;

            o_surfel_marker_.markers.push_back(marker);
        }
    }

    double final_surfel_time = omp_get_wtime();
    std::cout<<"Surfel Time: "<<(final_surfel_time - initial_surfel_time)*1000.0<<" ms"<<std::endl;

    ROS_WARN("Surfel size %ld", ioniq_cad_surfel_pcptr_->points.size());
    ioniq_surfel_kdtree_->setInputCloud(ioniq_cad_surfel_pcptr_);

    GenerateHashMap();

    ROS_WARN("Surfel Model Initialized!");
}

void CadRegistration::CallbackMainPointCloud(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
    i_main_point_cloud_ = *msg;
    
    pcl::moveFromROSMsg(i_main_point_cloud_, *i_main_point_cloud_ptr_);
    i_main_lidar_raw_tuple_ = std::make_tuple(i_main_point_cloud_ptr_, i_main_point_cloud_.header.frame_id, i_main_point_cloud_.header.stamp);
    
    i_lidar_struct_.timestamp = i_main_point_cloud_.header.stamp.toSec();
    i_lidar_struct_.points = i_main_point_cloud_ptr_;
    i_lidar_struct_.frame_id = i_main_point_cloud_.header.frame_id;
    
    b_is_new_point_cloud_ = true;
    std::cout<<"Subscribed LiDAR!"<<std::endl;
}

void CadRegistration::CallbackPillarObjects(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_pillar_objects_);

    i_pillar_objects_.time_stamp = msg->header.stamp.toSec();
    
    d_latest_pillar_object_time_ = i_pillar_objects_.time_stamp;
    v_main_lidar_frame_id_ = msg->header.frame_id;

    i_pillar_objects_.object.clear();

    for (const auto& box : msg->boxes) {
        DetectObject obj;

        obj.id = 0;
        obj.detection_confidence = box.value; 
        obj.classification = static_cast<ObjectClass>(box.label);

        obj.dimension.length = box.dimensions.x;
        obj.dimension.width = box.dimensions.y;
        obj.dimension.height = box.dimensions.z;

        obj.state.x = box.pose.position.x;
        obj.state.y = box.pose.position.y;
        obj.state.z = box.pose.position.z - (box.dimensions.z / 2.0); // Center -> Bottom 변환

        tf2::Quaternion q(
            box.pose.orientation.x,
            box.pose.orientation.y,
            box.pose.orientation.z,
            box.pose.orientation.w);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        obj.state.yaw = yaw;

        obj.state.v_x = 0.0;
        obj.state.v_y = 0.0;
        obj.state.a_x = 0.0;
        obj.state.a_y = 0.0;

        i_pillar_objects_.object.push_back(obj);
    }

    b_is_new_pillar_object_ = true;
}

/*
TODO:
1. Sync LiDAR and Pillar
2. Extract Object Point from Pillar box
3. Publish with PointsObjects msg
*/
void CadRegistration::Run(){
    int i_num_objects = 0;
    double d_exec_time_ms = 0.0;

    double d_start_time_sec = ros::Time::now().toSec();

    pcl::PointCloud<pcl::PointXYZI>::Ptr synced_point_cloud;
    PointsObjects o_points_objects;

    LidarDataStruct cur_lidar_struct; {
        std::lock_guard<std::mutex> lock(mutex_main_point_cloud_);
        cur_lidar_struct = i_lidar_struct_;
    }

    DetectObjects cur_objects; {
        std::lock_guard<std::mutex> lock(mutex_pillar_objects_);
        cur_objects = i_pillar_objects_;
    }

    // Push back Lidar data to deque
    if(b_is_new_point_cloud_){
        deq_lidar_struct_.push_back(cur_lidar_struct);

        while(deq_lidar_struct_.size() > 10)
            deq_lidar_struct_.pop_front();

        b_is_new_point_cloud_ = false;
    }

    // End if no new pillar objects
    if(b_is_new_pillar_object_ == false) {
        ROS_WARN("NO Pillar Object");
        return;
    }

    // Start Main Run
    std::cout<<" "<<std::endl;
    ROS_INFO("[Run] Start main Run");

    // std::cout<<"LiDAR  Time:"<<cur_lidar_struct.timestamp<<std::endl;
    // std::cout<<"Pillar Time:"<<cur_objects.time_stamp<<std::endl;
    if(SyncLidarDetection(cur_objects, synced_point_cloud) == false)
        return;

    v_d_extract_points_time_ms_.clear();

    // o_points_objects time == cur_objects time
    if(ExtractPointsFromDetection(cur_objects, synced_point_cloud, o_points_objects) == false)
        return;

    o_refined_cad_pcptr_->points.clear();
    o_refined_cad_surfel_pcptr_->points.clear();

    for(int i = 0; i< o_points_objects.object.size(); i++){
        AngleBasedTime(o_points_objects.object[i].state);
    }

    RegistrationRefinement(o_points_objects);

    UpdateJSKLidarObjects(o_points_objects);
    UpdatePointsObjectsPc(o_points_objects);

    b_is_new_pillar_object_ = false;
    b_is_new_publish_ = true;

    double d_end_time_sec = ros::Time::now().toSec();
    d_exec_time_ms = (d_end_time_sec - d_start_time_sec)*1000.0;
    i_num_objects = o_points_objects.object.size();
    ROS_INFO("Extract %d Objects in %f ms", i_num_objects, d_exec_time_ms);
}

void CadRegistration::Publish(){
    if(b_is_new_publish_){
        // p_points_objects_pc_.publish(o_points_objects_pc_);
        p_refined_cad_pc_.publish(o_refined_cad_pc_);
        p_jsk_refined_box_.publish(o_jsk_lidar_objects_);
        p_surfels_.publish(o_surfel_marker_);

        b_is_new_publish_ = false;
    }
}

bool CadRegistration::SyncLidarDetection(DetectObjects& i_objects, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& o_synced_point_cloud)
{
    double d_lidar_time, d_i_objects_time, d_iter_lidar_time;
    int i_lidar_struct_size;

    d_i_objects_time = i_objects.time_stamp;

    i_lidar_struct_size = deq_lidar_struct_.size();
    
    for(int i = deq_lidar_struct_.size() - 1; i >= 0; i--){
        d_iter_lidar_time = deq_lidar_struct_[i].timestamp;

        if(fabs(d_iter_lidar_time - d_i_objects_time) < 0.01){
            o_synced_point_cloud = deq_lidar_struct_[i].points;
            return true;
        }
    }


    return false;
}
bool CadRegistration::ExtractPointsFromDetection(DetectObjects& i_objects, 
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& i_synced_point_cloud,
                            PointsObjects& o_points_objects)
{

    double extract_start = omp_get_wtime();

    o_points_objects.time_stamp = i_objects.time_stamp;
    v_d_extract_points_time_ms_.reserve(i_objects.object.size());

    for (const auto& obj : i_objects.object) {
        double extract_start_per_obj = omp_get_wtime();
        PointsObject p_obj;
        p_obj.id = obj.id;
        p_obj.detection_confidence = obj.detection_confidence;
        p_obj.classification = obj.classification;
        p_obj.dynamic_state = obj.dynamic_state;
        p_obj.dimension = obj.dimension;
        p_obj.state = obj.state;

        double cos_yaw = cos(obj.state.yaw);
        double sin_yaw = sin(obj.state.yaw);

        double obj_x = obj.state.x;
        double obj_y = obj.state.y;
        double obj_z = obj.state.z;

        for (const auto& point : *i_synced_point_cloud) {
            // 점이 객체의 경계 상자 내에 있는지 확인

            // 회전 적용
            double localX = cos_yaw * (point.x - obj_x) + sin_yaw * (point.y - obj_y);
            double localY = -sin_yaw * (point.x - obj_x) + cos_yaw * (point.y - obj_y);
            double localZ = point.z - obj_z;

            double e = 0.2;

            if (localX >= - obj.dimension.length / 2 - e &&
                localX <= obj.dimension.length / 2 + e &&
                localY >= - obj.dimension.width / 2 - e &&
                localY <= obj.dimension.width / 2 + e &&
                localZ >= 0.3 &&
                localZ <= obj.dimension.height) {

                pcl::PointXYZI local_point;

                local_point.x = localX;
                local_point.y = localY;
                local_point.z = localZ;
                local_point.intensity = point.intensity;
                p_obj.points.push_back(local_point);
            }
        }

        double extract_end_per_obj = omp_get_wtime();

        if (!p_obj.points.empty()) {
            o_points_objects.object.push_back(p_obj);
            v_d_extract_points_time_ms_.push_back((extract_end_per_obj - extract_start_per_obj) * 1000.0);
        }
    }

    double extract_end = omp_get_wtime();

    std::cout<<"Point Extraction: "<< (extract_end - extract_start) * 1000.0<<" ms"<<std::endl;

    return true;
}

void CadRegistration::AngleBasedTime(ObjectState& i_object_state)
{
    double d_object_angle_deg = atan2(i_object_state.y, i_object_state.x) * 180.0/M_PI;
    double d_extra_time = - 0.1 * (d_object_angle_deg + 180.0)/360.0;

    std::cout<<"Angle: "<<d_object_angle_deg<<" deg, Extra: "<< d_extra_time<<" sec."<<std::endl;

    i_object_state.time_stamp += d_extra_time;
}

bool CadRegistration::CheckBehind(ObjectState i_object_state)
{
    double d_object_angle_deg = atan2(i_object_state.y, i_object_state.x) * 180.0/M_PI;
    if(fabs(d_object_angle_deg) > 170){
        return true;
    }

    return false;
}

void CadRegistration::RegistrationRefinement(PointsObjects& point_objects)
{   
    double d_start_time_sec = ros::Time::now().toSec();
    ROS_INFO("RegistrationRefinement Started");
    for(int object_idx = 0; object_idx < point_objects.object.size(); ++object_idx){
        auto& object = point_objects.object[object_idx];
        d_voxel_filter_time_ms_ = 0.0;
        d_registration_loop_time_ms_ = 0.0;
        
        if(CheckBehind(object.state) == true){
            ROS_WARN("Object is Behind!");
            continue;
        } else {
            if(RunRegistration(object) == true){
                ROS_INFO("Registration Success dx: %f, dy: %f, dz: %f, dyaw: %f", 
                        f_arr_init_to_cad_[3], f_arr_init_to_cad_[4], f_arr_init_to_cad_[5], f_arr_init_to_cad_[2] * 180.0/M_PI);
                std::ofstream file(str_calc_time_csv_file_, std::ios::app);
                if (file.is_open()) {
                    if (object_idx < v_d_extract_points_time_ms_.size()) {
                        file << v_d_extract_points_time_ms_[object_idx] << ","
                            << d_voxel_filter_time_ms_ << ","
                            << d_registration_loop_time_ms_ << "\n";
                    }
                    file.close();
                }
            }   
            else{
                ROS_WARN("Registration Fail!");
            }
        }

    }


    double d_end_time_sec = ros::Time::now().toSec();
    double d_exec_time_ms = (d_end_time_sec - d_start_time_sec)*1000.0;
    ROS_INFO("RegistrationRefinement Done in %f ms",d_exec_time_ms);
}

// Target: CAD, Source: i_cloud
// 하지만 최종적으로 이동되는건 PointsObject 의 RT  
bool CadRegistration::RunRegistration(PointsObject& p_object)
{
    d_optimization_ms_ = 0.0;
    d_surf_ms_ = 0.0;
    d_kd_tree_ms_ = 0.0;
    d_plane_equation_ms_ = 0.0;
    d_point_to_map_ms_ = 0.0;
  
    i_optimization_time_num_ = 0;
    i_surf_time_num_ = 0;
    i_kd_tree_time_num_ = 0;
    i_plane_equation_time_num_ = 0;
    i_point_to_map_time_num_ = 0;  
    
    bool b_is_optimized = true;
    b_isDegenerate_ = false;
    

    for (int i = 0; i < 6; ++i){
        f_arr_init_to_cad_[i] = 0;
    }

    double filter_start = omp_get_wtime();
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // // 필터 적용
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud = p_object.points;
    voxel_grid_cluster_.setInputCloud(cloud);
    voxel_grid_cluster_.filter(*filtered_cloud);
    // VoxelFilter(p_object.points, *filtered_cloud);

    double filter_end = omp_get_wtime();
    d_voxel_filter_time_ms_ = (filter_end - filter_start)*1000.0;
    std::cout<<"DS: "<<(filter_end - filter_start)*1000.0 <<" ms "<<std::endl;

    std::cout<<"DS odri: "<<filtered_cloud->points.size()<<std::endl;

    double registration_start = omp_get_wtime();

    int iterCount = 0;
    for (iterCount = 0; iterCount < cfg_max_iteration; iterCount++) 
    {
        laserCloudOri_->clear();
        coeffSel_->clear();

        double time_start = omp_get_wtime();

        i_direct_search_num_ = 0;
        i_rel_search_num_ = 0;
        i_not_searched_num_ = 0;

        if (b_use_p2p_registration_ == false) {
            // SurfOptimization(iterCount, *filtered_cloud); 
            SurfelOptimization(iterCount, *filtered_cloud); 

            std::cout<<"Dir search: "<< i_direct_search_num_<<" Sur: "<< i_rel_search_num_<<" Not: "<<i_not_searched_num_<<std::endl;
        }
        else {
            // Point to Point Registration
            Point2PointOptimization(iterCount, *filtered_cloud);
        }

        double surf_end = omp_get_wtime();
        d_surf_ms_ += (surf_end - time_start) * 1000.0;
        i_surf_time_num_++;

        if(laserCloudOri_->size() < 10){
            ROS_WARN("Too Small Correspondence %ld", laserCloudOri_->size());
            return false;
        }

        if (b_use_p2p_registration_ == false){
            if (LMOptimization4DoF(iterCount) == true){
                // ROS_INFO("Converged in iterCount %d",iterCount);

                break;
            }
        }
        else{
            if (LMOptimization4DoFP2P(iterCount) == true)
            {
                ROS_INFO("P2P: Converged in iterCount %d",iterCount);
                b_is_optimized = true;

                break;
            }
        }

        double optimization_end = omp_get_wtime();

        d_optimization_ms_ += (optimization_end - surf_end) * 1000.0;
        i_optimization_time_num_++;

    }

    double registration_end = omp_get_wtime();
    d_registration_loop_time_ms_ = (registration_end - registration_start)*1000.0;
    std::cout<<"Registration Time: "<<(registration_end - registration_start)*1000.0 <<std::endl;
    std::cout<<"Surf Total: "<<d_surf_ms_<<" mean: "<<d_surf_ms_/i_surf_time_num_<<
            " Opt Total: "<<d_optimization_ms_<<" mean: "<<d_optimization_ms_/ i_optimization_time_num_<<std::endl;
    std::cout<<"kdtree Total: "<<d_kd_tree_ms_<<" mean: "<<d_kd_tree_ms_/ i_kd_tree_time_num_<<
                " plane Total: "<<d_plane_equation_ms_<<" mean: "<<d_plane_equation_ms_/i_plane_equation_time_num_<<
                " point_to_map Total: "<<d_point_to_map_ms_<<" mean: "<<d_point_to_map_ms_/i_point_to_map_time_num_<<std::endl;

    if(b_is_optimized == false){
        return false;
    }

    // TODO: Fix p_object xyz yaw
    // pc_to_cad_tf 에 Box Local pointcloud를 Box에 맞추는 RT 저장됨.
    // pc_to_cad_tf 를 역변환 하면 기존에 구한 box를 point cloud에 맞출 수 있음
    // point cloud는 pc_to_cad_tf로 옮기고, Box만 inverse

    p_object.dimension.width = 2.13;
    p_object.dimension.length = 4.59;
    
    UpdatePointAssociateToMap();
    double gx, gy, gz, lx, ly, lz,l_del_yaw, state_yaw;

    l_del_yaw = -f_arr_init_to_cad_[2];
    state_yaw = p_object.state.yaw;
    lx = -f_arr_init_to_cad_[3];
    ly = -f_arr_init_to_cad_[4];
    lz = -f_arr_init_to_cad_[5];

    gx = lx * cos(state_yaw) - ly * sin(state_yaw);
    gy = lx * sin(state_yaw) + ly * cos(state_yaw);

    p_object.state.x += gx;
    p_object.state.y += gy;
    p_object.state.z += lz;
    p_object.state.yaw += l_del_yaw;

    p_object.dimension.height = 1.592;

    if(b_visualize_cad_points_ == true){
        pcl::transformPointCloud(*filtered_cloud, p_object.points, pc_to_cad_tf);

        Eigen::Affine3f cad_global_tf = pcl::getTransformation(p_object.state.x, p_object.state.y, p_object.state.z,
                                            0, 0, p_object.state.yaw);

        pcl::PointCloud<pcl::PointSurfel> temp_ioniq_surfel_pc;
        pcl::transformPointCloud(*ioniq_cad_surfel_pcptr_, temp_ioniq_surfel_pc, cad_global_tf);
        *o_refined_cad_surfel_pcptr_ += temp_ioniq_surfel_pc;

        if(b_visualize_sample_cad_ == true){
            *o_refined_cad_surfel_pcptr_ += *ioniq_cad_surfel_pcptr_;
        }
    }

    return true;
}

void CadRegistration::SurfOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points)
{
    // box_points: Box Local 좌표계의 포인트
    // ioniq_kdtree_: 원점이 Ioniq 중심 바닥점인 point cloud
    // f_arr_init_to_cad_: Iteration에 걸쳐서 RT를 저장할 array
    // pc_to_cad_tf: RT가 저장된 Affine3f

    UpdatePointAssociateToMap(); // 매핑될 위치를 Eigen Affine3f 형식으로 저장 pc_to_cad_tf

    for (int i = 0; i < box_points.points.size(); i++)
    {
        pcl::PointXYZ pointOri, pointSel;
        pcl::PointXYZI coeff;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // pointOri = box_points.points[i];
        pointOri.x = box_points.points[i].x;
        pointOri.y = box_points.points[i].y;
        pointOri.z = box_points.points[i].z;
        
        // Local 좌표계의 point를 pc_to_cad_tf 좌표계 상으로 이동시켜줌

        double time_start = omp_get_wtime();

        PointAssociateToMap(&pointOri, &pointSel); 

        double point_to_map_end = omp_get_wtime();
        d_point_to_map_ms_ += (point_to_map_end - time_start) * 1000.0;
        i_point_to_map_time_num_++;

        if(isnan(pointSel.x) || isnan(pointSel.y) || isnan(pointSel.z)) continue;
        
        ioniq_kdtree_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        double kd_tree_end = omp_get_wtime();
        d_kd_tree_ms_ += (kd_tree_end - point_to_map_end) * 1000.0;  
        i_kd_tree_time_num_++;

        Eigen::Matrix<float, 5, 3> matA0;
        Eigen::Matrix<float, 5, 1> matB0;
        Eigen::Vector3f matX0;

        matA0.setZero();
        matB0.fill(-1);
        matX0.setZero();

        // 5번째로 가까운 포인트가 1.0 m 에 있는가?
        if (pointSearchSqDis[4] < 1.0 ) {
            for (int j = 0; j < 5; j++) {
                matA0(j, 0) = ioniq_cad_pcptr_->points[pointSearchInd[j]].x;
                matA0(j, 1) = ioniq_cad_pcptr_->points[pointSearchInd[j]].y;
                matA0(j, 2) = ioniq_cad_pcptr_->points[pointSearchInd[j]].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);
            float pa = matX0(0, 0); // 평면의 방정식의 x계수
            float pb = matX0(1, 0); // 평면의 방정식의 y계수
            float pc = matX0(2, 0); // 평면의 방정식의 z계수
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc); // 이 값은 1이 아니기 때문에 normalization 해줌

            pa /= ps; pb /= ps; pc /= ps; pd /= ps; // 평면 법선 벡터 크기로 정규화 해줌

            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                if (fabs(pa * ioniq_cad_pcptr_->points[pointSearchInd[j]].x +
                         pb * ioniq_cad_pcptr_->points[pointSearchInd[j]].y +
                         pc * ioniq_cad_pcptr_->points[pointSearchInd[j]].z + pd) > 0.2) { // 5개 뽑은거에서 대입했을때 거리가 0.2보다 크면 평면조건에 벗어나므로 평면아님 땅땅땅
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) { // 5개 구한게 평면이면 ~~
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; // 평면과 점 사이거리

                // float s = 1 - 0.9 * fabs(pd2) / 
                //         sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z)); 

                float s = 1.0 - (1.6 - pointSel.z);

                if(s > 1.0) s = 1.0;
                if( s < 0.2) s = 0.2;

                // float s = 1.0;

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    
                    laserCloudOri_->points.push_back(pointOri);
                    coeffSel_->points.push_back(coeff);
                }
            }
        }

        double plane_equation_end = omp_get_wtime();
        d_plane_equation_ms_ += (plane_equation_end - kd_tree_end) * 1000.0;  
        i_plane_equation_time_num_++;
    
    }
}

void CadRegistration::SurfelOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points)
{
    // box_points: Box Local 좌표계의 포인트
    // ioniq_kdtree_: 원점이 Ioniq 중심 바닥점인 point cloud
    // f_arr_init_to_cad_: Iteration에 걸쳐서 RT를 저장할 array
    // pc_to_cad_tf: RT가 저장된 Affine3f

    UpdatePointAssociateToMap(); // 매핑될 위치를 Eigen Affine3f 형식으로 저장 pc_to_cad_tf

    std::vector<int> pointSearchInd;
    pointSearchInd.resize(1);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.resize(1);

    float pa, pb, pc, pd, score, pd2;

    for (int i = 0; i < box_points.points.size(); i++)
    {
        pcl::PointXYZ pointOri;
        pcl::PointSurfel pointSurf;
        pcl::PointXYZI coeff;

        // pointOri = box_points.points[i];
        pointOri.x = box_points.points[i].x;
        pointOri.y = box_points.points[i].y;
        pointOri.z = box_points.points[i].z;
        
        // Local 좌표계의 point를 pc_to_cad_tf 좌표계 상으로 이동시켜줌

        double time_start = omp_get_wtime();

        PointAssociateToMapSurfel(&pointOri, &pointSurf); 
        if(isnan(pointSurf.x) || isnan(pointSurf.y) || isnan(pointSurf.z)) continue;

        double point_to_map_end = omp_get_wtime();
        d_point_to_map_ms_ += (point_to_map_end - time_start) * 1000.0;
        i_point_to_map_time_num_++;
        
        if(b_use_hash_search_ == true)
            HashSearch(pointSurf,pointSearchInd[0], pointSearchSqDis[0]);
        else    
            ioniq_surfel_kdtree_->nearestKSearch(pointSurf, 1, pointSearchInd, pointSearchSqDis);

        double kd_tree_end = omp_get_wtime();
        d_kd_tree_ms_ += (kd_tree_end - point_to_map_end) * 1000.0;  
        i_kd_tree_time_num_++;

        if(pointSearchSqDis[0] < 0.5){
            pa = ioniq_cad_surfel_pcptr_->points[pointSearchInd[0]].normal_x;
            pb = ioniq_cad_surfel_pcptr_->points[pointSearchInd[0]].normal_y;
            pc = ioniq_cad_surfel_pcptr_->points[pointSearchInd[0]].normal_z;
            pd = ioniq_cad_surfel_pcptr_->points[pointSearchInd[0]].curvature;
            score = ioniq_cad_surfel_pcptr_->points[pointSearchSqDis[0]].confidence; // Planarity

            pd2 = pa * pointSurf.x + pb * pointSurf.y + pc * pointSurf.z + pd; // 평면과 점 사이거리

            if(b_use_gm_kernel_ == true){
                score *= (f_gm_kernel_simga_ * f_gm_kernel_simga_) / ((f_gm_kernel_simga_ + pd2*pd2) * (f_gm_kernel_simga_ + pd2*pd2));
            }

            coeff.x = score * pa;
            coeff.y = score * pb;
            coeff.z = score * pc;
            coeff.intensity = score * pd2;
                
            laserCloudOri_->points.push_back(pointOri);
            coeffSel_->points.push_back(coeff);
        }

        double plane_equation_end = omp_get_wtime();
        d_plane_equation_ms_ += (plane_equation_end - kd_tree_end) * 1000.0;  
        i_plane_equation_time_num_++;
    }
}

void CadRegistration::Point2PointOptimization(int iterCount, const pcl::PointCloud<pcl::PointXYZI> box_points)
{
    // box_points: Box Local 좌표계의 포인트
    // ioniq_kdtree_: 원점이 Ioniq 중심 바닥점인 point cloud
    // f_arr_init_to_cad_: Iteration에 걸쳐서 RT를 저장할 array
    // pc_to_cad_tf: RT가 저장된 Affine3f

    UpdatePointAssociateToMap(); // 매핑될 위치를 Eigen Affine3f 형식으로 저장 pc_to_cad_tf

    std::vector<int> pointSearchInd;
    pointSearchInd.resize(1);
    std::vector<float> pointSearchSqDis;
    pointSearchSqDis.resize(1);

    for (int i = 0; i < box_points.points.size(); i++)
    {
        pcl::PointXYZ pointOri;
        pcl::PointXYZ pointSel;
        pcl::PointXYZI coeff;

        Eigen::MatrixXf coeffs_eigen = Eigen::MatrixXf::Zero(box_points.points.size(), 3);

        Eigen::MatrixXf pt_cloud_closest_target_point = Eigen::MatrixXf::Zero(1, 3);
        Eigen::MatrixXf pt_cloud_curr_source_point = Eigen::MatrixXf::Zero(1, 3);

        pointOri.x = box_points.points[i].x;
        pointOri.y = box_points.points[i].y;
        pointOri.z = box_points.points[i].z;
        
        // Local 좌표계의 point를 pc_to_cad_tf 좌표계 상으로 이동시켜줌
        double time_start = omp_get_wtime();
        PointAssociateToMap(&pointOri, &pointSel);
        if(isnan(pointSel.x) || isnan(pointSel.y) || isnan(pointSel.z)) continue;

        double point_to_map_end = omp_get_wtime();
        d_point_to_map_ms_ += (point_to_map_end - time_start) * 1000.0;
        i_point_to_map_time_num_++;
        
        // 가장 가까운 CAD 모델의 포인트 찾기
        ioniq_kdtree_->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        double kd_tree_end = omp_get_wtime();
        d_kd_tree_ms_ += (kd_tree_end - point_to_map_end) * 1000.0;  
        i_kd_tree_time_num_++;

        pt_cloud_closest_target_point(0, 0) = ioniq_cad_pcptr_->points[pointSearchInd[0]].x;
        pt_cloud_closest_target_point(0, 1) = ioniq_cad_pcptr_->points[pointSearchInd[0]].y;
        pt_cloud_closest_target_point(0, 2) = ioniq_cad_pcptr_->points[pointSearchInd[0]].z;
        pt_cloud_curr_source_point(0, 0) = pointSel.x;
        pt_cloud_curr_source_point(0, 1) = pointSel.y;
        pt_cloud_curr_source_point(0, 2) = pointSel.z;
        coeffs_eigen.row(i) = pt_cloud_curr_source_point.row(0).adjoint() - pt_cloud_closest_target_point.row(0).adjoint();

        if(pointSearchSqDis[0] < 0.5) {
            // Point-to-Point 거리 계산
            float dx = ioniq_cad_pcptr_->points[pointSearchInd[0]].x - pointSel.x;
            float dy = ioniq_cad_pcptr_->points[pointSearchInd[0]].y - pointSel.y;
            float dz = ioniq_cad_pcptr_->points[pointSearchInd[0]].z - pointSel.z;
            
            // 거리에 따른 가중치 계산 (선택적)
            float weight = 1.0;
            // float dist = sqrt(dx*dx + dy*dy + dz*dz);
            float dist = sqrt(pointSearchSqDis[0]);
            if(b_use_gm_kernel_) {
                weight = (f_gm_kernel_simga_ * f_gm_kernel_simga_) / 
                        ((f_gm_kernel_simga_ + dist*dist) * (f_gm_kernel_simga_ + dist*dist));
            }

            // 계수 설정
            // coeff.x = weight * dx;
            coeff.x = weight * coeffs_eigen(i, 0);
            // coeff.y = weight * dy;
            coeff.y = weight * coeffs_eigen(i, 1);
            // coeff.z = weight * dz;
            coeff.z = weight * coeffs_eigen(i, 2);
            // coeff.intensity = weight * dist;
            coeff.intensity = weight * pointSearchSqDis[0];
                
            laserCloudOri_->points.push_back(pointOri);
            coeffSel_->points.push_back(coeff);
        }

        double plane_equation_end = omp_get_wtime();
        d_plane_equation_ms_ += (plane_equation_end - kd_tree_end) * 1000.0;  
        i_plane_equation_time_num_++;
        // ROS_INFO("P2P: laserCloudOri_ size: %d", laserCloudOri_->size());
    }
}


void CadRegistration::UpdatePointAssociateToMap()
{
    pc_to_cad_tf = Trans2Affine3f(f_arr_init_to_cad_);
}

Eigen::Affine3f CadRegistration::Trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

// Local 좌표계의 point를 pc_to_cad_tf 좌표계 상으로 이동시켜줌
void CadRegistration::PointAssociateToMap(pcl::PointXYZ const * const pi, pcl::PointXYZ * const po)
{
    po->x = pc_to_cad_tf(0,0) * pi->x + pc_to_cad_tf(0,1) * pi->y + pc_to_cad_tf(0,2) * pi->z + pc_to_cad_tf(0,3);
    po->y = pc_to_cad_tf(1,0) * pi->x + pc_to_cad_tf(1,1) * pi->y + pc_to_cad_tf(1,2) * pi->z + pc_to_cad_tf(1,3);
    po->z = pc_to_cad_tf(2,0) * pi->x + pc_to_cad_tf(2,1) * pi->y + pc_to_cad_tf(2,2) * pi->z + pc_to_cad_tf(2,3);
}

// Local 좌표계의 point를 pc_to_cad_tf 좌표계 상으로 이동시켜줌
void CadRegistration::PointAssociateToMapSurfel(pcl::PointXYZ const * const pi, pcl::PointSurfel * const po)
{
    po->x = pc_to_cad_tf(0,0) * pi->x + pc_to_cad_tf(0,1) * pi->y + pc_to_cad_tf(0,2) * pi->z + pc_to_cad_tf(0,3);
    po->y = pc_to_cad_tf(1,0) * pi->x + pc_to_cad_tf(1,1) * pi->y + pc_to_cad_tf(1,2) * pi->z + pc_to_cad_tf(1,3);
    po->z = pc_to_cad_tf(2,0) * pi->x + pc_to_cad_tf(2,1) * pi->y + pc_to_cad_tf(2,2) * pi->z + pc_to_cad_tf(2,3);
}

bool CadRegistration::LMOptimization4DoF(int iterCount)
{
    // // This optimization is from the original loam_velodyne by Ji Zhang

    double init_start = omp_get_wtime();
    float sin_yaw = sin(f_arr_init_to_cad_[2]); // yaw
    float cos_yaw = cos(f_arr_init_to_cad_[2]); //

    int laserCloudSelNum = laserCloudOri_->size();

    Eigen::MatrixXf matA(laserCloudSelNum, 4);
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::MatrixXf matAtA(4, 4);
    Eigen::MatrixXf matAtB(4, 1);
    Eigen::VectorXf matX(4);
    

    for (int i = 0; i < laserCloudSelNum; i++) {
        float yaw_grad = (cos_yaw*laserCloudOri_->points[i].x + sin_yaw*laserCloudOri_->points[i].y) * coeffSel_->points[i].y
                       + (sin_yaw*laserCloudOri_->points[i].x - cos_yaw*laserCloudOri_->points[i].y) * coeffSel_->points[i].x;

        matA(i, 0) = yaw_grad; // yaw gradient
        matA(i, 1) = coeffSel_->points[i].x; // x gradient
        matA(i, 2) = coeffSel_->points[i].y; // y gradient
        matA(i, 3) = coeffSel_->points[i].z; // z gradient
        matB(i)    = -coeffSel_->points[i].intensity; // error
    }

    // 계산 과정
    Eigen::MatrixXf matAt = matA.transpose();
    Eigen::MatrixXf matAtAdiag = (matAt * matA).diagonal().asDiagonal();

    matAtA = matAt * matA + f_lm_lambda_ * matAtAdiag;
    matAtB = matAt * matB;

    // 선형 시스템 해결
    matX = matAtA.colPivHouseholderQr().solve(matAtB);

    if (iterCount == 0) {

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigSolver(matAtA);
        Eigen::VectorXf eigenValues = eigSolver.eigenvalues(); // 오름차순
        Eigen::MatrixXf eigenVectors = eigSolver.eigenvectors().transpose(); // 행에 저장되도록 변경.

        Eigen::MatrixXf eigenVectors2 = eigenVectors;

        // float eigenThresholds[4] = {10, 3, 2, 1.0};
        // float eigenThresholds[4] = {2.5, 0.75, 0.5, 0.2};
        float eigenThresholds[4] = {0.1, 0.1, 0.2, 1.0};
        b_isDegenerate_ = false;

        for (int i = 0; i < 4; i++) {
            if (eigenValues[i] < eigenThresholds[i]) {
                eigenVectors2.row(i).setZero(); // 해당 고유벡터를 0으로 설정
                std::cout<<"EV: "<<eigenValues[i]<<" < "<< eigenThresholds[i]<<std::endl;
                b_isDegenerate_ = true;
            }
            else{
                break;
            }
        }

        matP = eigenVectors.inverse() * eigenVectors2;

        std::cout<<"matP"<<std::endl;
        std::cout<<matP<<std::endl;
    }

    if (b_isDegenerate_) {
        std::cout << "IsDegenerate!" << std::endl;
        
        std::cout<<matX<<std::endl;
        Eigen::VectorXf matX2 = matP * matX;
        matX = matX2;

        std::cout<<" to " <<std::endl;
        std::cout<<matX<<std::endl;
    }

    f_arr_init_to_cad_[2] += matX(0); // yaw
    f_arr_init_to_cad_[3] += matX(1); // x
    f_arr_init_to_cad_[4] += matX(2); // y
    f_arr_init_to_cad_[5] += matX(3); // z

    // std::cout<<"dx: "<<matX.at<float>(1, 0)<<" dy: "<<matX.at<float>(2, 0)<<" dz: "<<matX.at<float>(3, 0)<<
    //             " dyaw: "<<matX.at<float>(0, 0)*180/M_PI<<std::endl;

    float deltaR = sqrt(pow(pcl::rad2deg(matX(0)), 2));
    float deltaT = sqrt(
                        pow(matX(1), 2) +
                        pow(matX(2), 2) +
                        pow(matX(3), 2));

    double other_end = omp_get_wtime();
    // std::cout<<"Other Time: "<<(other_end - other_start) * 1000.0 <<std::endl;

    if (iterCount > 2 && deltaR < 0.5 && deltaT < 0.01) {
        return true; // converged
    }
    return false; // keep optimizing

}

void CadRegistration::GenerateHashMap()
{
    f_min_hash_x_m_ = - ceil(f_max_hash_x_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;
    f_max_hash_x_m_ = ceil(f_max_hash_x_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;

    f_min_hash_y_m_ = - ceil(f_max_hash_y_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;
    f_max_hash_y_m_ = ceil(f_max_hash_y_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;

    f_min_hash_z_m_ = - ceil(f_max_hash_y_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;
    f_max_hash_z_m_ = ceil(f_max_hash_y_m_ / f_hash_grid_size_m_) * f_hash_grid_size_m_;

    i_hash_x_num_ = ceil((f_max_hash_x_m_ - f_min_hash_x_m_) / f_hash_grid_size_m_);
    i_hash_y_num_ = ceil((f_max_hash_y_m_ - f_min_hash_y_m_) / f_hash_grid_size_m_);
    i_hash_z_num_ = ceil((f_max_hash_z_m_ - f_min_hash_z_m_) / f_hash_grid_size_m_);

    i_hash_total_num_ = i_hash_x_num_ * i_hash_y_num_ * i_hash_z_num_;

    int i_surfel_pcptr_num = ioniq_cad_surfel_pcptr_->points.size();

    vec_voxel_index_.resize(i_hash_total_num_);

    for(int i = 0; i < i_surfel_pcptr_num; i++)
    {
        int voxel_ind = GetVoxelIndex(ioniq_cad_surfel_pcptr_->points[i].x, ioniq_cad_surfel_pcptr_->points[i].y, ioniq_cad_surfel_pcptr_->points[i].z);
        if(voxel_ind >= 0){
            vec_voxel_index_[voxel_ind].push_back(i);
        }
    }
}

bool CadRegistration::LMOptimization4DoFP2P(int iterCount)
{
    double init_start = omp_get_wtime();
    float sin_yaw = sin(f_arr_init_to_cad_[2]); // yaw
    float cos_yaw = cos(f_arr_init_to_cad_[2]); //

    int laserCloudSelNum = laserCloudOri_->size();

    Eigen::MatrixXf matA(laserCloudSelNum, 4);
    Eigen::VectorXf matB(laserCloudSelNum);
    Eigen::MatrixXf matAtA(4, 4);
    Eigen::MatrixXf matAtB(4, 1);
    Eigen::VectorXf matX(4);

    for (int i = 0; i < laserCloudSelNum; i++) {
        // Point-to-Point 정합을 위한 그래디언트 계산
        // float yaw_grad = -sin_yaw * laserCloudOri_->points[i].x * coeffSel_->points[i].x 
        //                + cos_yaw * laserCloudOri_->points[i].y * coeffSel_->points[i].x
        //                - cos_yaw * laserCloudOri_->points[i].x * coeffSel_->points[i].y
        //                - sin_yaw * laserCloudOri_->points[i].y * coeffSel_->points[i].y;
        float yaw_grad = (cos_yaw*laserCloudOri_->points[i].x + sin_yaw*laserCloudOri_->points[i].y) * coeffSel_->points[i].y
                       + (sin_yaw*laserCloudOri_->points[i].x - cos_yaw*laserCloudOri_->points[i].y) * coeffSel_->points[i].x;

        matA(i, 0) = yaw_grad; // yaw gradient
        matA(i, 1) = coeffSel_->points[i].x; // x gradient
        matA(i, 2) = coeffSel_->points[i].y; // y gradient
        matA(i, 3) = coeffSel_->points[i].z; // z gradient
        matB(i)    = -coeffSel_->points[i].intensity; // error
    }

    // 계산 과정
    Eigen::MatrixXf matAt = matA.transpose();
    Eigen::MatrixXf matAtAdiag = (matAt * matA).diagonal().asDiagonal();

    matAtA = matAt * matA + f_lm_lambda_ * matAtAdiag;  
    // matAtA = matAt * matA + f_lm_lambda_ * Eigen::MatrixXf::Identity(4, 4);  
    matAtB = matAt * matB;

    // 선형 시스템 해결
    matX = matAtA.colPivHouseholderQr().solve(matAtB);
    // matX = matAtA.ldlt().solve(matAtB);

    if (iterCount == 0) {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigSolver(matAtA);
        Eigen::VectorXf eigenValues = eigSolver.eigenvalues(); // 오름차순
        Eigen::MatrixXf eigenVectors = eigSolver.eigenvectors().transpose(); // 행에 저장되도록 변경.
        Eigen::MatrixXf eigenVectors2 = eigenVectors;
        
        // float eigenThresholds[4] = {2.5, 0.75, 0.5, 0.2};
        // float eigenThresholds[4] = {0.01, 0.01, 0.02, 1.0};
        // float eigenThresholds[4] = {0.1, 0.1, 0.2, 1.0};
        float eigenThresholds[4] = {0.01, 0.01, 0.02, 1.0};
        b_isDegenerate_ = false;
        
        for (int i = 0; i < 4; i++) {
            if (eigenValues[i] < eigenThresholds[i]) {
                eigenVectors2.row(i).setZero(); // 해당 고유벡터를 0으로 설정
                b_isDegenerate_ = true;
                std::cout<<"EV: "<<eigenValues[i]<<" < "<< eigenThresholds[i]<<std::endl;
            }
            else{
                break;
            }
        }
        
        matP = eigenVectors.inverse() * eigenVectors2;

        std::cout<<"matP"<<std::endl;
        std::cout<<matP<<std::endl;
    }

    if (b_isDegenerate_) {
        std::cout << "IsDegenerate!" << std::endl;
        Eigen::VectorXf matX2 = matP * matX;
        matX = matX2;
    }

    f_arr_init_to_cad_[2] += matX(0); // yaw
    f_arr_init_to_cad_[3] += matX(1); // x
    f_arr_init_to_cad_[4] += matX(2); // y
    f_arr_init_to_cad_[5] += matX(3); // z

    float deltaR = sqrt(pow(pcl::rad2deg(matX(0)), 2));
    float deltaT = sqrt(
                    pow(matX(1), 2) +
                    pow(matX(2), 2) +
                    pow(matX(3), 2));

    std::cout<< "P2P: iterCount: "<<iterCount<<", deltaR: "<<deltaR<<", deltaT: "<<deltaT<<std::endl;

    if (iterCount > 2 && deltaR < 0.3 && deltaT < 0.01) {
        return true; // converged
    }
    return false; // keep optimizing
}

// nearest_ind = -1 when cannot find hash index
void CadRegistration::HashSearch(const pcl::PointSurfel p_source, int& nearest_ind, float& nearest_distance)
{
    nearest_ind = -1; // Can't find hash index
    nearest_distance = FLT_MAX;
    float cur_distance;
    bool b_find_nearest = false;

    int voxel_ind = GetVoxelIndex(p_source.x, p_source.y, p_source.z);

    if(voxel_ind < 0)
        return;

    if(vec_voxel_index_[voxel_ind].size() > 0){ 
        // When point exsist in same voxel
        for(auto point_index : vec_voxel_index_[voxel_ind])
        {
            cur_distance = sqrt((ioniq_cad_surfel_pcptr_->points[point_index].x - p_source.x) * (ioniq_cad_surfel_pcptr_->points[point_index].x - p_source.x) +
                                (ioniq_cad_surfel_pcptr_->points[point_index].y - p_source.y) * (ioniq_cad_surfel_pcptr_->points[point_index].y - p_source.y) +
                                (ioniq_cad_surfel_pcptr_->points[point_index].z - p_source.z) * (ioniq_cad_surfel_pcptr_->points[point_index].z - p_source.z));

            if (cur_distance < nearest_distance){
                nearest_distance = cur_distance;
                nearest_ind = point_index;
            }
        }

        i_direct_search_num_++;
    }
    else{
        // Search surround voxel
        std::vector<int> vec_sur_voxel_indices = GetSurroundVoxelIndices(voxel_ind);

        for(auto sur_voxel_ind : vec_sur_voxel_indices){
            for(auto point_index : vec_voxel_index_[sur_voxel_ind])
            {
                cur_distance = sqrt((ioniq_cad_surfel_pcptr_->points[point_index].x - p_source.x) * (ioniq_cad_surfel_pcptr_->points[point_index].x - p_source.x) +
                                    (ioniq_cad_surfel_pcptr_->points[point_index].y - p_source.y) * (ioniq_cad_surfel_pcptr_->points[point_index].y - p_source.y) +
                                    (ioniq_cad_surfel_pcptr_->points[point_index].z - p_source.z) * (ioniq_cad_surfel_pcptr_->points[point_index].z - p_source.z));

                if (cur_distance < nearest_distance){
                    nearest_distance = cur_distance;
                    nearest_ind = point_index;
                }
            }
        }

        i_rel_search_num_++;

        if(vec_sur_voxel_indices.size() == 0) i_not_searched_num_++;
    }

    return;
}

std::vector<int> CadRegistration::GetSurroundVoxelIndices(int cur_voxel_index)
{
    int i_cur_x, i_cur_y, i_cur_z;
    int i_sur_voxel_index;
    std::vector<int> vec_sur_voxel_indices;

    i_cur_z = static_cast<int>(cur_voxel_index / (i_hash_x_num_ * i_hash_y_num_)); 
    i_cur_y = static_cast<int>((cur_voxel_index - i_cur_z * i_hash_x_num_ * i_hash_y_num_) / i_hash_x_num_);
    i_cur_x = cur_voxel_index - i_cur_z * i_hash_x_num_ * i_hash_y_num_ - i_cur_y * i_hash_x_num_;

    for(int x_ind = i_cur_x - 1; x_ind <= i_cur_x + 1 ; x_ind++){
        for(int y_ind = i_cur_y - 1; y_ind <= i_cur_y + 1 ; y_ind++){
            for(int z_ind = i_cur_z - 1; z_ind <= i_cur_z + 1 ; z_ind++){
                if(x_ind >= 0 && x_ind < i_hash_x_num_ &&
                    y_ind >= 0 && y_ind < i_hash_y_num_ &&
                    z_ind >= 0 && z_ind < i_hash_z_num_){
                    
                    i_sur_voxel_index = (i_hash_x_num_*i_hash_y_num_) * z_ind + i_hash_x_num_ * y_ind + x_ind;
                    
                    if(vec_voxel_index_[i_sur_voxel_index].size() > 0){
                        vec_sur_voxel_indices.push_back(i_sur_voxel_index);
                    }
                }
            }
        }
    }

    
    return vec_sur_voxel_indices;
}

int CadRegistration::GetVoxelIndex(const float p_x, const float p_y, const float p_z)
{
    if(p_x > f_max_hash_x_m_ || p_x < f_min_hash_x_m_ ||
       p_y > f_max_hash_y_m_ || p_y < f_min_hash_y_m_ || 
       p_z > f_max_hash_z_m_ || p_z < f_min_hash_z_m_)
    {
        return -1;
    }

    return  (i_hash_x_num_*i_hash_y_num_) * static_cast<int>((p_z - f_min_hash_z_m_) / f_hash_grid_size_m_) + 
                static_cast<int>((p_y - f_min_hash_y_m_) / f_hash_grid_size_m_) * i_hash_x_num_ +
                static_cast<int>((p_x - f_min_hash_x_m_) / f_hash_grid_size_m_);
}

void CadRegistration::VoxelFilter(const pcl::PointCloud<pcl::PointXYZI> i_cloud, pcl::PointCloud<pcl::PointXYZI>& o_cloud)
{
    vec_cluster_exist.clear();
    vec_cluster_exist.resize(i_hash_total_num_);
    o_cloud.clear();

    int i_cloud_num = i_cloud.size();
    int voxel_ind;
    
    for(int i = 0; i < i_cloud_num ; i++)
    {
        voxel_ind = GetVoxelIndex(i_cloud[i].x, i_cloud[i].y, i_cloud[i].z);
        if (vec_cluster_exist[voxel_ind] == false){
            vec_cluster_exist[voxel_ind] = true;
            o_cloud.push_back(i_cloud[i]);
        }
    }
}

Eigen::Quaterniond CadRegistration::NormalToQuaternion(double pa, double pb, double pc) {
    // z-축 벡터
    Eigen::Vector3d z_axis(0, 0, 1);
    // 법선 벡터
    Eigen::Vector3d normal(pa, pb, pc);
    normal.normalize();  // 법선 벡터 정규화

    // 회전 축
    Eigen::Vector3d axis = z_axis.cross(normal);
    axis.normalize();  // 회전 축 정규화

    // 회전 각도
    double angle = acos(z_axis.dot(normal));

    // 쿼터니언 생성
    Eigen::Quaterniond quaternion;
    quaternion = Eigen::AngleAxisd(angle, axis);

    return quaternion;
}

std::array<int, 3> CadRegistration::ValueToRainbowColor(float value) {
    // RGB 값 범위를 확실히 0~255 내로 제한하기 위한 조건
    if (value < 0.0f) value = 0.0f;
    if (value > 1.0f) value = 1.0f;

    // 각 색상 영역에 대한 구분
    // 6개의 구간으로 나누어 각 구간에서의 변화를 계산
    float pos = value * 5.0f; // 0 ~ 6 범위
    int section = static_cast<int>(pos); // 구간 인덱스 0 ~ 5
    float fractional = pos - (float)section; // 구간 내 위치

    int rising = static_cast<int>(255.0 * fractional);   // 증가하는 값
    int falling = static_cast<int>(255.0 * (1 - fractional)); // 감소하는 값

    std::array<int, 3> rgb = {0, 0, 0};

    switch (section) {
        case 0:  // Red to Yellow (Red stays, Green rises)
            rgb = {255, rising, 0};
            break;
        case 1:  // Yellow to Green (Red falls, Green stays)
            rgb = {falling, 255, 0};
            break;
        case 2:  // Green to Cyan (Green stays, Blue rises)
            rgb = {0, 255, rising};
            break;
        case 3:  // Cyan to Blue (Green falls, Blue stays)
            rgb = {0, falling, 255};
            break;
        case 4:  // Blue to Magenta (Blue stays, Red rises)
            rgb = {rising, 0, 255};
            break;
    }

    return rgb;
}


void CadRegistration::UpdateJSKLidarObjects(const PointsObjects& objects) {
    jsk_recognition_msgs::BoundingBoxArray o_jsk_bbox_array;
    o_jsk_bbox_array.header.frame_id = "velodyne";
    o_jsk_bbox_array.header.stamp = ros::Time(objects.time_stamp);
    for (auto object : objects.object) {
        jsk_recognition_msgs::BoundingBox o_jsk_bbox;
        o_jsk_bbox.header.frame_id = "velodyne";

        if (object.state.time_stamp > 0) {
            o_jsk_bbox.header.stamp = ros::Time(object.state.time_stamp);
        }
        else {
            o_jsk_bbox.header.stamp = ros::Time(objects.time_stamp);
        }
        
        o_jsk_bbox.label = object.classification;
        o_jsk_bbox.dimensions.x = object.dimension.length;
        o_jsk_bbox.dimensions.y = object.dimension.width;
        o_jsk_bbox.dimensions.z = object.dimension.height;
        o_jsk_bbox.pose.position.x = object.state.x;
        o_jsk_bbox.pose.position.y = object.state.y;
        o_jsk_bbox.pose.position.z = object.state.z + object.dimension.height/2.0;

        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, object.state.yaw);
        myQuaternion = myQuaternion.normalize();
        o_jsk_bbox.pose.orientation.x = myQuaternion.getX();
        o_jsk_bbox.pose.orientation.y = myQuaternion.getY();
        o_jsk_bbox.pose.orientation.z = myQuaternion.getZ();
        o_jsk_bbox.pose.orientation.w = myQuaternion.getW();
        o_jsk_bbox_array.boxes.push_back(o_jsk_bbox);
    }
    o_jsk_lidar_objects_ = o_jsk_bbox_array;
}

void CadRegistration::UpdatePointsObjectsPc(const PointsObjects& point_objects)
{   
    pcl::PointCloud<pcl::PointXYZI> merged_pc;

    for(const auto& obj : point_objects.object){
        
        pcl::PointCloud<pcl::PointXYZI> global_points;
        double cos_yaw = cos(obj.state.yaw);
        double sin_yaw = sin(obj.state.yaw);
        
        // convert Box Coordinate to LiDAR Coordinate
        for(const auto& point : obj.points){
            pcl::PointXYZI iter_point;
            iter_point.x = cos_yaw * point.x - sin_yaw * point.y + obj.state.x; 
            iter_point.y = sin_yaw * point.x + cos_yaw * point.y + obj.state.y;
            iter_point.z = point.z + obj.state.z;
            iter_point.intensity = point.intensity;
            global_points.points.push_back(iter_point);
        }

        merged_pc += global_points;
    }

    pcl::toROSMsg(merged_pc, o_points_objects_pc_);
    o_points_objects_pc_.header.stamp = ros::Time(point_objects.time_stamp);
    o_points_objects_pc_.header.frame_id = "velodyne";

    pcl::toROSMsg(*o_refined_cad_surfel_pcptr_, o_refined_cad_pc_);
    o_refined_cad_pc_.header.stamp = ros::Time(point_objects.time_stamp);
    o_refined_cad_pc_.header.frame_id = "velodyne";
}

int main(int argc, char** argv) {
    std::string node_name = "cad_registration";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_INFO("Initialize node, get parameters...");

    double period;
    if (!nh.getParam("/cad_registration/period_box_point_generator", period))
        period = 0.1;
    
    if (period <= 0.0) period = 0.1;

    ROS_INFO("Complete to get parameters! (Period: %.3f)", period);

    CadRegistration cad_registration;

    ros::Rate loop_rate(1.0 / period); // Period(sec) -> Frequency(Hz)

    while (ros::ok()) {
        ros::spinOnce();
        cad_registration.Run();
        cad_registration.Publish();
        loop_rate.sleep();
    }

    return 0;
}