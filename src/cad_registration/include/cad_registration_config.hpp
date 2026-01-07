#ifndef _CAD_REGISTRATION_CONFIG_HPP__
#define _CAD_REGISTRATION_CONFIG_HPP__
#pragma once

using namespace std;

typedef struct {
    int detection_mode; // 0: both, 1: only lidar, 2: only pillar
    double merge_iou_threshold;
    double lidar_wait_time_sec;
    double frenet_boundary_margin_m;
    int fusion_method;
    int angle_based_time_compensation;
    double lidar_single_scan_time;
} CadRegistrationParams;

#endif  // _CAD_REGISTRATION_CONFIG_HPP__