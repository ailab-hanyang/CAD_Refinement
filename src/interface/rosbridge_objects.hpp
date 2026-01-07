/****************************************************************************/
// Module:      rosbridge_objects.hpp
// Description: ROS bridge for objects
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
//      Jan 5, 2026: Soyeong Kim - Remove ailab dependencies and clean up code
/****************************************************************************/

#ifndef __ROSBRIDGE_OBJECTS__
#define __ROSBRIDGE_OBJECTS__
#pragma once

// ROS header
#include <ros/ros.h>
#include <tf/tf.h>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS Message Header
#include <sensor_msgs/PointCloud2.h>

// ROS Message Header
#include <hyu_msgs/DetectObjects.h>
#include <hyu_msgs/PointsObjects.h>

// Interface Header
#include "rosbridge_time.hpp"
#include "interface_objects.hpp"

using namespace hyu_types;

namespace hyu_functions{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // Get functions
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    DetectObjects GetDetectObjects(const hyu_msgs::DetectObjects& msg) {
        hyu_msgs::DetectObjects i_objects = msg;

        DetectObjects objects;
        objects.time_stamp = GetTimeStamp(i_objects.header.stamp);
        for (auto i_object : i_objects.object) {
            DetectObject object;
            object.id = i_object.id;
            object.detection_confidence = i_object.detection_confidence;

            object.classification = (ObjectClass)i_object.classification;
            object.dynamic_state = (ObjectDynamicState)i_object.dynamic_state;
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            object.state.time_stamp = GetTimeStamp(i_object.state.header.stamp);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }  

    template<typename pointType>
    PointsObjects GetPointsObjects(const hyu_msgs::PointsObjects& msg) {
        hyu_msgs::PointsObjects i_objects = msg;

        PointsObjects objects;
        objects.time_stamp = GetTimeStamp(i_objects.header.stamp);
        for (auto i_object : i_objects.object) {
            PointsObject object;
            pcl::PointCloud<pointType> point_cloud;

            object.id = i_object.id;
            object.detection_confidence = i_object.detection_confidence;

            object.classification = (ObjectClass)i_object.classification;
            object.dynamic_state = (ObjectDynamicState)i_object.dynamic_state;
            
            object.dimension.length = i_object.dimension.length;
            object.dimension.width = i_object.dimension.width;
            object.dimension.height = i_object.dimension.height;

            pcl::fromROSMsg(i_object.points, point_cloud);
            object.points = point_cloud;

            object.state.time_stamp = GetTimeStamp(i_object.state.header.stamp);
            object.state.x = i_object.state.x;
            object.state.y = i_object.state.y;
            object.state.z = i_object.state.z;
            object.state.roll = i_object.state.roll;
            object.state.pitch = i_object.state.pitch;
            object.state.yaw = i_object.state.yaw;
            object.state.v_x = i_object.state.v_x;
            object.state.v_y = i_object.state.v_y;
            object.state.v_z = i_object.state.v_z;
            object.state.a_x = i_object.state.a_x;
            object.state.a_y = i_object.state.a_y;
            object.state.a_z = i_object.state.a_z;
            object.state.roll_rate = i_object.state.roll_rate;
            object.state.pitch_rate = i_object.state.pitch_rate;
            object.state.yaw_rate = i_object.state.yaw_rate;

            objects.object.push_back(object);
        }

        return objects;
    }  
} // namespace hyu_functions

#endif  // __ROSBRIDGE_OBJECTS__