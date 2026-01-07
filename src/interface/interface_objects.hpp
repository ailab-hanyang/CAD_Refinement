/****************************************************************************/
// Module:      interface_objects.hpp
// Description: objects interface
//
// Authors: Yuseung Na (ys.na0220@gmail.com)
// Version: 0.1
//
// Revision History
//      June 14, 2023: Yuseung Na - Created.
//      Jan 5, 2026: Soyeong Kim - Remove ailab dependencies and clean up code
/****************************************************************************/

#ifndef __INTERFACE_OBJECTS_HPP__
#define __INTERFACE_OBJECTS_HPP__
#pragma once

// STD Header
#include <stdint.h>
#include <map>
#include <utility>
#include <vector>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hyu_types {
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // enum
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //     
    typedef enum {
        UNKNOWN = 0,
        CAR,
        BARRIER,
        SIGN
    } ObjectClass;

    typedef enum {
        UNKNOWN_STATE = 0,
        STATIC = 1,
        DYNAMIC = 2,
    } ObjectDynamicState;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
    // structs
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

    typedef struct {
        double time_stamp{0.0};
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
        double v_x;
        double v_y;
        double v_z;
        double a_x;
        double a_y;
        double a_z;
        double roll_rate;
        double pitch_rate;
        double yaw_rate;
    } ObjectState;

    typedef struct {
        double length;
        double width;
        double height;
    } ObjectDimension;

    typedef struct {  
        unsigned int id;
        float detection_confidence;

        ObjectClass classification;
        ObjectDynamicState dynamic_state;

        ObjectDimension dimension;
        
        ObjectState state;
    } DetectObject;

    typedef struct {        
        double time_stamp{0.0};
        std::vector<DetectObject> object;
    } DetectObjects;

    typedef struct {  
        unsigned int id;
        float detection_confidence;
        bool is_behind;

        ObjectClass classification;
        ObjectDynamicState dynamic_state;

        ObjectDimension dimension;
        
        ObjectState state;

        pcl::PointCloud<pcl::PointXYZI> points;
        
    } PointsObject;
    
    typedef struct {        
        double time_stamp{0.0};
        std::vector<PointsObject> object;
    } PointsObjects;
}

#endif // __INTERFACE_OBJECTS_HPP__
