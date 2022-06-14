#pragma once

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Quaternion.h"
#include "gtsam/inference/Symbol.h"

/*
Helper containing useful geometric transforms
*/

// ! Loose terminology of "sensor" and "vehicle". Need to eventually be careful to distinguish sonar frame and IMU frame

namespace slam_geometry{
    
    /**
     * Store a range and bearing 
     */
    struct RangeBearingMeasurement
    {
        double range = 0;
        double bearing = 0;
        double elevation = 0;

        // Symbolic pose from which measurement was taken
        gtsam::Symbol measured_from_symbol = 0;
    };
    
    /**
     * Prediction function for 3D landmark into 2D sonar frame
     * Elevation also returned for testing purposes
     * The quaternion used to construct the input pose MUST be normalized!
     * @param  {gtsam::Pose3} vehicle_world    : Pose of vehicle in world frame
     * @param  {gtsam::Point3} landmark_world  : Position of landmark in world frrame
     * @return {slam_geometry::RangeBearingMeasurement} : 2D measurement
     */
    RangeBearingMeasurement prediction(gtsam::Pose3 vehicle_world, gtsam::Point3 landmark_world, boost::optional<gtsam::Matrix&> dz_dq = boost::none, boost::optional<gtsam::Matrix&> dq_dx = boost::none);

    /**
     * Inverse prediction function for 2D sonar frame into 3D world
     * The quaternion used to construct the input pose MUST be normalized!
     * @param  {gtsam::Pose3} vehicle_world          : Pose of vehicle in world frame
     * @param  {RangeBearingMeasurement} measurement : Measurement of feature in vehicle frame
     * @return {gtsam::Point3}                       : Location of feature in world frame
     */
    gtsam::Point3 prediction_inv(gtsam::Pose3 vehicle_world, RangeBearingMeasurement measurement);

    /**
     * Predict the optimal elevation angle of an observed feature based on known height assumption
     * @param  {gtsam::Pose3} sensor_in_world : Pose of sensor in world frame
     * @param  {double} bearing               : Bearing of feature in sensor frame
     * @param  {double} range                 : Range of feature in sensor frame
     * @param  {double} floor_z_rel_world     : Z-coordinate of floor in world frame
     * @return {double}                       : Optimal elevation (rad)
     */
    double predictOptimalElevation(gtsam::Pose3 sensor_in_world, double bearing, double range, double floor_z_rel_world);
}