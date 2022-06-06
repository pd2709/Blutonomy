// slam_geometry.hpp
// Helper class containig useful geometric transformations
#pragma once

#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Unit3.h"
#include "gtsam/geometry/Quaternion.h"
#include "gtsam/inference/Symbol.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"


constexpr double DEG2RAD = M_PI / 180;

// ! Loose terminology of "sensor" and "vehicle". Need to eventually be careful to distinguish sonar frame and IMU frame

namespace slam_geometry{
    
    /**
     * Store a range and bearing 
     */
    struct RangeBearingMeasurement
    {
        RangeBearingMeasurement(){};
        RangeBearingMeasurement(double range, double bearing, double elevation):range(range),bearing(bearing),elevation(elevation){};
        double range = 0;
        double bearing = 0;
        double elevation = 0;

        // Symbolic pose from which measurement was taken
        gtsam::Symbol measured_from_symbol = 0;

        friend std::ostream &operator<<(std::ostream &os, const RangeBearingMeasurement& meas)
        {
            os << meas.bearing << ", " << meas.range;
            return os;
        }

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
     * Convert a RangeBearingMeasurement to a GTSAM Unit3, double pair
     * @param  {RangeBearingMeasurement} meas : The RangeBearingMeasurement
     * @return {std::pair<gtsam::Unit3,}      : Converted bearing on a unit sphere and range to feature
     */
    std::pair<gtsam::Unit3, double> rangeBearingMeasurement2gtsam(RangeBearingMeasurement meas);

    /**
     * Inverse prediction function for 2D sonar frame into 3D world
     * The quaternion used to construct the input pose MUST be normalized!
     * @param  {gtsam::Pose3} vehicle_world          : Pose of vehicle in world frame
     * @param  {RangeBearingMeasurement} measurement : Measurement of feature in vehicle frame
     * @return {gtsam::Point3}                       : Location of feature in world frame
     */
    gtsam::Point3 prediction_inv(gtsam::Pose3 vehicle_world, RangeBearingMeasurement measurement);

    /**
     * Plot local polar measurements in sonar midplane
     * @param  {cv::Mat} image                                        : 
     * @param  {std::vector<RangeBearingMeasurement>} measurements_local : Polar measurements made in the frame of the sonar sensor
     */
    void plotLocalInLocal(cv::Mat &image, std::vector<RangeBearingMeasurement> measurements_local, cv::Scalar color);

    /**
     * Plot landmark points in global frame projected back into sonar midplane 
     * @param  {cv::Mat} image                               : 
     * @param  {gtsam::Pose3} sonar_world                    : Pose of sonar sensor to project into
     * @param  {std::vector<gtsam::Point3>} landmarks_global : 3D lanmark points to project
     */
    void plotGlobalInLocal(cv::Mat &image, gtsam::Pose3 vehicle_world, std::vector<gtsam::Point3> landmarks_global);

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