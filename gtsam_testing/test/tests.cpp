#include "../LandmarkFactor.hpp"
#include "../GeometricTransforms.hpp"
#include "../SemiParametricLandmark.hpp"
#include <gtest/gtest.h>

#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/base/ThreadsafeException.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"

using namespace gtsam;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

constexpr double RAD2DEG = 180.0/M_PI;
constexpr double DEG2RAD = M_PI/180.0;

TEST(LandmarkTest, create_empty)
{
    LandmarkFactor factor;
    ASSERT_EQ(factor.size(), 0);
}

TEST(LandmarkTest, create_one_stores_measurement)
{   
    // The range and bearing 
    RangeBearingMeasurement meas;
    meas.range = 1;
    meas.bearing = M_PI/4;
    meas.measured_from_symbol = X(0);

    std::vector<RangeBearingMeasurement> measurements;
    measurements.push_back(meas);

    // The landmark symbol
    Symbol landmark_symbol(L(0));
    Symbol base_symbol(X(0));

    // The noise model. Should be a 2x2 matrix for one measuement
    // sigma bearing, sigma range
    auto noise_model = noiseModel::Diagonal::Sigmas(Vector::Ones(2)*0.1);

    // Create the factor
    std::list<Key> keys{X(0), L(0)};
    LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol, keys);

    // Retrieve and check
    RangeBearingMeasurement meas_recovered = factor.getMeasurement(base_symbol);
    ASSERT_EQ(meas_recovered.range, 1);
    ASSERT_EQ(meas_recovered.bearing, M_PI/4);
}

TEST(LandmarkTest, create_multiple_stores_measurement)
{   
    // The range and bearing 
    RangeBearingMeasurement meas1;
    meas1.range = sqrt(2);
    meas1.bearing = M_PI/4;
    meas1.measured_from_symbol = X(0);
    Symbol base_symbol = meas1.measured_from_symbol;

    RangeBearingMeasurement meas2;
    meas2.range = 1;
    meas2.bearing = M_PI/2;
    meas2.measured_from_symbol = X(1);

    RangeBearingMeasurement meas3;
    meas3.range = sqrt(2);
    meas3.bearing = 3*(M_PI/4);
    meas3.measured_from_symbol = X(2);

    std::vector<RangeBearingMeasurement> measurements;
    measurements.push_back(meas1);
    measurements.push_back(meas2);
    measurements.push_back(meas3);

    // The landmark symbol
    Symbol landmark_symbol(L(0));

    // The noise model. Should be a 6x6 matrix for three measuements
    // sigma bearing, sigma range
    auto noise_model = noiseModel::Diagonal::Sigmas(Vector::Ones(6)*0.1);

    // Create the factor
    std::list<Key> keys{X(0), X(1), X(2), L(0)};
    LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol, keys);

    // Retrieve and check
    RangeBearingMeasurement meas_recovered_1 = factor.getMeasurement(X(0));
    ASSERT_NEAR(meas_recovered_1.range, 1.414, 0.01);
    ASSERT_NEAR(meas_recovered_1.bearing, 0.785, 0.01);

    RangeBearingMeasurement meas_recovered_2 = factor.getMeasurement(X(1));
    ASSERT_NEAR(meas_recovered_2.range, 1, 0.01);
    ASSERT_NEAR(meas_recovered_2.bearing, 1.571, 0.01);

    RangeBearingMeasurement meas_recovered_3 = factor.getMeasurement(X(2));
    ASSERT_NEAR(meas_recovered_3.range, 1.414, 0.01);
    ASSERT_NEAR(meas_recovered_3.bearing, 2.356, 0.01);
}

/*
TransformTest suite used to verify the GTSAM transform convention
*/
TEST(TransformTest, trans_world2loc_2d)
{
    // Create a point in world coords
    // Point at coordinates (5, 5, 0)
    Point3 landmark_world(5, 5, 0);
    
    // Create a vehicle pose in world coords
    // Vehicle at coordinates (2, 2, 0) facing along x-axis
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(2, 2, 0));

    // Transform landmark to vehicle frame 
    // Expect landmark to be at (3, 3, 0) in vehicle frame
    Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world);

    ASSERT_NEAR(landmark_vehicle.x(), 3, 0.001);
    ASSERT_NEAR(landmark_vehicle.y(), 3, 0.001);
    ASSERT_NEAR(landmark_vehicle.z(), 0, 0.001);

}

TEST(TransformTest, trans_world2loc_3d)
{
    // Create a point in world coords
    // Point at coordinates (3, 6, 10)
    Point3 landmark_world(3, 6, 10);
    
    // Create a vehicle pose in world coords
    // Vehicle at coordinates (5, -1, -8) facing along y-axis
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(5, -1, -8));

    // Transform landmark to vehicle frame 
    // Expect landmark to be at (-2, 7, 18) in vehicle frame
    Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world);

    ASSERT_NEAR(landmark_vehicle.x(), -2, 0.001);
    ASSERT_NEAR(landmark_vehicle.y(), 7, 0.001);
    ASSERT_NEAR(landmark_vehicle.z(), 18, 0.001);

}

TEST(TransformTest, trans_world2loc_2d_rotated)
{
    // Create a point in world coords
    // Point at coordinates (5, 5, 0)
    Point3 landmark_world(5, 5, 0);
    
    // Create a vehicle pose in world coords
    // Vehicle at coordinates (2, 2, 0) facing UPSIDE DOWN along x-axis
    Pose3 vehicle_world(Rot3(0, 1, 0, 0), Point3(2, 2, 0));

    // Transform landmark to vehicle frame 
    // Expect landmark to be at (3, 3, 0) in vehicle frame
    Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world);

    ASSERT_NEAR(landmark_vehicle.x(), 3, 0.001);
    ASSERT_NEAR(landmark_vehicle.y(), -3, 0.001);
    ASSERT_NEAR(landmark_vehicle.z(), 0, 0.001);

}

TEST(TransformTest, trans_world2loc_3d_rotated_origin)
{
    // Generic rotation about all axes
    // Reference value generated in MATLAB
    // Create a point in world coords
    Point3 landmark_world(-1, 0, 20);
    
    // Create a vehicle pose in world coords
    //! If the input quaternion is not normalized, the test fails
    Pose3 vehicle_world(Rot3(Quaternion(0.563, 0.563, 0.225, 0.563).normalized()), Point3(0, 0, 0));

    // Transform landmark to vehicle frame 
    // Expect landmark to be at (3, 3, 0) in vehicle frame
    Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world);

    ASSERT_NEAR(landmark_vehicle.x(), 7.3342, 0.01);
    ASSERT_NEAR(landmark_vehicle.y(), 18.0986, 0.01);
    ASSERT_NEAR(landmark_vehicle.z(), 4.4328, 0.01);

}

TEST(TransformTest, trans_world2loc_3d_rotated_general)
{
    // Generic rotation about all axes
    // Reference value generated in MATLAB
    // Create a point in world coords
    Point3 landmark_world(-1, 0, 20);
    
    // Create a vehicle pose in world coords
    Pose3 vehicle_world(Rot3(Quaternion(0.563, 0.563, 0.225, 0.563).normalized()), Point3(20, -5, 6));

    // Transform landmark to vehicle frame 
    // Expect landmark to be at (3, 3, 0) in vehicle frame
    Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world);

    ASSERT_NEAR(landmark_vehicle.x(), 4.1651, 0.01);
    ASSERT_NEAR(landmark_vehicle.y(), 19.0535, 0.01);
    ASSERT_NEAR(landmark_vehicle.z(), -16.7815, 0.01);

}

TEST(PredictionTest, simple)
{

    // Create a point in world coords
    // Point at coordinates (5, 0, 0)
    Point3 landmark_world(5, 0, 0);
    
    // Create a vehicle pose in world coords
    // Vehicle at coordinates (2, 2, 0) facing along x-axis
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 5, 0.01);
    ASSERT_NEAR(meas.bearing, 0, 0.01);
}

TEST(PredictionTest, simple_offAxis)
{

    // Point in-plane but slighty off to one side
    Point3 landmark_world(5, 5, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 7.071, 0.01);
    ASSERT_NEAR(meas.bearing, M_PI_4, 0.01);
}

TEST(PredictionTest, simple_offAxis_neg)
{

    // Point off-plane in opposite direction
    Point3 landmark_world(5, -5, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 7.071, 0.01);
    ASSERT_NEAR(meas.bearing, -M_PI_4, 0.01);
}

TEST(PredictionTest, simple_offAxis_both)
{

    // Point off-plane at nonzero elevation
    Point3 landmark_world(5, 5, 5);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 8.660, 0.01);
    ASSERT_NEAR(meas.bearing, M_PI_4, 0.01);
}

TEST(PredictionTest, simple_offAxis_both_neg)
{

    // Point off-plane at nonzero elevation
    Point3 landmark_world(5, -5, 5);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 8.660, 0.01);
    ASSERT_NEAR(meas.bearing, -M_PI_4, 0.01);
}

TEST(PredictionTest, translated_x)
{

    // Translated vehicle
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(-5, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 10, 0.01);
    ASSERT_NEAR(meas.bearing, 0, 0.01);
}

TEST(PredictionTest, translated_y)
{

    // Translated vehicle
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 5, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 7.07, 0.01);
    ASSERT_NEAR(meas.bearing, -M_PI_4, 0.01);
}

TEST(PredictionTest, translated_z)
{

    // Translated vehicle
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, -5));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 7.07, 0.01);
    ASSERT_NEAR(meas.bearing, 0, 0.01);
}

TEST(PredictionTest, rotated_z)
{

    // Rotated vehicle 45 degrees CCW about y-axis
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(0.924, 0, 0, 0.383), Point3(0, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 5, 0.01);
    ASSERT_NEAR(meas.bearing, -M_PI_4, 0.01);
}

TEST(PredictionTest, rotated_z_translated_x)
{

    // Rotated vehicle 45 degrees CCW about z-axis and translated
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(0.924, 0, 0, 0.383), Point3(-5, 0, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 10, 0.01);
    ASSERT_NEAR(meas.bearing, -M_PI_4, 0.01);
}

TEST(PredictionTest, rotated_z_translated_y)
{

    // Rotated vehicle 45 degrees CCW about z-axis and translated
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(0.924, 0, 0, 0.383), Point3(0, -5, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 7.07, 0.01);
    ASSERT_NEAR(meas.bearing, 0, 0.01);
}

TEST(PredictionTest, rotated_z_translated_xy)
{

    // Rotated vehicle 45 degrees CW about z-axis and translated
    Point3 landmark_world(5, 0, 0);
    Pose3 vehicle_world(Rot3(0.924, 0, 0, -0.383), Point3(-3, -3, 0));

    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    ASSERT_NEAR(meas.range, 8.544, 0.01);
    ASSERT_NEAR(meas.bearing, 65.566*(M_PI/180.0), 0.01);
}

TEST(PredictionInverseTest, simple)
{
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));
    slam_geometry::RangeBearingMeasurement meas;
    meas.range = 1;
    meas.bearing = 0;
    meas.elevation = 0;
    gtsam::Point3 landmark_world = slam_geometry::prediction_inv(vehicle_world, meas);

    ASSERT_NEAR(landmark_world.x(), 1, 0.01);
    ASSERT_NEAR(landmark_world.y(), 0, 0.01);
    ASSERT_NEAR(landmark_world.z(), 0, 0.01);
}

TEST(PredictionInverseTest, simple_offAxis)
{
    Point3 landmark_world(1, 1, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, simple_offAxis_both)
{
    Point3 landmark_world(1, 1, 2);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, simple_offAxis_both_neg)
{
    Point3 landmark_world(-1, 1, -2);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, 0));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, translated_x)
{
    Point3 landmark_world(0, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(1, 0, 0));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, translated_y)
{
    Point3 landmark_world(0, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 3, 0));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, translated_z)
{
    Point3 landmark_world(0, 0, 0);
    Pose3 vehicle_world(Rot3(1, 0, 0, 0), Point3(0, 0, -2));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, rot_trans_1)
{
    Point3 landmark_world(3, 4, -1);
    Pose3 vehicle_world(Rot3(Quaternion(0.70, 0, 0.70, 0).normalized()), Point3(1, 0, -2));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, rot_trans_2)
{
    Point3 landmark_world(5, -4, -1);
    Pose3 vehicle_world(Rot3(Quaternion(0.5, 0.3, -0.2, 0.1).normalized()), Point3(-10, 8, -2));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

TEST(PredictionInverseTest, rot_trans_3)
{
    Point3 landmark_world(10, -40, -21);
    Pose3 vehicle_world(Rot3(Quaternion(-0.1, 0.3, 0.3, -0.9).normalized()), Point3(50, 11, -1));
    
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(vehicle_world, landmark_world);

    // Apply inverse
    gtsam::Point3 landmark_world_recovered = slam_geometry::prediction_inv(vehicle_world, meas);

    // Check equality
    ASSERT_NEAR(landmark_world.x(), landmark_world_recovered.x(), 0.01);
    ASSERT_NEAR(landmark_world.y(), landmark_world_recovered.y(), 0.01);
    ASSERT_NEAR(landmark_world.z(), landmark_world_recovered.z(), 0.01);
}

/**
 * Semi-parametric landmark testing
 **/

// TEST(SemiParametricLandmarkTest, dimension)
// {
//     // Check that it's 2D
//     SemiParametricLandmark landmark(0, 1);
//     ASSERT_EQ(landmark.dim(), 2);
// }

// TEST(SemiParametricLandmarkTest, create_zero_bearing)
// {
//     double bearing = 0;
//     double range = 0.00001;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), bearing, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_small_1)
// {
//     double bearing = 0.01;
//     double range = 0.2;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), bearing, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_small_2)
// {
//     double bearing = -M_PI - 0.1;
//     double range = 3;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), M_PI-0.1, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_small_3)
// {
//     double bearing = M_PI + 0.1;
//     double range = 4.1;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), -M_PI+0.1, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_big_1)
// {
//     double bearing = 6*M_PI + 0.1;
//     double range = 100;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), 0.1, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_big_2)
// {
//     double bearing = -5*M_PI + 0.1;
//     double range = 1000;
//     SemiParametricLandmark landmark(bearing, range);
//     ASSERT_NEAR(landmark.getBearing(), -M_PI+0.1, 0.0001);
//     ASSERT_NEAR(landmark.getRange(), range, 0.0001);
// }

// TEST(SemiParametricLandmarkTest, create_illegal_1)
// {
//     // Cannot have negative range
//     double bearing = M_PI_4;
//     double range = -1;
//     ASSERT_THROW(SemiParametricLandmark landmark(bearing, range), gtsam::OutOfRangeThreadsafe);
    
// }

// constexpr double retract_eps = 0.0001;

// // Retraction from tangent space onto manifold
// TEST(SemiParametricLandmarkTest, retract_zero)
// {
//     Vector2 delta(0, 0);
//     SemiParametricLandmark landmark(1, 1);

//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 1, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 1, retract_eps);
// }

// // Retraction from tangent space onto manifold
// // x goes with psi, y goes with r
// TEST(SemiParametricLandmarkTest, retract_small_y)
// {
//     double dy = 0.01;
//     double dx = 0;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 1.01, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 1, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_small_y_neg)
// {
//     double dy = -0.02;
//     double dx = 0;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 0.98, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 1, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_small_x)
// {
//     double dy = 0.0;
//     double dx = 0.01;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 1, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 1.01, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_small_x_neg)
// {
//     double dy = 0.0;
//     double dx = -0.02;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 1, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 0.979998666, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_small_2d)
// {
//     double dy = 0.04;
//     double dx = -0.03;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 1.04, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 0.9699954, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_big_2d)
// {
//     double dy = 1;
//     double dx = 1;
//     double l_psi = 1;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 2, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 2.570796327, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_big_2d_wrapped)
// {
//     double dy = 8;
//     double dx = -0.1; // asin(dx) = -0.100167
//     double l_psi = -M_PI+0.01; // -3.131592
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 9, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 3.051426, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_cross_zero)
// {
//     double dy = 2;
//     double dx = 0.1;
//     double l_psi = -0.05;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     SemiParametricLandmark retracted = landmark.retract(delta);

//     ASSERT_NEAR(retracted.getRange(), 3, retract_eps);
//     ASSERT_NEAR(retracted.getBearing(), 0.0501674, retract_eps);
// }

// TEST(SemiParametricLandmarkTest, retract_illegal_range)
// {
//     double dy = -2;
//     double dx = +0.1;
//     double l_psi = -0.01;
//     double l_r = 1;

//     Vector2 delta(dx, dy);
//     SemiParametricLandmark landmark(l_psi, l_r);
//     ASSERT_THROW(landmark.retract(delta), gtsam::OutOfRangeThreadsafe);
// }

// constexpr double local_eps = 0.0001;

// // Test the local mapping (from manifold to tangent space)
// TEST(SemiParametricLandmarkTest, local_zero)
// {

//     // Other vector
//     double l_psi = 0;
//     double l_r = 1;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 1;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0, local_eps);
//     ASSERT_NEAR(delta[1], 0, local_eps);
// }

// // Test the local mapping (from manifold to tangent space)
// TEST(SemiParametricLandmarkTest, local_small_psi)
// {
//     // Other vector
//     double l_psi = 0.01;
//     double l_r = 1;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 1;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0.0099998, local_eps);
//     ASSERT_NEAR(delta[1], 0, local_eps);
// }

// TEST(SemiParametricLandmarkTest, local_small_psi_neg)
// {
//     // Other vector
//     double l_psi = 0.9;
//     double l_r = 1;

//     // Base vector
//     double l_psi_1 = 1;
//     double l_r_1 = 1;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], -0.0998334, local_eps);
//     ASSERT_NEAR(delta[1], 0, local_eps);
// }

// TEST(SemiParametricLandmarkTest, local_small_r)
// {
//     // Other vector
//     double l_psi = 0;
//     double l_r = 1.2;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 1;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0, local_eps);
//     ASSERT_NEAR(delta[1], 0.2, local_eps);
// }

// TEST(SemiParametricLandmarkTest, local_small_r_neg)
// {
//     // Other vector
//     double l_psi = 0;
//     double l_r = 0.8;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 1;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0, local_eps);
//     ASSERT_NEAR(delta[1], -0.2, local_eps);
// }

// TEST(SemiParametricLandmarkTest, local_small_both)
// {
//     // Other vector
//     double l_psi = 2.4;
//     double l_r = 5;

//     // Base vector
//     double l_psi_1 = 2;
//     double l_r_1 = 6;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0.3894183, local_eps);
//     ASSERT_NEAR(delta[1], -1.0, local_eps);
// }

// TEST(SemiParametricLandmarkTest, local_bearing_wrap)
// {
//     // Other vector
//     double l_psi = M_PI+0.1;
//     double l_r = 5;

//     // Base vector
//     double l_psi_1 = M_PI-0.1;
//     double l_r_1 = 5;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     Vector2 delta = landmark_base.localCoordinates(landmark_other);

//     ASSERT_NEAR(delta[0], 0.1986693, local_eps);
//     ASSERT_NEAR(delta[1], 0, local_eps);
// }

// TEST(SemiParametricLandmarkTest, print)
// {
//     SemiParametricLandmark landmark(0, 1);
//     ASSERT_NO_THROW(landmark.print());
// }

// TEST(SemiParametricLandmarkTest, equal_1)
// {
//     // Other vector
//     double l_psi = 0;
//     double l_r = 5;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 5;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     ASSERT_TRUE(landmark_base.equals(landmark_other));
// }

// TEST(SemiParametricLandmarkTest, equal_2)
// {
//     // Other vector
//     double l_psi = 0.3;
//     double l_r = 5;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 5;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     ASSERT_FALSE(landmark_base.equals(landmark_other));
// }

// TEST(SemiParametricLandmarkTest, equal_3)
// {
//     // Other vector
//     double l_psi = 0;
//     double l_r = 3;

//     // Base vector
//     double l_psi_1 = 0;
//     double l_r_1 = 5;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     ASSERT_FALSE(landmark_base.equals(landmark_other));
// }

// TEST(SemiParametricLandmarkTest, equal_4)
// {
//     // Other vector
//     double l_psi = M_PI+0.1;
//     double l_r = 3;

//     // Base vector
//     double l_psi_1 = 3*M_PI+0.1;
//     double l_r_1 = 3;

//     SemiParametricLandmark landmark_base(l_psi_1, l_r_1);
//     SemiParametricLandmark landmark_other(l_psi, l_r);

//     ASSERT_TRUE(landmark_base.equals(landmark_other));
// }

// TEST(CreateValues, simple)
// {
//     Values val;
//     ASSERT_NO_THROW(val.insert(0, SemiParametricLandmark(0, 1)));
// }

// TEST(UnwhitenedError, no_base)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at the origin

//     // The range and bearing
//     RangeBearingMeasurement meas;
//     meas.range = 1;
//     meas.bearing = 0;
//     meas.measured_from_symbol = X(0);
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0, 1);

//     // The base pose variable
//     Symbol base_symbol(X(0));
//     Pose3 base_var;

//     // The noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

//     // Create the factor
//     std::list<Key> keys{X(0), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol, keys);

//     // Create the values
//     Values val;
//     val.insert(landmark_symbol, landmark_var);
//     // val.insert(base_symbol, base_var); // Intentionally omitted for test case

//     ASSERT_THROW(factor.unwhitenedError(val), gtsam::InvalidArgumentThreadsafe);
// }

// TEST(UnwhitenedError, no_landmark)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at the origin

//     // The range and bearing
//     RangeBearingMeasurement meas;
//     meas.range = 1;
//     meas.bearing = 0;
//     meas.measured_from_symbol = X(0);
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0, 1);

//     // The base pose variable
//     Symbol base_symbol(X(0));
//     Pose3 base_var;

//     // The noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

//     // Create the factor
//     std::list<Key> keys{X(0), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol,keys);

//     // Create the values
//     Values val;
//     // val.insert(landmark_symbol, landmark_var);
//     val.insert(base_symbol, base_var); // Intentionally omitted for test case

//     ASSERT_THROW(factor.unwhitenedError(val), gtsam::InvalidArgumentThreadsafe);
// }

// constexpr double unwhitened_eps = 0.0001;

// TEST(UnwhitenedError, single_meas_equal)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at the origin

//     // The measured range and bearing
//     RangeBearingMeasurement meas;
//     meas.range = 1;
//     meas.bearing = 0;
//     meas.measured_from_symbol = X(0);
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0, 1);

//     // The base pose variable
//     Symbol base_symbol(X(0));
//     Pose3 base_var;

//     // The noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

//     // Create the factor
//     std::list<Key> keys{X(0), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol, keys);

//     // Create the values
//     Values val;
//     val.insert(base_symbol, base_var);
//     val.insert(landmark_symbol, landmark_var);

//     // Compute the error on the un-optimised factor and check
//     Vector err = factor.unwhitenedError(val);
//     ASSERT_EQ(err.size(), 2);
//     ASSERT_NEAR(err[0], 0, unwhitened_eps);
//     ASSERT_NEAR(err[1], 0, unwhitened_eps);
// }

// TEST(UnwhitenedError, single_meas_not_equal)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at the origin, with some error

//     // The measured range and bearing
//     RangeBearingMeasurement meas;
//     meas.bearing = 0.1;
//     meas.range = 0.9;
//     meas.measured_from_symbol = X(0);
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0, 1);

//     // The base pose variable
//     Symbol base_symbol(X(0));
//     Pose3 base_var;

//     // The noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

//     // Create the factor
//     std::list<Key> keys{X(0), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, base_symbol, keys);

//     // Create the values
//     Values val;
//     val.insert(base_symbol, base_var);
//     val.insert(landmark_symbol, landmark_var);

//     // Compute the error on the un-optimised factor and check
//     // Remember that order is bearing then range
//     Vector err = factor.unwhitenedError(val);
//     ASSERT_EQ(err.size(), 2);
//     ASSERT_NEAR(err[0], -0.1, unwhitened_eps);
//     ASSERT_NEAR(err[1], 0.1, unwhitened_eps);
// }

// TEST(UnwhitenedError, two_meas_exact)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at two poses, with some error
//     // The vehicle moves one unit in the -y direction, still pointing along the x-axis
//     /**
//      *        ^ x
//      *        l0 (1,0)
//      *        |
//      *        |
//      * y <----x0-----x1
//      *        (0,0)   (0,-1)
//      * 
//      * The bearing from x1 is 45 deg and range is sqrt(2)
//      * The bearing from x0 is 0 deg and range is 1
//      * */

//     // First measurement
//     // The measured range and bearing (w/noise)
//     RangeBearingMeasurement meas;
//     meas.bearing = 0;
//     meas.range = 1;
//     meas.measured_from_symbol = X(0);
//     Pose3 meas_var;

//     // Second measurement
//     RangeBearingMeasurement meas1;
//     meas1.bearing = M_PI_4;
//     meas1.range = sqrt(2);
//     meas1.measured_from_symbol = X(1);
//     Pose3 meas1_var(Rot3(), Point3(0, -1, 0));

//     // Create the noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector4(0.1, 0.1, 0.1, 0.1));

//     // Create the measurements
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);
//     measurements.push_back(meas1);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0, 1);

//     // Create the values
//     Values val;
//     val.insert(X(0), meas_var);
//     val.insert(X(1), meas1_var);
//     val.insert(landmark_symbol, landmark_var);

//     // Create the factor
//     std::list<Key> keys{X(0), X(1), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, X(0), keys);

//     // Compute the error on the un-optimised factor and check
//     // Remember that order is bearing then range
//     Vector err = factor.unwhitenedError(val);
//     ASSERT_EQ(err.size(), 4);
//     ASSERT_NEAR(err[0], 0, unwhitened_eps);  // X(0) bearing
//     ASSERT_NEAR(err[1], 0, unwhitened_eps);  // X(0) range
//     ASSERT_NEAR(err[2], 0, unwhitened_eps);  // X(1) bearing
//     ASSERT_NEAR(err[3], 0, unwhitened_eps);  // X(1) range
// }

// TEST(UnwhitenedError, two_meas_exact_optimise)
// {

//     // Observe a single landmark at 1 unit along the x-axis from a vehicle at two poses, with some error
//     // The vehicle moves one unit in the -y direction, still pointing along the x-axis
//     /**
//      *        ^ x
//      *        l0 (1,0)
//      *        |
//      *        |
//      * y <----x0-----x1
//      *        (0,0)   (0,-1)
//      * 
//      * The bearing from x1 is 45 deg and range is sqrt(2)
//      * The bearing from x0 is 0 deg and range is 1
//      * */

//     // First measurement
//     // The measured range and bearing (w/noise)
//     RangeBearingMeasurement meas;
//     meas.bearing = 0;
//     meas.range = 1;
//     meas.measured_from_symbol = X(0);
//     Pose3 meas_var;

//     // Second measurement
//     RangeBearingMeasurement meas1;
//     meas1.bearing = M_PI_4;
//     meas1.range = sqrt(2);
//     meas1.measured_from_symbol = X(1);
//     Pose3 meas1_var(Rot3(), Point3(0, -1, 0));

//     // Thirs measurement
//     RangeBearingMeasurement meas2;
//     meas2.bearing = -M_PI_4;
//     meas2.range = sqrt(2);
//     meas2.measured_from_symbol = X(2);
//     Pose3 meas2_var(Rot3(), Point3(0, 1, 0));

//     // Create the noise model. Should be a 2x2 matrix for a single measuement
//     // sigma bearing, sigma range
//     auto noise_model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));

//     // Create the measurements
//     std::vector<RangeBearingMeasurement> measurements;
//     measurements.push_back(meas);
//     measurements.push_back(meas1);
//     measurements.push_back(meas2);

//     // The landmark variable
//     Symbol landmark_symbol(L(0));
//     SemiParametricLandmark landmark_var(0.1, 0.8);

//     // Create the values
//     Values val;
//     val.insert(X(0), meas_var);
//     val.insert(X(1), meas1_var);
//     val.insert(X(2), meas2_var);
//     val.insert(landmark_symbol, landmark_var);

//     // Create the factor
//     std::list<Key> keys{X(0), X(1), X(2), L(0)};
//     LandmarkFactor factor(noise_model, measurements, landmark_symbol, X(0), keys);

//     // Do the optimisation
//     NonlinearFactorGraph graph;
//     graph.emplace_shared<LandmarkFactor>(factor);
//     LevenbergMarquardtParams params;
//     params.setVerbosityLM("TRYLAMBDA");
//     params.setVerbosity("LINEAR");
//     //params.setErrorTol(0.1); // extremely arbitrary
//     //params.setAbsoluteErrorTol(0.0001); // also extremely arbitrary
//     LevenbergMarquardtOptimizer optimizer(graph, val, params);
//     Values result = optimizer.optimize();
//     Marginals marginals(graph, result);
//     print(marginals.marginalCovariance(L(0)), "L0 covariance");

//     // // Compute the error on the un-optimised factor and check
//     // // Remember that order is bearing then range
//     // Vector err = factor.unwhitenedError(val);
//     // ASSERT_EQ(err.size(), 4);
//     // ASSERT_NEAR(err[0], 0, unwhitened_eps);  // X(0) bearing
//     // ASSERT_NEAR(err[1], 0, unwhitened_eps);  // X(0) range
//     // ASSERT_NEAR(err[2], 0, unwhitened_eps);  // X(1) bearing
//     // ASSERT_NEAR(err[3], 0, unwhitened_eps);  // X(1) range
// }


/*
    Optimal elevation solver tests
*/

TEST(OptimalElevation, simple)
{

    Point3 landmark_world(1, 0, -1);
    Pose3 sensor_world(Rot3(Quaternion(1, 0, 0, 0).normalized()), Point3(0,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, rotated_sonar)
{

    Point3 landmark_world(1, 0, -1);
    Pose3 sensor_world(Rot3(Quaternion(0.991, 0, 0.131, 0).normalized()), Point3(0,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, rotated_translated_sonar)
{

    Point3 landmark_world(1, 0, -1);
    Pose3 sensor_world(Rot3(Quaternion(0.991, 0, 0.131, 0).normalized()), Point3(0.1,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, general_1)
{

    Point3 landmark_world(-1, 4, -1);
    Pose3 sensor_world(Rot3(Quaternion(0.991, 0, 0.131, 0).normalized()), Point3(0.1,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, general_2)
{

    double z_ground = -1.1;
    Point3 landmark_world(-1, 4, z_ground);
    Pose3 sensor_world(Rot3(Quaternion(0.991, 0, 0.131, 0).normalized()), Point3(0.1,0,-0.5));
  

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}


TEST(OptimalElevation, general_3)
{

    Point3 landmark_world(-1, 4, -1);
    Pose3 sensor_world(Rot3(Quaternion(0.987, 0.099, 0.129, 0).normalized()), Point3(0.1,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, general_4)
{

    Point3 landmark_world(-1, 4, -1);
    Pose3 sensor_world(Rot3(Quaternion(0.987, -0.196, 0.198, 0.036).normalized()), Point3(0.1,0,0));
    double z_ground = -1;

    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}

TEST(OptimalElevation, general_5)
{

    double z_ground = -1.4;
    Point3 landmark_world(1, 0, z_ground);
    Pose3 sensor_world(Rot3(Quaternion(0.983, 0.129, 0.129, -0.017).normalized()), Point3(0,0,0));


    // Here we try to recover elevation
    // Apply prediction
    slam_geometry::RangeBearingMeasurement meas = slam_geometry::prediction(sensor_world, landmark_world);

    
    double elev_best = slam_geometry::predictOptimalElevation(sensor_world, meas.bearing, meas.range, z_ground); 

    std::cout << "elevation when -30 straight on is " << elev_best*RAD2DEG << " deg when rotated 15 deg in x" << std::endl;

    // Check equality
    ASSERT_NEAR(meas.elevation, elev_best, 0.01);
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}