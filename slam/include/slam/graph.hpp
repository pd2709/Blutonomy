// graph.hpp
// Peforms the factor graph optimisation

#pragma once

#include "gtsam/inference/Symbol.h"
#include "gtsam/navigation/CombinedImuFactor.h"
#include "gtsam/geometry/concepts.h"
#include "gtsam/navigation/ImuFactor.h"
#include "gtsam/navigation/PreintegrationParams.h"
#include "gtsam/nonlinear/NonlinearFactorGraph.h"
#include "gtsam/slam/BetweenFactor.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "gtsam/sam/BearingRangeFactor.h"
#include "gtsam/nonlinear/Marginals.h"
#include "gtsam_unstable/slam/RelativeElevationFactor.h"

#include "slam_geometry.hpp" // Helper structs and functions

#include "associate.hpp"
#include "associate_nearest.hpp"
#include "associate_jcbb.hpp"

// Symbol shorthands make it much neater to represent the keys in gtsam::Values data structures
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz) - acceleration, gyro biases
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot) - vehicle velocity
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y) - vehicle position
using gtsam::symbol_shorthand::S;   // Pose 3 (x,y,z,r,p,y) - sonar position
using gtsam::symbol_shorthand::L;  // Landmark observation
using gtsam::symbol_shorthand::O; // Origin

class Graph{

    public:

    // Create an empty graph
    Graph(double ACCEL_NOISE_SIGMA, double GYRO_NOISE_SIGMA, double ACCEL_BIAS_RW_SIGMA, double GYRO_BIAS_RW_SIGMA, double LANDMARK_NOISE_ELEV, double LANDMARK_NOISE_BEAR, double LANDMARK_NOISE_RANGE, double g, gtsam::Point3 SONAR_TRANSLATION, gtsam::Rot3 SONAR_ROTATION, bool SHOULD_USE_JCBB, bool SHOULD_USE_DEPTH_FACTORS);

    // Add IMU data measurement
    void addImuMeasurement(gtsam::Vector3 linear_acceleration, gtsam::Vector3 angular_velocity, float dt);

    // Add new landmakrk measurements
    void addLandmarkMeasurements(const std::vector<slam_geometry::RangeBearingMeasurement> new_features);

    /**
     * Returns dead reckoned position
     * @return {std::pair<gtsam::Vector3, gtsam::Quaternion>}  : Latest position and rotation
     */
    gtsam::Pose3 getEstimateDeadReckon();

    // Following are only public to facilitate data publishing

    // Track number of landmarks
    int n_landmarks = 0;

    gtsam::Values result; // Last graph result
    gtsam::Marginals last_marginals; // Store last marginal result.

    gtsam::NavState prev_state;
    gtsam::NavState prop_state;
    gtsam::imuBias::ConstantBias prev_bias;

    int frame_count_ = 0;  // The priors are attached at frame_count_=0. At every successive image, frame_count is incremented to index the pose associated with that frame.

    std::vector<slam_geometry::RangeBearingMeasurement> associator_landmarks_sensor;  // Landmarks optimised at step frame_count-1, projected into frame_count under vehicle dead reckoning, which are used in association 
    std::vector<slam_geometry::RangeBearingMeasurement> associator_features_sensor; // Features detected at step frame_count
    std::vector<gtsam::Point3> associator_landmarks_global; // Landmarks optimised at step frame_count-1, projected into frame_count under vehicle dead reckoning, which are used in association
    std::vector<gtsam::Point3> associator_features_global; // Features detected at step frame_count

    Associate::hypothesis hypothesis; // Hypothesis connecting associator_landmarks and associator_features

    bool is_imu_received = false; // Track whether any imu data received yet
    bool is_graph_empty = true; // True until first landmark factors added

    // Solver
    gtsam::ISAM2 *isam2_ = 0;
    
    private:
    // Initialise new NL factor graph with priors
    void initGraphWithPriors();

    // Configure the solver
    void setupSolver();

    // Initialise new landmark
    void initLandmark(gtsam::Point3 init_pos);

    // Returns a pre-integrator with the correct noise and bias factors for this IMU
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();

    // Graph
    gtsam::NonlinearFactorGraph *graph_ = 0;
    

    // IMU preintegrators
    std::shared_ptr<gtsam::PreintegrationType> imu_preintegrator_between_;      // Reset each sonar frame, for between integration
    std::shared_ptr<gtsam::PreintegrationType> imu_preintegrator_perpetual_;    // Never reset, for DR trajectory calculation

    // The initial values for each graph update
    gtsam::Values initial_values_;

    gtsam::Point3 SONAR_POS_OFFSET_; 
    gtsam::Rot3 SONAR_ROT_OFFSET_; // 15 degrees down

    // We use the sensor specs to build the noise model for the IMU factor.
    // The numbers hard coded below are sampled from a UUV simulator message
    // Set via launch file
    // ! Lachlan suggests updating the below for the PixHawk - look at datasheet for info

    double ACCEL_NOISE_SIGMA_;       //  m s^-2 Hz^-0.5 - from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6767180/
    double GYRO_NOISE_SIGMA_;        //  rad s^-1 Hz^-0.5  - from https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6767180/
    double ACCEL_BIAS_RW_SIGMA_;     //  m s^-3 Hz^-0.5 - //! Complete guess from https://github.com/borglab/gtsam/issues/213
    double GYRO_BIAS_RW_SIGMA_;      //  rad s^-2 Hz^-0.5 //! Complete guess from https://github.com/borglab/gtsam/issues/213

    double LANDMARK_NOISE_ELEV_;  //   rad
    double LANDMARK_NOISE_BEAR_;  //   rad
    double LANDMARK_NOISE_RANGE_; // m

    double g_;    // m/s^2 accel due to gravity

    bool SHOULD_USE_JCBB_ = false;           // True for JCBB to be used, else use ICP
    bool SHOULD_USE_DEPTH_FACTORS_ = false;  // True for depth factors to be added to graph, else do not include

};