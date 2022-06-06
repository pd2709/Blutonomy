#include "../include/slam/graph.hpp"

#include <gtsam/base/serialization.h>
#include <gtsam/base/serializationTestHelpers.h>


// #include "../include/matt_thesis_slam_oculus/gtsam_serialisation_defs.hpp" // Definitions required for serialisation of gtsam objects, facilitating MATLAB export.

// Largely based on this example: https://github.com/haidai/gtsam/blob/master/examples/ImuFactorsExample.cpp

// Initialise acceleration with zero bias 
gtsam::Vector3 IMU_BIAS_ACCEL(0, 0, 0);


Graph::Graph(double ACCEL_NOISE_SIGMA, double GYRO_NOISE_SIGMA, double ACCEL_BIAS_RW_SIGMA, double GYRO_BIAS_RW_SIGMA, double LANDMARK_NOISE_ELEV, double LANDMARK_NOISE_BEAR, double LANDMARK_NOISE_RANGE, double g, gtsam::Point3 SONAR_TRANSLATION, gtsam::Rot3 SONAR_ROTATION, bool SHOULD_USE_JCBB, bool SHOULD_USE_DEPTH_FACTORS)
{
    
    // TODO: Set up constants here
    SONAR_POS_OFFSET_ = SONAR_TRANSLATION;
    SONAR_ROT_OFFSET_ = SONAR_ROTATION;

    ACCEL_NOISE_SIGMA_ = ACCEL_NOISE_SIGMA;
    GYRO_NOISE_SIGMA_ = GYRO_NOISE_SIGMA;
    ACCEL_BIAS_RW_SIGMA_ = ACCEL_BIAS_RW_SIGMA;
    GYRO_BIAS_RW_SIGMA_ = GYRO_BIAS_RW_SIGMA;

    LANDMARK_NOISE_ELEV_ = LANDMARK_NOISE_ELEV;
    LANDMARK_NOISE_BEAR_ = LANDMARK_NOISE_BEAR;
    LANDMARK_NOISE_RANGE_ = LANDMARK_NOISE_RANGE;

    g_ = g;

    SHOULD_USE_JCBB_ = SHOULD_USE_JCBB;
    SHOULD_USE_DEPTH_FACTORS_ = SHOULD_USE_DEPTH_FACTORS;

    // Set up the IMU preintegrator and solver
    initGraphWithPriors();
    setupSolver();
    //cv::namedWindow("reprojection_sonar_display", cv::WINDOW_NORMAL); // Display re-projected landmarks on the image itself

}

void Graph::addImuMeasurement(gtsam::Vector3 linear_acceleration, gtsam::Vector3 angular_velocity, float dt)
{   
    is_imu_received = true;
    // Don't modify imu data at all
    imu_preintegrator_between_->integrateMeasurement(linear_acceleration, angular_velocity, dt);
    imu_preintegrator_perpetual_->integrateMeasurement(linear_acceleration, angular_velocity, dt);

} 

// Configure the solver
void Graph::setupSolver()
{
    gtsam::ISAM2Params parameters;

    // TODO: following are from batch processing default values, may need to adjust
    // TODO: place somewhere easier to configure (header/conf file)
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam2_ = new gtsam::ISAM2(parameters);
}

gtsam::Pose3 Graph::getEstimateDeadReckon()
{
    //! These priors are also used in graph initialisation, so they should be defined somewhere central rather than twice
    // gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(1, 0, 0, 0);
    gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(-0.99161, 0.019928, 0.011390, 0.03392);
    gtsam::Point3 prior_point(0, 0, 0); // TODO - make these customisable (should come from launch file)
    gtsam::Pose3 prior_pose(prior_rotation, prior_point);

    gtsam::Vector3 prior_velocity(0, 0, 0);

    gtsam::imuBias::ConstantBias prior_imu_bias(IMU_BIAS_ACCEL, gtsam::Vector3(0,0,0)); // Assume zero initial bias
   
    // Store the previous state for imu integration and latest predicted outcome
    gtsam::NavState prior_state(prior_pose, prior_velocity);

    gtsam::NavState present_state = imu_preintegrator_perpetual_->predict(prior_state, prior_imu_bias);

    gtsam::Point3 pos = present_state.position();

    //std::cout << "DR Estimate: (x,y,z) = (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;

    return present_state.pose();
}

// Creates the graph and adds the prior factors
void Graph::initGraphWithPriors()
{
    // Initial orientation (quaternions)
    // TODO: replace with actual first measurement from IMU
    // ! Note that GTSAM expects w, x, y, z NOT x first!!!
    gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(1, 0, 0, 0);

    // Initial location in 3D global frame (cartesian coords)
    // ! If things don't work, re-asses this line. I'm initialising differently to example
    gtsam::Point3 prior_point(0, 0, 0);

    // The initial 6-DOF pose is a combined position and orientation
    gtsam::Pose3 prior_pose(prior_rotation, prior_point);

    // Assuming zero initial velocity, though this could be updated from other information sources
    gtsam::Vector3 prior_velocity(0, 0, 0);

    gtsam::imuBias::ConstantBias prior_imu_bias(IMU_BIAS_ACCEL, gtsam::Point3(0,0,0)); // Assume zero initial bias

    // A values object is a key-value map used to specify the values of a bunch of elements of a factor graph
    // Here it is used to store the prior factors
    // Namespaced keys are used for the indices

    initial_values_.insert(X(frame_count_), prior_pose);
    initial_values_.insert(V(frame_count_), prior_velocity);
    initial_values_.insert(B(frame_count_), prior_imu_bias);

    // TODO: Is it fine to assume a no-noise starting state (i.e. to zero all the following)?
    // Assemble prior noise model and add it the graph.`
    //! We know prior perfectly, so adjusted to make small
    auto point_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
    auto pose_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.001, 0.001, 0.001)
            .finished());                                                    // rad,rad,rad,m, m, m
    auto velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); // m/s
    auto bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    // Add all prior factors (pose, velocity, bias) to the graph.
    graph_ = new gtsam::NonlinearFactorGraph();
    graph_->addPrior(X(frame_count_), prior_pose, pose_noise_model);
    graph_->addPrior(V(frame_count_), prior_velocity, velocity_noise_model);
    graph_->addPrior(B(frame_count_), prior_imu_bias, bias_noise_model);

    // Grab the IMU parameters and initialise the preintegrator with these
    // TODO: does this belong in IMU init function?
    auto p = imuParams();

    // imu_preintegrator_between_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
    // imu_preintegrator_perpetual_ = std::make_shared<gtsam::PreintegratedImuMeasurements>(p, prior_imu_bias);
    imu_preintegrator_between_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p, prior_imu_bias);
    imu_preintegrator_perpetual_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(p, prior_imu_bias);

    // Sample code includes assert here, seemingly to verify that the preintegrator was creates successfully
    assert(imu_preintegrator_between_);
    assert(imu_preintegrator_perpetual_);
    
    // Store the previous state for imu integration and latest predicted outcome
    prev_state = gtsam::NavState(prior_pose, prior_velocity);
    prop_state = prev_state;
    prev_bias = prior_imu_bias; // Assuming bias does not change

    // Optional additional depth factors
    if (SHOULD_USE_DEPTH_FACTORS_)
    {
        initial_values_.insert(O(0), prior_point); // Anchor origin for relative altitude measurements
        graph_->addPrior(O(0), prior_point, point_noise_model);
    }
}

void Graph::initLandmark(gtsam::Point3 init_pos)
{

    initial_values_.insert(L(n_landmarks), init_pos);
    n_landmarks++;
}

// Returns a pre-integrator with the correct noise and bias factors for this IMU
// TODO: parameterise in config or header file
// TODO: account for different types of IMU (simulated vs PixHawk)
boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> Graph::imuParams()
{
    // From these single covariances, construct the covariance matrices
    // If the input is standard deviation, all elements need to be raised to power of 2 instead of 1
    // (Recall: variance = sigma^2)
    gtsam::Matrix33 measured_acc_cov = gtsam::I_3x3 * pow(ACCEL_NOISE_SIGMA_, 2);
    gtsam::Matrix33 measured_omega_cov = gtsam::I_3x3 * pow(GYRO_NOISE_SIGMA_, 2);
    gtsam::Matrix33 integration_error_cov =
        gtsam::I_3x3 * 1e-8; // error committed in integrating position from velocities.// TODO: value inherited from example. Update.

 // was 1e-8
 // change to 1e-5

    gtsam::Matrix33 bias_acc_cov = gtsam::I_3x3 * pow(ACCEL_BIAS_RW_SIGMA_, 2);
    gtsam::Matrix33 bias_omega_cov = gtsam::I_3x3 * pow(GYRO_BIAS_RW_SIGMA_, 2);
    gtsam::Matrix66 bias_acc_omega_int =
        gtsam::I_6x6 * 1e-5; // error in the bias used for preintegration. // TODO: value inherited from example. Update.

    // TODO: MakeSharedD assumes a NED frame. Confirm that this is the convention used in Gazebo.
    // ! Think about initialisation (see here: https://nicolovaligi.com/articles/robotics-for-developers/adding-accelerometer/)
    // ! Hard coded gravity value here!
    auto p = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g_);

    // PreintegrationBase params:
    p->accelerometerCovariance =
        measured_acc_cov; // acc white noise in continuous
    p->integrationCovariance =
        integration_error_cov; // integration uncertainty continuous
    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance =
        measured_omega_cov; // gyro white noise in continuous
    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov;     // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_int;

    return p;
}

void Graph::addLandmarkMeasurements(const std::vector<slam_geometry::RangeBearingMeasurement> new_features)
{

    /* Useful variables
    */

   if (is_imu_received == false)
   {
       std::cout << "No IMU data yet - skipping add landmarks " << std::endl;
       return;
   }

    // We're going to add measurements, so graph will no longer be empty.
   is_graph_empty = false;


    // Get the propagated vehicle pose
    // Given the previous state and bias, the preintegrator will propagate a prediction for the next state
    prop_state = imu_preintegrator_between_->predict(prev_state, prev_bias);
    gtsam::Pose3 wTx = prop_state.pose();    
    gtsam::Pose3 xTs(SONAR_ROT_OFFSET_, SONAR_POS_OFFSET_); // Build up the sonar offset in the vehicle frame
    // Compose rotation. See: https://samarth-robo.github.io/blog/2019/12/29/gtsam_conventions.html
    // This is the sonar frame, in which to represent points.
    gtsam::Pose3 wTs = wTx.compose(xTs);

    // This the rotation matrix giving sonar rotation relative to the world frame, in which landmarks are optimised
    // TODO: can use this to rotate marginals into frame then use as better input to JCBB
    gtsam::Matrix33 R_s = wTs.rotation().matrix();

    // Wipe previous associator record
    associator_landmarks_sensor.clear();
    associator_features_sensor.clear();
    associator_landmarks_global.clear(); 
    associator_features_global.clear(); 

    // Landmarks in global frame
    // Landmarks in sensor frame
    for (int i = 0; i < n_landmarks; i++)
    {
        gtsam::Point3 landmark_global = result.at<gtsam::Point3>(L(i));       // Landmark in global coordinates
        associator_landmarks_global.push_back(landmark_global);
        associator_landmarks_sensor.push_back(slam_geometry::prediction(wTs, landmark_global)); // Landmark in sensor coordinates
    }

    // Features in sensor frame
    // Features in global frame
    for (int i = 0; i < new_features.size(); i++)
    {
        slam_geometry::RangeBearingMeasurement feature_sensor = new_features[i];
        associator_features_sensor.push_back(feature_sensor);
        associator_features_global.push_back(slam_geometry::prediction_inv(wTs, feature_sensor));

    }

    frame_count_++;

    /*
        IMU FACTOR
        */
    auto preint_imu = dynamic_cast<const gtsam::PreintegratedCombinedMeasurements &>(*imu_preintegrator_between_);
    gtsam::CombinedImuFactor imu_factor(X(frame_count_ - 1), V(frame_count_ - 1),
                                        X(frame_count_), V(frame_count_),
                                        B(frame_count_ - 1), B(frame_count_), 
                                        preint_imu);

    graph_->add(imu_factor);


    /*
        VEHICLE TO SONAR FACTOR
        */

    // Add this sonar pose to the graph
    auto sonar_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-6);
    graph_->emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(frame_count_), S(frame_count_), xTs, sonar_noise_model);
    initial_values_.insert(S(frame_count_), xTs);

    /*
        RELATIVE ELEVATION FACTOR
    */

   // Optional additional depth factors
    if (SHOULD_USE_DEPTH_FACTORS_)
    {
        std::cout << "adding depth factor " << std::endl;
        auto depth_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);graph_->emplace_shared<gtsam::RelativeElevationFactor>(X(frame_count_), O(0), 0.0, depth_noise_model);
    }
   

    /* 
        SONAR TO LANDMARK FACTORS
        */

    // Create noise model for measurements
    auto landmark_noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(LANDMARK_NOISE_ELEV_, LANDMARK_NOISE_BEAR_, LANDMARK_NOISE_RANGE_)); // 0.1 rad std on altitude, azimuth, 0.2 m on range // TODO: refine noise assumptions (based on some hand calcs possibly)

    
    // If this is the first set of measurements received, create new landmarks with all points
    //! Could also initialise the factor graph at this point
    if (frame_count_ == 1){

        // For each landmark, create point
        for (auto f = associator_features_sensor.begin(); f < associator_features_sensor.end(); f++)
        {

            // Create landmark orientation represenation on a unit sphere
            gtsam::Unit3 bearing3D;
            double range;

            std::tie(bearing3D, range) = slam_geometry::rangeBearingMeasurement2gtsam(*f);

            gtsam::Point3 feature_pos_sonar = range * bearing3D;

            gtsam::Point3 feature_pos_world = wTs.transformFrom(feature_pos_sonar);
            initLandmark(feature_pos_world);

            // Add the first measurement of this landmark to the graph
            // We've just initiliased the landmark, so we know that its index will be n-1
            graph_->emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(S(frame_count_), L(n_landmarks - 1), bearing3D, range, landmark_noise);
            //std::cout << "Feature at :";
            //feature_pos_world.print();
        }
    }
    
    
    // Otherwise, perform data association
    else{

        /**
         * Do data association
         * 
         * 
         * Input:
         * 
         * - Feature measurements       (std::vector<slam_geometry::RangeBearingMeasurement>)
         * - Landmark positions         (std::vector<gtsam::Point3>)
         * - Current sonar position     (gtsam::Pose3 wTs)
         * 
         * Output:
         * - Hypothesis                 (JCBB:hypothesis hypothesis)
         **/


        // *********************************************************************************

        // EVERYTHING IN HERE IS JCBB - SPECIFIC

        // Place features on sonar plane image
        cv::Mat projection_image = cv::Mat::zeros(256, 391, CV_8UC3);
        slam_geometry::plotLocalInLocal(projection_image, associator_features_sensor, cv::Scalar(0, 0, 255));
        //cv::imwrite("reprojection_sonar_display.png", projection_image);
        // Plot both together
        slam_geometry::plotLocalInLocal(projection_image, associator_landmarks_sensor,cv::Scalar(0,255,0));
        // Show landmarks projected into sonar frame
        //cv::imshow("reprojection_sonar_display", projection_image);
        //cv::imwrite("reprojection_sonar_display.png", projection_image);
        //cv::waitKey(1000);

        // Pass vectors of landmarks x and features y to JCBB
        // For each feature, JCBB should return the index in the landmark vector corresponding to it
        Eigen::Matrix2d cov(2, 2);
        cov << 0.01, 0, 0, 0.04; // ! Remember that covariance is made up of sigma^2
        // cov << 0.01, 0, 0, 0.01;
        
        // Choose associator depending on set flag
        std::shared_ptr<Associate> associator;
        if (SHOULD_USE_JCBB_)
        {
            associator = std::make_shared<JCBB>(associator_landmarks_sensor, associator_features_sensor, cov, cov);
        }
        else
        {
           associator = std::make_shared<Nearest>(associator_landmarks_global, associator_features_global);
        }
        


        std::cout << "Starting Associator ... " << std::endl;
        hypothesis = associator->run();
        std::cout << "Finished Associator" << std::endl;

        std::cout << "Matched " << hypothesis.n_matches() << " landmarks and features" << std::endl;


        // *********************************************************************************

        // Keep track of features that have been matched
        // Depending on whether a feature is matched or not we store it differently for plotting
        std::vector<bool> is_matched(associator_features_sensor.size(), false);

        // Based on associations, add the measurement factors
        // We know than landmarks are indexed from 0 to n_landmarks, because this is how we constructed the landmarks vector
        // By convention, a new landmark will have index > n_landmarks
        for (int match_index = 0; match_index<hypothesis.n_matches(); match_index++)
        {
            //std::cout << "Matched feature " << hypothesis.feature_indices[match_index] << " with landmark " << hypothesis.landmark_indices[match_index] << std::endl;
            // Mark this as matched
            is_matched[hypothesis.feature_indices[match_index]] = true;

            // Get the measurement
            slam_geometry::RangeBearingMeasurement f = associator_features_sensor[hypothesis.feature_indices[match_index]];

            gtsam::Unit3 bearing3D;
            double range;

            std::tie(bearing3D, range) = slam_geometry::rangeBearingMeasurement2gtsam(f);

            // If this is a new landmark, initialise it now
            if (hypothesis.landmark_indices[match_index] >= n_landmarks)
            {
                initLandmark(associator_features_global[hypothesis.feature_indices[match_index]]);
                //std::cout << "Initialised new landmark ";

                // First measurement of landmark so not a "match" on this update
                is_matched[hypothesis.feature_indices[match_index]] = false;
            }

            // Add measurement factor to graph
            graph_->emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose3, gtsam::Point3>>(S(frame_count_), L(hypothesis.landmark_indices[match_index]), bearing3D, range, landmark_noise);

        
        }
    }


    // Now optimize and compare results.

    // This odometry prediction becomes the next initial value
    initial_values_.insert(X(frame_count_), prop_state.pose());
    initial_values_.insert(V(frame_count_), prop_state.v());
    initial_values_.insert(B(frame_count_), prev_bias);

    isam2_->update(*graph_, initial_values_);
    isam2_->update();
    result = isam2_->calculateEstimate();

    // Set the previous values equal to the current, in preparation for the next step
    prev_state = gtsam::NavState(result.at<gtsam::Pose3>(X(frame_count_)),
                                 result.at<gtsam::Vector3>(V(frame_count_)));
    prev_bias = result.at<gtsam::imuBias::ConstantBias>(B(frame_count_));

    // Reset the preintegration object, so we can start the preintegration afresh
    imu_preintegrator_between_->resetIntegrationAndSetBias(prev_bias);

    // Terminal print for quick check:
    std::cout << "SLAM Estimate: " << prev_state.pose();

    //graph_->print();

    // gtsam::Point3 prior_point(0, 0, 0);
    // auto point_noise_model = gtsam::noiseModel::Isotropic::Sigma(3, 0.001);
    // graph_->addPrior(O(0), prior_point, point_noise_model);
    // last_marginals = gtsam::Marginals(*graph_, result, gtsam::Marginals::QR);
    // std::cout << "Covariance of pose X" << frame_count_ << " is " << isam2_->marginalCovariance(X(frame_count_)) << std::endl;

/*
        Prepare the graph for the next update
    */
    // Reset here rather than end of addLandmarkMeasurements() so that data publishing in node has time to run
    // reset the graph
    graph_->resize(0); // ! This line is ABSOLUTELY CRUCIAL, at least in the simulated case.
    // Without it, I believe the graph becomes too "rigid" and cannot be optimised properly.
    // Resize removes all factors from the graph (not variables) so that only new factors are optimised with at next step.

    initial_values_.clear();


}