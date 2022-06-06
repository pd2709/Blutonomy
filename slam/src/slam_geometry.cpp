#include "../include/slam/slam_geometry.hpp"

namespace slam_geometry{
    
    RangeBearingMeasurement prediction(gtsam::Pose3 vehicle_world, gtsam::Point3 landmark_world, boost::optional<gtsam::Matrix&> dz_dq, boost::optional<gtsam::Matrix&> dq_dx)
    {
        //gtsam::Matrix dq_dx = gtsam::Matrix36(); // Derivative of measurement in local cartesian coords wrt vehicle pose

        // Calculate
        gtsam::Point3 landmark_vehicle = vehicle_world.transformTo(landmark_world, dq_dx);
        // std::cout << "x=" << landmark_vehicle.x() << " y=" << landmark_vehicle.y() << " z=" << landmark_vehicle.z() << std::endl;

        //std::cout << "dq_dx = " << std::endl << dq_dx << std::endl;

        // Compute the Jacobian dz_dq
        if (dz_dq)
        {

            double x = landmark_vehicle.x();
            double y = landmark_vehicle.y();
            double z = landmark_vehicle.z();

            double r2 = sqrt(pow(x, 2) + pow(y, 2));
            double r3 = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

            *dz_dq << -y / r2, -x / r2, 0,
                x / r3, y / r3, z / r3;
        }

        //Derivative of measurement in polar coords wrt local
        // gtsam::Matrix23 dz_dq;
        // dz_dq <<
        // -y/r2, -x/r2, 0, 
        // x/r3, y/r3, z/r3;

        // if (dz_dx){
        //     *dz_dx = dz_dq * dq_dx;
        // }

        RangeBearingMeasurement meas;
        meas.bearing = atan2(landmark_vehicle.y(), landmark_vehicle.x());
        meas.range = landmark_vehicle.norm();
        meas.elevation = atan2(landmark_vehicle.z(), sqrt(pow(landmark_vehicle.x(),2)+pow(landmark_vehicle.y(),2)));
        return meas;
    }

    /**
     * Inverse prediction function for 2D sonar frame into 3D world
     * The quaternion used to construct the input pose MUST be normalized!
     * @param  {gtsam::Pose3} vehicle_world          : Pose of vehicle in world frame
     * @param  {RangeBearingMeasurement} measurement : Measurement of feature in vehicle frame
     * @return {gtsam::Point3}                       : Location of feature in world frame
     */
    gtsam::Point3 prediction_inv(gtsam::Pose3 vehicle_world, RangeBearingMeasurement measurement)
    {
        // Construct point in local coords
        double x, y, z;
        x = measurement.range * (cos(measurement.bearing)*cos(measurement.elevation));
        y = measurement.range * (sin(measurement.bearing)*cos(measurement.elevation));
        z = measurement.range * sin(measurement.elevation);
        gtsam::Point3 landmark_vehicle(x, y, z);

        // Convert to global coords
        gtsam::Point3 landmark_world = vehicle_world.transformFrom(landmark_vehicle);

        return landmark_world;
    }

    void plotLocalInLocal(cv::Mat &image, std::vector<RangeBearingMeasurement> measurements_local, cv::Scalar color)
    {
        // Create the vector of points
        std::vector<cv::Point2d> points;

        // ! These hard-coded values are only correct for start of Test6_migrated. Need to make more general by taking from latest header.
        double r_res = 0.0102048415423; // Close range 0.00510242 | Long range: 0.0102048415423
        int y_max = 256;
        double hfov = 130.0*DEG2RAD;

        for (auto i = measurements_local.begin(); i<measurements_local.end(); i++ )
        {
            // Create this point
            int y = y_max*(0.5-i->bearing/hfov);
            int x = i->range/r_res;

            cv::circle(image, cv::Point2d(x,y), 2, color, cv::FILLED);

        }
        // l.elevation = 0;    // We assume that Oculus landmarks are at zero-elevation (i.e. on sensor midplane)
        
        // int x = kpt_iter->pt.x;
        // int y = kpt_iter->pt.y;

        // l.bearing = (double(y_max-2*y)/double(2*y_max)) * hfov;
        // l.range = x * r_res;
    
    }

    void plotGlobalInLocal(cv::Mat &image, gtsam::Pose3 vehicle_world, std::vector<gtsam::Point3> landmarks_global)
    {
        // Convert from global to local
        std::vector<RangeBearingMeasurement> landmarks_local;

        for (auto l = landmarks_global.begin(); l < landmarks_global.end(); l++)
        {
            landmarks_local.push_back(prediction(vehicle_world, *l));
        }

        // Pass to local plotter
        plotLocalInLocal(image, landmarks_local, cv::Scalar(0, 255, 0));
    }

    std::pair<gtsam::Unit3, double> rangeBearingMeasurement2gtsam(RangeBearingMeasurement meas)
    {
        float x, y, z;

        float elevation = meas.elevation;
        float bearing = meas.bearing;
        float range = meas.range;

        x = range * cos(elevation) * cos(bearing);
        y = range * cos(elevation) * sin(bearing);
        z = range * sin(elevation);

        // In the assumption that landmarks lie on sensor plane, z should always be zero
        // assert(z == 0);

        gtsam::Unit3 measured_bearing(x, y, z);

        return std::pair<gtsam::Unit3, double>(measured_bearing, range);
    }

    double predictOptimalElevation(gtsam::Pose3 sensor_in_world, double bearing, double range, double floor_z_rel_world)
    {
        double err_min = 1000;
        double elev_best = 0;

        double cb = cos(bearing);
        double sb = sin(bearing);

        for (double elev = -M_PI_4; elev < M_PI_4; elev += 0.005)
        {
            gtsam::Point3 v_sens(range * cb * cos(elev), range * sb * cos(elev), range * sin(elev));
            gtsam::Point3 v_z_up = sensor_in_world.transform_from(v_sens); // FROM pose TO world coordinates
            double err = abs(floor_z_rel_world - v_z_up.z());
            if (err < err_min)
            {
                err_min = err;
                elev_best = elev;
            }
        }

        return elev_best;
    }
}