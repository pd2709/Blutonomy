#include "GeometricTransforms.hpp"

namespace slam_geometry{

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
}