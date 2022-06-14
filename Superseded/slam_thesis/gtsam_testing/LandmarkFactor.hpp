#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/inference/Symbol.h>
#include <map> // Using a map to track measurements
#include "gtsam/geometry/BearingRange.h"
#include "gtsam/base/ThreadsafeException.h"

#include "GeometricTransforms.hpp"
#include "SemiParametricLandmark.hpp"

#include <boost/function.hpp>

using namespace gtsam;
using namespace slam_geometry;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;

class LandmarkFactor : public gtsam::NoiseModelFactor
{
    private:

        // Store the measurement at each vehicle pose keyed with Symbol
        std::map<Symbol, RangeBearingMeasurement> stored_measurements_;

        // Symbolic key of the landmark associated with this factor
        Symbol landmark_symbol_;
        Symbol base_pose_symbol_; //... and the pose of first observation, within which frame the variable is parameterised
        
    protected:

        // Copied from gtsam.org/doxygen/a00932_source.html
        typedef NoiseModelFactor Base;
        typedef LandmarkFactor This;

    public:

        /**
         * Default Constructor for I/O
         */
        LandmarkFactor() {}

        //! The landmark key is assumed to be last in the key list
        LandmarkFactor(const SharedNoiseModel& noiseModel, std::vector<RangeBearingMeasurement> new_measurements, Symbol new_landmark_symbol, Symbol new_base_pose_symbol, KeyList all_keys):
        Base(noiseModel, all_keys),
        landmark_symbol_(new_landmark_symbol),
        base_pose_symbol_(new_base_pose_symbol)
        {
            std::cout << "Created LandMark factor with keys" << std::endl;

            for (int i=0; i<all_keys.size(); i++)
            {
                Symbol k = keys_[i];
                k.print();
            }
            

            std::cout << std::endl;
            

            // Add each measurement to the map, keyed by the vehicle symbol
            for (auto it = new_measurements.begin(); it != new_measurements.end(); ++it)
            {
                auto &meas = *it;
                stored_measurements_.insert(std::pair<Symbol,RangeBearingMeasurement>(meas.measured_from_symbol, meas));
            }
        }

        ~LandmarkFactor() {}

        /**
         * Error function z-h(x), which GTSAM calls during the optimisation process
         * 
         * The error vector contains one entry for every diagonal element in the square noise model. For an n*n noise model, the error vector would be [x0_psi; x0_r; x1_psi; x1_r; ... xn-1_psi, xn-1_r]
         * 
         * @param  {Values} x                              : Pose3 and SemiParametricLandmark values connected to this factor
         * @param  {boost::optional<std::vector<Matrix>} > : Optional Jacobian (returns derivative wrt each of the values)
         * @return {Vector}                                : Vector of bearing and range errors for the stacked measurements
         */
        Vector unwhitenedError(const Values &x, boost::optional<std::vector<Matrix> &> H = boost::none) const override
        {

            // Allocate the error vector
            // The dimension of the factor is equal to the dimension of the noise model, which is what this->dim(returns). The number off error terms should be equal to the dimension of the noise model.
            Vector total_error = Vector::Zero(this->dim());
            //std::cout << "dim of factor = " << this->dim() << std::endl;

            // Not entirely sure why this check is needed, but the NoiseModelFactorN classes include it
            if (this->active(x))
            {

                // Extract the latest landmark and base pose values from the Values
                if (!x.exists(landmark_symbol_))
                {
                    throw(InvalidArgumentThreadsafe("No landmark value provided to LandmarkFactor. Expected to find at key " + string(landmark_symbol_)));
                }

                if (!x.exists(base_pose_symbol_))
                {
                    throw(InvalidArgumentThreadsafe("No base pose value provided to LandmarkFactor. Expected to find at key " + string(base_pose_symbol_)));
                }

                const SemiParametricLandmark &landmark_variable = x.at(landmark_symbol_).cast<SemiParametricLandmark>();
                // We write this as a measurement to pass to the inverse projection function
                RangeBearingMeasurement m_b;
                m_b.bearing = landmark_variable.getBearing();
                m_b.range = landmark_variable.getRange();

                // Here we project the landmark variable using the latest estimate of its elevation
                // TODO: replace this with the latest bearing estimate from the separate update loop
                m_b.elevation = 0;

                // We save the base pose
                const Pose3 &x_b = x.at(base_pose_symbol_).cast<Pose3>();

                // We iterate over all Pose3 values
                // Filter to only extract pose variables
                Values::ConstFiltered<Pose3> pose_vals = x.filter<Pose3>();

                int i = 0;
                for (const auto &pose : pose_vals)
                {
                    Symbol sym = pose.key;
                    //print(sym); // Debugging print - which symbol are we dealing with?

                    // Sanity check that we are not dealing with a landmark (should have filtered only poses)
                    assert(pose.key != landmark_symbol_);

                    // Get this pose and measurement
                    Pose3 x_k = pose.value;
                    RangeBearingMeasurement m = stored_measurements_.at(sym);

                    // For any other symbol, compute the inverse projection
                    Point3 pi_inv = prediction_inv(x_b, m_b);

                    // Project into this sonar pose
                    Matrix dz_dq = Matrix26();
                    Matrix dq_dx = Matrix36();
                    RangeBearingMeasurement pi = prediction(x_k, pi_inv, dz_dq, dq_dx);

                    //std::cout << pi.bearing << " | " << pi.range << std::endl;

                    // Calulate the terms in the error vector due to this measurement
                    //std::cout << "i=" << i << " and size of vec is " << total_error.size() << std::endl;
                    total_error[0] += pi.bearing - m.bearing;
                    total_error[1] += pi.range - m.range;

                    // Compute optional Jacobians
                    if (H)
                    {
                        //std::cout << "H was requested and has dim " << (*H).size() << std::endl;

                        //! We assume that the landmark key is last
                        // Compute the Pose3 Jacobians
                        //Symbol k = keys_[i];
                        //k.print();

                        // Each pose variable Jacobian should be 2x6
                        //(*H)[i] = dz_dx;

                        //std::cout << "dz_dx = " << std::endl << dz_dx << std::endl;
                    }

                    i++;
                }

                if (H)
                {
                    // Compute the landmark variable Jacobian
                    // As written in notebook, this is the 2x2 identity
                    (*H)[keys_.size() - 1] = Matrix::Identity(2, 2);

                    //std::cout << "H = " << (*H)[2].size() << std::endl;
                }

                // Return the error
            }

            std::cout << "Computed unwhitened error without throwing an exception " << std::endl;
            //std::cout << "keys size" << keys_.size() << std::endl;

            return total_error;

            // If Hamiltonian requested, compute this
            //! Need to check what order matrices should be returned in. Does the order just need to match the noise model? The dimension of the vector H is zero...
        }

        RangeBearingMeasurement getMeasurement(Symbol vehicleKey)
        {
            return stored_measurements_.at(vehicleKey);
        }

};
// int returnOne(){
//     return 1;
// }