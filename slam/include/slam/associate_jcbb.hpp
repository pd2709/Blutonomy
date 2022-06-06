// associate_jcbb
// Data association in the sensor frame
// Not really joint compatibility branch and bound, but a simpler algorithm utilising individual compatibility

#pragma once

#include "associate.hpp"

#include "slam_geometry.hpp"

#include <gtest/gtest_prod.h>

class JCBB : public Associate{

    public:
    JCBB(std::vector<slam_geometry::RangeBearingMeasurement> landmarks, std::vector<slam_geometry::RangeBearingMeasurement> features, Eigen::Matrix2d cov_landmarks, Eigen::Matrix2d cov_features):landmarks_(landmarks), features_(features), cov_landmarks_(cov_landmarks), cov_features_(cov_features){
        n_ = landmarks_.size();
        m_ = features_.size();
    };
    JCBB(){}; // Empty ctor for tests if needed
    ~JCBB(){};

    /**
     * Run the associate algorithm
     * @return {std::vector<int>} : Result, of same dimension and features. For each feature, returns index of matched landmark, or -1 if no match 
     */
    hypothesis run();

    /**
     * Implementation of the MATLAB function chi2inv
     * @param  {float} a : p-value
     * @param  {int} dof : degrees of freedom
     * @return {double}  : Maximum X consistent with p
     */
    double chi2inv(float a, int dof);

    /**
     * Construct error functin hiji in JCBB
     * Separated out for unit testing purposes
     * @param  {hypothesis} H    : Hypothesis
     * @return {Eigen::ArrayXd}  : x-y for pairs in hypothesis
     */
    Eigen::ArrayXd constructErr(hypothesis H);

    private:

    /**
     * Recursively run JCBB 
     * @param  {std::vector<int>} H : The current hypothesis 
     * @param  {int} i              : The measurement (feature) index
     */
    void jcbb(hypothesis H, int i);

    bool isJointCompatible(hypothesis H);
    
    /**
     * Returns the squared Mahalanobis distance of the pairings in H
     * @return {double}  : Squared Mahalanobis distance
     */
    double squaredMahalanobisDist(hypothesis H);

    std::vector<slam_geometry::RangeBearingMeasurement> landmarks_;
    std::vector<slam_geometry::RangeBearingMeasurement> features_;
    hypothesis best_;

    int n_ = 0; // No. landmarks
    int m_ = 0; // No. features

    // TODO: generalise to covariance matrices that may be non-uniform across landmarks and features
    Eigen::Matrix2d cov_landmarks_; // Covariance matrix for a single landmark
    Eigen::Matrix2d cov_features_;  // Covariance matrix for a single feature

    FRIEND_TEST(JCBB_squaredMahalanobisDist, one_pair);
    FRIEND_TEST(JCBB_squaredMahalanobisDist, two_pairs);
    FRIEND_TEST(JCBB_squaredMahalanobisDist, many_pairs);

};