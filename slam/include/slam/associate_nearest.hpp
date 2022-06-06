// associate_nearest.hpp
// Perform data association using a nearest neighbour approach
#pragma once

#include "associate.hpp"

#include "gtsam/geometry/Point3.h"

class Nearest : public Associate{

    public:
    Nearest(std::vector<gtsam::Point3> landmarks, std::vector<gtsam::Point3> features):landmarks_(landmarks), features_(features){
        n_ = landmarks_.size();
        m_ = features_.size();
    };

    hypothesis run();

    private:
    std::vector<gtsam::Point3> landmarks_;
    std::vector<gtsam::Point3> features_;
    hypothesis best_;

    int n_ = 0; // No. landmarks
    int m_ = 0; // No. features

};