// Associate.hpp
// Abstract base class for associators
#pragma once

#include "slam_geometry.hpp"

class Associate{

    public:

    /**
     * An associator hypothesis
     * A matched pair is given by a feature and landmark index
    */
    struct hypothesis{
        std::vector<int> feature_indices;
        std::vector<int> landmark_indices;

        int n_matches()
        {
            if(feature_indices.size() != landmark_indices.size()){
                throw(std::domain_error("Expected feature and landmark indices to be same size"));
            }
            return landmark_indices.size();
        }
    };

    /**
     * Run the associate algorithm
     * @return {std::vector<int>} : Result, of same dimension and features. For each feature, returns index of matched landmark, or -1 if no match 
     */
    virtual hypothesis run() = 0;

    private:

};