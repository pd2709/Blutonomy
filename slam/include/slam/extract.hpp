// extract.hpp
// A class to extract key points from an image

#pragma once

// OpenCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "cv_bridge/cv_bridge.h"

// Helpful structs and methods
#include "slam_geometry.hpp"

#include <cmath>

using namespace std;
using namespace cv;


class Extract{

    public:
        Extract(double threshold);
        ~Extract();

        std::vector<cv::KeyPoint> process_image(cv::Mat &image);

    private:
        cv::Ptr<cv::AKAZE> detector;
        void detectKeypoints(cv:: Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
        void neighbourFilter(std::vector<cv::KeyPoint> &in_keypoints, cv::Mat &in_descriptors, std::vector<KeyPoint> &out_keypoints, cv::Mat &out_descriptors);

};