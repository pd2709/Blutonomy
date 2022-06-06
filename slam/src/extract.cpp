#include "../include/slam/extract.hpp"

Extract::Extract(double threshold)
{
    // Set up AKAZE detector
    detector = cv::AKAZE::create();
    detector->setThreshold(threshold); // 0.001 for experimental
}

Extract::~Extract()
{

}

void Extract::detectKeypoints(cv:: Mat &image, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    detector->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
}


// Filters out keypoints based on minimum distance from origin along y-direction and distance from neighbours
// Parameters are set via dynamcic reconfigure
// Returns
void Extract::neighbourFilter(std::vector<cv::KeyPoint> &in_keypoints, cv::Mat &in_descriptors, std::vector<KeyPoint> &out_keypoints, cv::Mat &out_descriptors)
{
    // // Remove below threshold line
    // // TODO: make more robust with check for existence if this code is used more extensively
    // int y_height = image.size[0];
    // int line_height = y_height * (1 - min_scan_range / sonar_range_max);

    double keypt_range = 10; // range in pixels to exclude as duplicate

    // For each keypoint
    for (int i = 0; i < in_keypoints.size(); i++)
    {
        auto keypt1 = in_keypoints.begin() + i;

        // The subject for removal consideration is keypt1
        bool should_add = true;

        // For all keypoints from the first+1 to end...
        auto keypt2 = keypt1 + 1;
        while (keypt2 != in_keypoints.end())
        {
            // Compute distance. Less than acceptable, flag keypt1 for removal
            float dist = sqrt(pow(keypt2->pt.x - keypt1->pt.x, 2) + pow(keypt2->pt.y - keypt1->pt.y, 2));
            if (dist < keypt_range)
            {
                should_add = false;
            }
            ++keypt2;
        }

        // Do the addition
        if (should_add)
        {
            out_keypoints.push_back(in_keypoints[i]);
            out_descriptors.push_back(in_descriptors.row(i));
        }
    }
}

/**
 * Extract_Oculus 
 * Extract keypoints from the image. Return these 
 * @param  {cv::Mat} image : The image to process
 */
std::vector<cv::KeyPoint> Extract::process_image(cv::Mat &image)
{
    // Get keypoints
    cv::Mat descriptors_raw, descriptors;
    std::vector<cv::KeyPoint> keypoints_raw, keypoints;
    detectKeypoints(image, keypoints_raw, descriptors_raw);

    // Filter keypoints that are too close to each other
    neighbourFilter(keypoints_raw, descriptors_raw, keypoints, descriptors);

    // // Add keypoints to image
    // cv::Mat image_keypoints;
    // cv::Mat image_8bit;
    // image.convertTo(image_8bit, CV_8UC1, 0.01); // DrawKeyPoints only seems to work on 8bit images
    // cv::drawKeypoints(image_8bit, keypoints, image_keypoints, cv::Scalar(0,0,255));

    // // Draw the sonar origin
    // cv::drawMarker(image_keypoints, cv::Point2d(0,image_keypoints.rows/2), cv::Scalar(0, 255, 0), MARKER_CROSS, 10, 1);

    // // Label the points
    // for (size_t i = 0; i<keypoints.size(); i++)
    // {
    //     cv::putText(image_keypoints, to_string(i), cv::Point2d(keypoints[i].pt.x, keypoints[i].pt.y), cv::FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(255, 0, 0));
    // }

    // // Display the image
    // cv::imshow("sonar_display", image_keypoints);
    // cv::imwrite("sonar_display.png", image_keypoints);
    // cv::waitKey(1000);

    // Return locations
    return keypoints;

}