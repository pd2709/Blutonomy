#include "../include/slam/associate_nearest.hpp"

Nearest::hypothesis Nearest::run()
{

    hypothesis h;

    // In accordance with algo in paper 44
    double D1 = 1;
    double D2 = 2.5;

    // Index to add new landmark
    int new_landmark_index = 0;

    // For each feature...
    for (int f = 0; f < features_.size(); f++)
    {

        // Reset tracking variables
        int j = -1;
        double d_min = 100000;
        int nv = 0;

        // Grab the feature
        gtsam::Point3 f_pos = features_[f];

        // ..check distance to each landmark in turn.
        for (int l = 0; l < landmarks_.size(); l++)
        {

            gtsam::Point3 l_pos = landmarks_[l];
            double d = f_pos.distance(l_pos);

            //std::cout << "Feature " << f << " Landmark " << l << " separated by d = " << d << std::endl;

            // If distance less than closest ever found..
            if (d < d_min)
            {
                // Store this as the best match
                d_min = d;
                j = l;

                // Count the number of matches within the "close enough" distance threshold D1
                if (d < D1)
                {
                    nv += 1;
                }
            }
        }

        // If the there is no match, create a new feature
        if (nv == 0 && d_min > 0.8)
        {
            //std::cout << "No match - create new feature\n";
            h.feature_indices.push_back(f);
            h.landmark_indices.push_back(landmarks_.size() + new_landmark_index);
            new_landmark_index++;
        }

        // If there are ambiguous matches, do not accept any
        else if (nv > 1)
        {
            //std::cout << "Ambiguous match - do not accept\n";
        }

        // If there is a single match, accept it
        else
        {
            //std::cout << "Match - feature " << f << " and landmark " << j << std::endl;
            h.feature_indices.push_back(f);
            h.landmark_indices.push_back(j);
            
        }

        // TODO: Implement final check for features already matched in this frame
    }

    return h;

    // Find the distance to each
    // If distance less than minimum seen, log this as the best match
    // If distance less than a generic minimum, increment number of “any matches”
    // If no features were matched and the feauture was a given distance from others, create new landmark
    // If there were multiple matches, don’t match with anything
    // If there was exactly one match, make the match

}
