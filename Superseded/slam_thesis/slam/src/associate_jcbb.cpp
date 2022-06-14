#include "../include/slam/associate_jcbb.hpp"

#include <boost/math/distributions/chi_squared.hpp>
#include <unsupported/Eigen/KroneckerProduct>

void JCBB::jcbb(JCBB::hypothesis h, int i)
{
    if (i>m_-1) //! Is this -1 correct?
    {
        if (h.n_matches() > best_.n_matches())
        {
            best_ = h;
        }
        else if (h.n_matches() == best_.n_matches())
        {
            if(squaredMahalanobisDist(h) < squaredMahalanobisDist(best_))
            {
                best_ = h;
            }
        }
    }

    else
    {
        for (int j = 0; j<n_; j++)
        {

            hypothesis individual_hypothesis;
            individual_hypothesis.feature_indices.push_back(i);
            individual_hypothesis.landmark_indices.push_back(j);
            bool is_ic = isJointCompatible(individual_hypothesis);

            hypothesis proposed_joint_hypothesis = h;
            proposed_joint_hypothesis.feature_indices.push_back(i);
            proposed_joint_hypothesis.landmark_indices.push_back(j);
            bool is_jc = isJointCompatible(proposed_joint_hypothesis);

            bool is_paired_already = (std::find(h.landmark_indices.begin(), h.landmark_indices.end(), j) != h.landmark_indices.end());

            if ((is_ic and is_jc) and !is_paired_already)
            {
                jcbb(proposed_joint_hypothesis, i+1);
            }
        }
    }

    if (h.n_matches() + m_ - i > best_.n_matches())
    {
        hypothesis proposed_joint_hypothesis = h;
        jcbb(proposed_joint_hypothesis, i+1);
    }
}

bool JCBB::isJointCompatible(hypothesis H)
{
    double dist = squaredMahalanobisDist(H);
    double threshold = chi2inv(0.9, 2*H.n_matches()); // Each match has 2 variables, so dof=2*n
    return dist < threshold;
}

double JCBB::squaredMahalanobisDist(hypothesis H)
{
    // Construct the h vector
    Eigen::ArrayXd hiji = constructErr(H);
    // std::cout << "hiji = " << std::endl << hiji << std::endl;

    int n = H.n_matches();

    Eigen::MatrixXd Hiji = Eigen::MatrixXd::Identity(2*n, 2*n);
    assert(Hiji.size() == pow(2*n, 2));

    Eigen::MatrixXd Giji = -Eigen::MatrixXd::Identity(2*n, 2*n);
    assert(Giji.size() == pow(2*n, 2));

    // std::cout << "Giji = " << std::endl << Giji << std::endl;

    Eigen::MatrixXd id = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd cov_x = Eigen::kroneckerProduct(id, cov_landmarks_);
    Eigen::MatrixXd cov_y = Eigen::kroneckerProduct(id, cov_features_);
    assert(cov_x.innerSize() == cov_x.outerSize()); // Should be square
    assert(cov_x.innerSize() == 2*n);   // Should be n pairs * 2 dof in size

    // std::cout << "cov_x = " << std::endl << cov_x << std::endl;
    // std::cout << "cov_y = " << std::endl << cov_y << std::endl;

    Eigen::MatrixXd Ciji = Hiji*cov_x*Hiji.transpose() + Giji*cov_y*Giji.transpose();

    // std::cout << "Ciji = " << std::endl << Ciji << std::endl;

    return hiji.transpose().matrix() * Ciji.inverse() * hiji.matrix();
    
}

JCBB::hypothesis JCBB::run()
{
    jcbb(best_, 0);
    return best_;
}

double JCBB::chi2inv(float a, int dof)
{
    boost::math::chi_squared distro(dof);
    return boost::math::quantile(distro, a);
}

Eigen::ArrayXd JCBB::constructErr(JCBB::hypothesis H)
{

    Eigen::ArrayXd x(2 * H.n_matches());
    Eigen::ArrayXd y(2 * H.n_matches());

    // We order values as [bearing1, range1, bearing2, range2... etc]
    // Start with vectors of indices
    for (int i = 0; i < H.n_matches(); i++)
    {
        x(2 * i) = landmarks_[H.landmark_indices[i]].bearing;
        x(2 * i + 1) = landmarks_[H.landmark_indices[i]].range;

        y(2 * i) = features_[H.feature_indices[i]].bearing;
        y(2 * i + 1) = features_[H.feature_indices[i]].range;
    }

    return x - y;
}