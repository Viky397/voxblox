// Author: Keenan Burnett
#include <iostream>
#include <vector>
#include "utils/hmm.hpp"

namespace kalman {

static double normalize(Eigen::MatrixXd &x) {
    double z = 0;
    double eps = 1e-15;
    for (uint i = 0; i < x.rows(); i++) {
        z += x(i, 0);
    }
    x /= (z + eps);
    return z;
}

static int convert_from_det_type(int det_type, std::vector<int> types) {
    auto it = std::find(types.begin(), types.end(), det_type);
    if (it != types.end()) {
        return it - types.begin();
    } else {
        std::cout << "WARNING: det_type was not expected in HMM" << std::endl;
        return 0;
    }
}

void HiddenMarkovModel::filter(int det_type, std::vector<float> class_confidences) {
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(num_types, 1);
    int hmm_type = convert_from_det_type(det_type, types);
    if (class_confidences.size() == 1) {
        x(hmm_type, 0) = class_confidences[0];
        float remainder = (1 - class_confidences[0]) / (num_types - 1);
        for (int i = 0; i < num_types; i++) {
            if (i == hmm_type)
                continue;
            x(i, 0) = remainder;
        }
    } else {
        if ((int)class_confidences.size() != num_types)
            std::cout << "WARNING: class_confidences is different from expected type vector length" << std::endl;
        for (uint i = 0; i < class_confidences.size(); i++) {
            x(i, 0) = class_confidences[i];
        }
    }
    x = C * x;
    if (!init) {
        alpha = x.cwiseProduct(pi);
        log_likelihoods.push_back(normalize(alpha));
        init = true;
        return;
    }
    alpha = x.cwiseProduct(A * alpha);
    log_likelihoods.push_back(normalize(alpha));
    if (log_likelihoods.size() > averaging_period)
        log_likelihoods.pop_front();
}

// Returns most likely type based on HMM filtering
int HiddenMarkovModel::get_type() {
    double max = 0;
    int largest = 0;
    for (int i = 0; i < num_types; i++) {
        if (alpha(i, 0) > max) {
            max = alpha(i, 0);
            largest = i;
        }
    }
    return types[largest];
}

// Returns log likelihood of data given latent states and parameters
// This value is averaged over the averaging_period
double HiddenMarkovModel::get_log_likelihood() {
    double sum = 0;
    for (uint i = 0; i < log_likelihoods.size(); i++) {
        sum += log_likelihoods[i];
    }
    return sum / (double)log_likelihoods.size();
}
}  // namespace kalman
