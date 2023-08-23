#ifndef GMM_NODE_HPP
#define GMM_NODE_HPP

std::vector<float> doRegression(float input, Eigen::MatrixXf  xi, const std::vector<Eigen::VectorXf>  means, 
                                const std::vector<float> weights, const std::vector<Eigen::MatrixXf>  covariances);

void getGMMfromBag(Eigen::MatrixXf &GMM_X, std::vector<Eigen::VectorXf> &GMM_means, std::vector<float> &GMM_weights, std::vector<Eigen::MatrixXf> &GMM_covariances);

#endif // GMM_NODE_HPP