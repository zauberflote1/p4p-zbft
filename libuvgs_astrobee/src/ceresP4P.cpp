/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-08-09 00:36:15
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-08-12 12:53:34
 * @ Description:
 * COBRAS FUMANTES
 * CERES P4P IMPLEMENTATION OUTER CLASS
 * 
 */


#include "carolus_node/ceresP4P.hpp"


#include "carolus_node/ceresP4P.hpp"

void CobrasFumantes::computeAndValidatePosesWithRefinement(
    const std::vector<Eigen::Vector3d>& sortedImagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const std::vector<cv::Point2f>& undistortedPoints,
    lambdatwist::CameraPose& bestPose) const
{
    // SET UP CERES PROBLEM
    ceres::Problem problem;

    // SETUP INITIAL GUESS
    double camera_params[6] = {0.000, 0.000, -0.001, 0.0, 0.0, 0.7};  // Changed size to 6, as we are using only 6 parameters

    double focalX_length = cameraMatrix_.at<double>(0, 0);  // FX
    double focalY_length = cameraMatrix_.at<double>(1, 1);  // FY
    double cx = cameraMatrix_.at<double>(0, 2);
    double cy = cameraMatrix_.at<double>(1, 2);

    // TEMP VECTORS
    Eigen::Vector2d observed_point;
    Eigen::Vector3d target_point;

    if (measType_ == 1) { // AUTODIFF
        for (size_t i = 0; i < sortedImagePoints.size(); ++i) {
            observed_point = sortedImagePoints[i].head<2>();
            target_point = knownPoints[i];

            auto cost_function = std::make_unique<ceres::AutoDiffCostFunction<ReprojectionErrorWithAutoDiff, 2, 6>>(
                new ReprojectionErrorWithAutoDiff(observed_point, target_point, focalX_length, focalY_length, cx, cy));
            problem.AddResidualBlock(cost_function.release(), nullptr, camera_params);
        }
    } else if (measType_ == 2) { // ANALYTIC 
        for (size_t i = 0; i < sortedImagePoints.size(); ++i) {
            observed_point = sortedImagePoints[i].head<2>();
            target_point = knownPoints[i];

            auto cost_function = ReprojectionErrorWithAnalyticDiff::Create(observed_point, target_point, focalX_length, focalY_length, cx, cy);
            problem.AddResidualBlock(cost_function, nullptr, camera_params);
        }
    } else if (measType_ == 3) { // NUMERICDIFF
        for (size_t i = 0; i < sortedImagePoints.size(); ++i) {
            observed_point = sortedImagePoints[i].head<2>();
            target_point = knownPoints[i];

            auto cost_function = std::make_unique<ceres::NumericDiffCostFunction<ReprojectionErrorWithNumericDiff, ceres::CENTRAL, 2, 6>>(
                new ReprojectionErrorWithNumericDiff(observed_point, target_point, focalX_length, focalY_length, cx, cy));
            problem.AddResidualBlock(cost_function.release(), nullptr, camera_params);
        }
    }

    // SET UP SOLVER OPTIONS
    ceres::Solver::Options options;
    options.trust_region_strategy_type = ceres::DOGLEG; // SEEMS BETTER THAN LEVENBERG-MARQUARDT
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_SCHUR; // FEEL FREE TO TRY DENSE_QR
    options.minimizer_progress_to_stdout = false;

    // SOLVE IT!
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // UPDATE BEST POSE
    bestPose.R = computeRotationMatrix(camera_params[3], camera_params[4], camera_params[5]);
    bestPose.t = Eigen::Vector3d(camera_params[0], camera_params[1], camera_params[2]);
}
