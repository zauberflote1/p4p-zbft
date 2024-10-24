/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 00:53:33
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-07-01 21:46:13
 * @ Description: POSE ESTIMATION IMPLEMENTATION USING 4 POINTS 
 */

#include "carolus_node/pose_est.hpp"
#include <algorithm>
#include <numeric>

namespace {
//SORT OF AN ADAPTION OF NASA ORIGINAL GEOMETRY SORTING, AS THE ORIGINAL IT DOESN'T WORK...
auto calculatePairwiseDistances(const std::vector<Eigen::Vector3d>& points) {
    std::vector<std::tuple<int, int, double>> distances;
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            distances.emplace_back(i, j, (points[i] - points[j]).norm());
        }
    }
    return distances;
}

std::vector<int> correlatePoints(const std::vector<Eigen::Vector3d>& imagePoints, const std::vector<Eigen::Vector3d>& knownPoints) {
    auto imageDistances = calculatePairwiseDistances(imagePoints);
    auto knownDistances = calculatePairwiseDistances(knownPoints);


    std::sort(imageDistances.begin(), imageDistances.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
    });

    std::sort(knownDistances.begin(), knownDistances.end(), [](const auto& a, const auto& b) {
        return std::get<2>(a) < std::get<2>(b);
    });


    std::vector<int> mapping(imagePoints.size(), -1);
    for (size_t i = 0; i < imageDistances.size(); ++i) {
        int imgIndex1 = std::get<0>(imageDistances[i]);
        int imgIndex2 = std::get<1>(imageDistances[i]);
        int knownIndex1 = std::get<0>(knownDistances[i]);
        int knownIndex2 = std::get<1>(knownDistances[i]);

        if (mapping[imgIndex1] == -1) mapping[imgIndex1] = knownIndex1;
        if (mapping[imgIndex2] == -1) mapping[imgIndex2] = knownIndex2;
    }

    return mapping;
}

inline Eigen::Vector3d FindMidpoint(const std::vector<Eigen::Vector3d>& candidate_target_list, const uint8_t* p1p2) {
    return (candidate_target_list[p1p2[0]] + candidate_target_list[p1p2[1]]) / 2.0;
}

inline void Midpoint2P3P4(const Eigen::Vector3d& midpoint, const std::vector<Eigen::Vector3d>& candidate_target_list, const uint8_t* p3p4, double* short_lengths) {
    short_lengths[0] = (midpoint - candidate_target_list[p3p4[0]]).norm();
    short_lengths[1] = (midpoint - candidate_target_list[p3p4[1]]).norm();
}

bool FindP1P2Indices(const double* v_p3p4, const double* v_p3pa, const double* v_p3pb, const uint8_t* p1p2, uint8_t* p1, uint8_t* p2) {
    double tmp0[3] = {0}; 
    bool success = false;

    // sign(sum(cross(P3P4,P3Pa)));
    cross(v_p3p4, v_p3pa, tmp0);
    double pax = signum(std::accumulate(std::begin(tmp0), std::end(tmp0), 0.0));

    // sign(sum(cross(P3P4,P3Pb)));
    cross(v_p3p4, v_p3pb, tmp0);
    double pbx = signum(std::accumulate(std::begin(tmp0), std::end(tmp0), 0.0));

    if (pax * pbx == -1.) {
        if (pax == 1.) { // pa = p2, pb = p1
            *p1 = p1p2[1];
            *p2 = p1p2[0];
            success = true;
        } else { // pa = p1, pb = p2
            *p1 = p1p2[0];
            *p2 = p1p2[1];
            success = true;
        }
    }

    return success;
}

} // namespace

bool SortTargetsUsingTetrahedronGeometry(const std::vector<Eigen::Vector3d>& candidate_target_list, std::vector<Eigen::Vector3d>& targets_out) {
    const uint8_t idx_lookup_table[6][2] =  {
        {0,1},
        {0,2},
        {0,3},
        {1,2},
        {1,3},
        {2,3},
    };

    const uint8_t not_idx_lookup_table[6][2] =  {
        {2,3},
        {1,3},
        {1,2},
        {0,3},
        {0,2},
        {0,1},
    };

    bool is_sorted = false; 
    double lengths[6];
    for (int i = 0; i < 6; ++i) {
        lengths[i] = (candidate_target_list[idx_lookup_table[i][0]] - candidate_target_list[idx_lookup_table[i][1]]).norm();
    }
    const uint8_t p1p2_table_idx = std::distance(lengths, std::max_element(lengths, lengths + 6));
    const uint8_t* p1p2 = idx_lookup_table[p1p2_table_idx];
    Eigen::Vector3d midpoint = FindMidpoint(candidate_target_list, p1p2);
    const uint8_t* p3p4 = not_idx_lookup_table[p1p2_table_idx];

    double short_lengths[2];
    Midpoint2P3P4(midpoint, candidate_target_list, p3p4, short_lengths);

    uint8_t p3, p4;
    if (short_lengths[0] < short_lengths[1]) {
        p3 = p3p4[0];
        p4 = p3p4[1];
    } else {
        p3 = p3p4[1];
        p4 = p3p4[0];
    }

    double v_p3p4[3] = {
        candidate_target_list[p3].x() - candidate_target_list[p4].x(),
        candidate_target_list[p3].y() - candidate_target_list[p4].y(),
        0.0
    };

    double v_p3pa[3] = {
        candidate_target_list[p3].x() - candidate_target_list[p1p2[0]].x(),
        candidate_target_list[p3].y() - candidate_target_list[p1p2[0]].y(),
        0.0
    };

    double v_p3pb[3] = {
        candidate_target_list[p3].x() - candidate_target_list[p1p2[1]].x(),
        candidate_target_list[p3].y() - candidate_target_list[p1p2[1]].y(),
        0.0
    };

    uint8_t p1, p2;
    bool success = FindP1P2Indices(v_p3p4, v_p3pa, v_p3pb, p1p2, &p1, &p2);

    if (success) {
        targets_out[0] = candidate_target_list[p1];
        targets_out[1] = candidate_target_list[p2];
        targets_out[2] = candidate_target_list[p3];
        targets_out[3] = candidate_target_list[p4];
        is_sorted = true;
    }

    return is_sorted;
}

void computeAndValidatePoses(
    const std::vector<Eigen::Vector3d>& imagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::Point2f>& undistortedPoints,
    CameraPose& bestPose,
    double& minError)
{

    std::vector<CameraPose> poses;
    //P3P LAMBDA-TWIST
    int numSolutions = p3p(imagePoints, knownPoints, &poses);

 
    for (int i = 0; i < numSolutions; ++i) {
        const CameraPose& pose = poses[i];

        //TARGET CENTERED, FOLLOW RIGHT-HAND RULE, Y>0
        // if (pose.t[1] < 0) {
        //     continue; 
        // }

        double totalError = 0.0;
        for (size_t j = 0; j < knownPoints.size(); ++j) {
            Eigen::Vector3d projectedPoint = pose.R * knownPoints[j] + pose.t;
            projectedPoint /= projectedPoint.z();
            Eigen::Vector2d projectedImagePoint(projectedPoint.x(), projectedPoint.y());
            projectedImagePoint.x() = projectedImagePoint.x() * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2);
            projectedImagePoint.y() = projectedImagePoint.y() * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2);
            totalError += (projectedImagePoint - Eigen::Vector2d(undistortedPoints[j].x, undistortedPoints[j].y)).norm();
        }
        if (totalError < minError) {
            minError = totalError;
            bestPose = pose;
        }
    }
}
//ATTEMPT TO MINIMIZE ERROR BY ADDING FOURTH POINT
//NEEDS CHANGES...
void refinePoseWithFourthPoint(
    const CameraPose& initialPose,
    const Eigen::Vector3d& fourthImagePoint,
    const Eigen::Vector3d& fourthKnownPoint,
    const cv::Mat& cameraMatrix,
    const cv::Point2f& undistortedFourthPoint,
    CameraPose& refinedPose,
    double& minError)
{
  
    Eigen::Vector3d projectedPoint = initialPose.R * fourthKnownPoint + initialPose.t;
    projectedPoint /= projectedPoint.z();
    Eigen::Vector2d projectedImagePoint(projectedPoint.x(), projectedPoint.y());
    projectedImagePoint.x() = projectedImagePoint.x() * cameraMatrix.at<double>(0, 0) + cameraMatrix.at<double>(0, 2);
    projectedImagePoint.y() = projectedImagePoint.y() * cameraMatrix.at<double>(1, 1) + cameraMatrix.at<double>(1, 2);


    double error = (projectedImagePoint - Eigen::Vector2d(undistortedFourthPoint.x, undistortedFourthPoint.y)).norm();


    if (error < minError) {
        minError = error;
        refinedPose = initialPose;
    }
}

void computeAndValidatePosesWithRefinement(
    const std::vector<Eigen::Vector3d>& imagePoints,
    const std::vector<Eigen::Vector3d>& knownPoints,
    const cv::Mat& cameraMatrix,
    const std::vector<cv::Point2f>& undistortedPoints,
    CameraPose& bestPose)
{
    double minError = std::numeric_limits<double>::max();

    //SPAN COMBINATIONS
    for (int i = 0; i < 4; ++i) {
        std::vector<Eigen::Vector3d> subImagePoints;
        std::vector<Eigen::Vector3d> subKnownPoints;
        for (int j = 0; j < 4; ++j) {
            if (i != j) {
                subImagePoints.push_back(imagePoints[j]);
                subKnownPoints.push_back(knownPoints[j]);
            }
        }
        CameraPose initialPose;
        computeAndValidatePoses(subImagePoints, subKnownPoints, cameraMatrix, undistortedPoints, initialPose, minError);

        //REFINE WITH FOURTH POINT
        refinePoseWithFourthPoint(initialPose, imagePoints[i], knownPoints[i], cameraMatrix, undistortedPoints[i], bestPose, minError);
    }
}
