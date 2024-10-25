/**
 * @ Author: zauberflote1
 * @ Create Time: 2024-06-28 00:53:33
 * @ Modified by: zauberflote1
 * @ Modified time: 2024-10-24 22:08:14
 * @ Description:
 * POSE ESTIMATION NODE FROM A 4 POINT TARGET NODE USING ROS
 * (NOT USING CV_BRIDGE AS IT MAY NOT BE COMPATIBLE WITH RESOURCE CONSTRAINED/CUSTOMS SYSTEMS)
 * 
 * ---------------------------------------------------------
 * SAY MY NAME WHEN YOU PRAY TO THE SKIES, SEE CAROLUS RISE
 * ---------------------------------------------------------
 * 
 */

#include <ros/ros.h>

#include <chrono>
#include <vector>
#include <iostream>
#include <optional>
#include <thread>
#include <future>
#include <mutex>
#include <queue>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <condition_variable>
#include <atomic>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "carolus_node/carolus_types.hpp"
#include "carolus_node/pose_est.hpp"
#include "carolus_node/ceresP4P.hpp"

class CarolusRexNode {
public:
    CarolusRexNode(ros::NodeHandle& nh) :
        nh_(nh),
        image_transport_(nh_),
        num_threads_(1),
        min_circularity_(0.4),
        saturation_threshold_(15),
        keep_running_(true),
        //INITIALIZE THE QUEUE PARAMS
        max_queue_size_(10), //PLAY WITH THIS NUMBER BASED ON MEMORY USAGE
        curr_queue_size_(0)

    {
        //LOAD USER PREFERENCES AND PARAMETERS

        //CONTROL PARAMETERS
        nh_.param("num_threads", num_threads_, 1);
        nh_.param("min_circularity", min_circularity_, 0.5);

        //BEACON POINTS
        std::vector<double> known_points_vector;
        if (nh_.getParam("known_points", known_points_vector)) {
            if (known_points_vector.size() % 3 != 0) {
                ROS_ERROR("Invalid known points size, expected multiple of 3");
            } else {
                knownPoints_.clear();
                for (size_t i = 0; i < known_points_vector.size(); i += 3) {
                    knownPoints_.emplace_back(known_points_vector[i], known_points_vector[i+1], known_points_vector[i+2]);
                }
            }
        } else {
            //DEFAULT SMALL TARGETS
            knownPoints_ = {
                {0.055, 0.0, 0.0},
                {-0.055, 0.0, 0.0},
                {0.0, 0.048, 0.0},
                {0.0, 0.0, 0.037}
            };
        }

        //CAMERA PROPERTIES (PINHOLE MODEL)
        double fx, fy, cx, cy;
        std::vector<double> distCoeffs_vector;
       //D455 STRAIGHT FROM KALIBR
        nh_.param("fx", fx, 423.84596179);
        nh_.param("fy", fy, 422.96425442);
        nh_.param("cx", cx, 423.75095666);
        nh_.param("cy", cy, 248.55000177);
        nh_.getParam("distortion", distCoeffs_vector);

        if (distCoeffs_vector.size() != 4) {
            ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
            distCoeffs_vector = {-0.04360337, 0.03103359, -0.00098949, 0.00150547};
        }

        // if (distCoeffs_vector.size() != 4) {
        //     ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
        //     distCoeffs_vector = {-0.02701573, 0.02348154, -0.00106455, -0.00364093};
        // }
        // nh_.param("fx", fx, 875.9435630126997);
        // nh_.param("fy", fy, 875.5638319050095);
        // nh_.param("cx", cx, 576.0009986724265);
        // nh_.param("cy", cy, 342.73010114675753);
        // nh_.getParam("distortion", distCoeffs_vector);

        // if (distCoeffs_vector.size() != 4) {
        //     ROS_ERROR("Invalid distortion coefficients size, expected 4 elements, using default values.");
        //     distCoeffs_vector = {0.0, 0.0, 0.0, 0.0};
        // }

        //CONSTRUCT CAMERA MATRIX AND DISTORTION COEFFICIENTS USING OPENCV FORMAT
        cameraMatrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx,
                                                0, fy, cy,
                                                0, 0, 1);

        distCoeffs_ = (cv::Mat_<double>(1, 4) << distCoeffs_vector[0], distCoeffs_vector[1], distCoeffs_vector[2], distCoeffs_vector[3]);

        //SETUP SUBSCRIBER AND PUBLISHERS
        //TEMP: LEAVING AS DEFAULT, CANNOT BE MODIFIED IN THE LAUNCH FILE 
        //      EITHER MODIFY THE CODE OR REMAP THE TOPICS FOR NOW

        //TODO: ADD NODLET OPTION TO MODIFY TOPICS AND LAUNCH MULTIPLE INSTANCES OF CAROLUSREXNODE
        image_sub_ = image_transport_.subscribe("/d455/color/image_raw", 10, &CarolusRexNode::imageCallback, this);
        image_pub_ = image_transport_.advertise("/postprocessed/image", 10);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/pose", 10);

        process_thread_ = std::thread(&CarolusRexNode::processImages, this);
        ROS_INFO("============================================");
        ROS_INFO("A P4P POSE ESTIMATION NODE");
        ROS_INFO("@1822 TROPICAL EMPIRE. All rights reserved.");
        ROS_INFO("============================================");
    }

    ~CarolusRexNode() {
        keep_running_ = false;
        cv_.notify_all();
        if (process_thread_.joinable()) {
            process_thread_.join();
        }
    }

private:
    cv::Mat preprocessImage(const cv::Mat& image) {
        //TODO: REORGANIZE THIS FOR BETTER PERFORMANCE AND USER SELECTION OF PREPROCESSING STEPS
        cv::Mat blurred, thresholded;

        //D455 GOLDEN STANDARD?
        cv::GaussianBlur(image, blurred, cv::Size(3, 3), 1);
        double threshValue = 240;
        cv::threshold(blurred, thresholded, threshValue, 255, cv::THRESH_BINARY);
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 2);
        cv::morphologyEx(thresholded, thresholded, cv::MORPH_DILATE, morph_kernel, cv::Point(-1, -1), 1);
        return thresholded;



        // double threshValue1 = 220;
        // cv::threshold(image, thresholded, threshValue1, 255, cv::THRESH_BINARY);
        // cv::equalizeHist(image, image);
        // double eq_clip_limit = 10.0;
        //   cv::Size eq_win_size = cv::Size(8, 8);
        //   cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(eq_clip_limit, eq_win_size);
        //   clahe->apply(image, image);
        //     cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
        // cv::equalizeHist(image, image);
        // cv::GaussianBlur(image, image, cv::Size(3, 3), 1);
        // double threshValue = 240;
        // cv::threshold(image, thresholded, threshValue, 255, cv::THRESH_BINARY);


        // // double threshValue = 240;
        // // cv::threshold(image, thresholded, threshValue, 255, cv::THRESH_BINARY);
        // // cv::GaussianBlur(thresholded, blurred, cv::Size(3, 3), 0);

        // cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        // // cv::Mat morph_kernel2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, morph_kernel, cv::Point(-1, -1), 0);
        // cv::morphologyEx(thresholded, thresholded, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 2);
        // // // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_OPEN, morph_kernel, cv::Point(-1, -1), 0);
        
        // // cv::morphologyEx(thresholded, thresholded, cv::MORPH_DILATE, morph_kernel, cv::Point(-1, -1), 1);
        // return thresholded;
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (curr_queue_size_.load() >= max_queue_size_) {
            ROS_WARN("Image queue is full, dropping oldest image.");
            producer_image_queue_.pop();
            producer_image_queue_.push(msg);
        } else {
            producer_image_queue_.push(msg);
            curr_queue_size_.fetch_add(1, std::memory_order_relaxed);
            }
        }
        //NOW NOTIFY THE PROCESSING THREAD
        cv_.notify_one();
    }

    void processImages() {
        while (ros::ok() && keep_running_) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            //WAIT FOR NEW IMAGE
            cv_.wait(lock, [this] { return !producer_image_queue_.empty() || !keep_running_; });
            if (!keep_running_) break;
            
            //ATOMICALY SWAP QUEUES
            std::swap(consumer_image_queue_, producer_image_queue_);
            //NOW UNLOCK
            lock.unlock();
            
            //PROCESS ALL IMAGES IN THE QUEUE
            while (!consumer_image_queue_.empty()) {
                auto msg = consumer_image_queue_.front();
                consumer_image_queue_.pop();
                curr_queue_size_.fetch_sub(1, std::memory_order_relaxed);

                //COLLECT TIMESTAMP
                auto timestamp = msg->header.stamp;
                cv::Mat image = convertImageMessageToMat(msg);

                if (image.empty()) {
                    ROS_ERROR("Failed to convert image message to cv::Mat. RGB8, BGR8 or MONO8 encoding expected.");
                    continue;
                }
            //TODO: ADD COMPILATION FLAG TO ENABLE/DISABLE COLOR PROCESSING
                //BEGIN PREPROCESSING
                cv::Mat imageMono;
                if (image.channels() == 3) {
                    cv::cvtColor(image, imageMono, cv::COLOR_BGR2GRAY);
                    //CONVERT ORIGINAL TO HSV
                    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                } else { //HOT FIX FOR MONO8 IMAGES
                    imageMono = image.clone();
                    cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
                    cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
                }
                cv::Mat preprocessedImage = preprocessImage(imageMono);

                //BEGIN BLOB DETECTION AND PROCESSING
                auto blobCarolusVec = findAndCalcContours(preprocessedImage, image, num_threads_);
                if (blobCarolusVec) {
                    std::vector<BlobCarolus> best_blobs = selectBlobs(blobCarolusVec.value(), min_circularity_);
                    //IF FOUND BLOBS, DRAW THEM ON THE IMAGE AND PUBLISH
                    if (!best_blobs.empty()) {
                        cv::Mat coloredPreprocessedImage;
                        cv::cvtColor(preprocessedImage, coloredPreprocessedImage, cv::COLOR_GRAY2BGR);
                        //GET HUE COLOR IN BGR FOR EACH BLOB
                        for (const auto& blob : best_blobs) {
                                double hue = blob.properties.hue;
                                cv::Mat hsvColor(1, 1, CV_8UC3, cv::Scalar(hue, 255, 255)); // Full saturation and value
                                cv::Mat bgrColor;
                                cv::cvtColor(hsvColor, bgrColor, cv::COLOR_HSV2BGR);
                                cv::Vec3b bgr = bgrColor.at<cv::Vec3b>(0, 0);

                                //OPENCV CONVERSION
                                cv::Scalar color(bgr[0], bgr[1], bgr[2]);

                                // ROS_INFO("HUE: %f", blob.properties.hue);
                                // ROS_INFO("Area: %f", blob.properties.m00);
                                cv::circle(coloredPreprocessedImage, cv::Point(blob.blob.x, blob.blob.y), 5, color, -1);
                        }

                        auto resultIMG_msg = convertMatToImageMessage(coloredPreprocessedImage, msg->header);
                        // ROS_INFO("Publishing processed image...");
                        image_pub_.publish(resultIMG_msg);

                        std::vector<Blob> blobs;
                        blobs.reserve(best_blobs.size());
                        for (const auto& blobCarolus : best_blobs) {
                            blobs.emplace_back(blobCarolus.blob);
                        }
                        processBlobs(blobs, timestamp);
                    }
                } else {
                    ROS_INFO("No valid contours found.");
                }
                //RELEASE IMAGE MEMORY
                image.release();
                imageMono.release();
                preprocessedImage.release();
            }
        }
    }
    CameraPose getFilteredPose(const CameraPose& new_pose) {
    if (!pose_queue_.empty()) {
        const CameraPose& last_pose = pose_queue_.back();
        double translation_diff = (last_pose.t - new_pose.t).norm();
        if (translation_diff > translation_threshold_) {
            ROS_WARN("Current pose is significantly different from the previous pose. Ignoring current pose.");
            return last_pose;
        }
    }

    pose_queue_.push_back(new_pose);
    if (pose_queue_.size() > filter_size_) {
        pose_queue_.pop_front();
    }

    Eigen::Matrix3d avg_R = Eigen::Matrix3d::Zero();
    Eigen::Vector3d avg_t = Eigen::Vector3d::Zero();
    for (const auto& pose : pose_queue_) {
        avg_R += pose.R;
        avg_t += pose.t;
    }
    avg_R /= pose_queue_.size();
    avg_t /= pose_queue_.size();

    CameraPose filtered_pose;
    filtered_pose.R = avg_R;
    filtered_pose.t = avg_t;
    return filtered_pose;
}
    std::deque<CameraPose> pose_queue_;
    const size_t filter_size_ = 7; 
    const double translation_threshold_ = 0.5; 

    

std::optional<std::vector<BlobCarolus>> findAndCalcContours(const cv::Mat &image, const cv::Mat &originalImageHSV, int num_threads) {
    std::vector<std::vector<cv::Point>> contours;

    auto start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS START
    //FIND CONTOURS IN THE IMAGE
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT COUNTOURS END
    std::chrono::duration<double> duration = end - start;
    ROS_INFO("Time to find contours: %f seconds", duration.count());

    //CHECK IF CONTOURS ARE VALID
    if (contours.empty() || contours.size() > 1000) {
        ROS_ERROR("ERROR: %lu contours found", contours.size());
        return std::nullopt;
    }

    //PREPARE FOR BLOB PROPERTIES CALCULATION
    std::vector<BlobCarolus> blobs;
    blobs.reserve(contours.size());

    start = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS START

    //PARALLELIZE BLOB CALCULATIONS
    std::vector<std::future<void>> futures;
    std::mutex blobs_mutex;
    int contours_per_thread = std::max(1, static_cast<int>(contours.size() / num_threads));
    int remainder = contours.size() % num_threads;

    for (int t = 0; t < num_threads; ++t) {
        futures.emplace_back(std::async(std::launch::async, [this, t, contours_per_thread, remainder, &contours, &blobs, &blobs_mutex, &originalImageHSV]() {
            int start_idx = t * contours_per_thread + std::min(t, remainder);
            int end_idx = start_idx + contours_per_thread + (t < remainder ? 1 : 0);

            for (int i = start_idx; i < end_idx; ++i) {
                const auto &contour = contours[i];

                //CHECK AREA OF CONTOUR (m00)
                cv::Moments moments = cv::moments(contour);
                if (moments.m00 < 30|| moments.m00 > 2500) {
                    continue;
                }
                //CALCULATE BLOB PROPERTIES
                double perimeter = cv::arcLength(contour, true);
                double circularity = (4 * CV_PI * moments.m00) / (perimeter * perimeter);
                double x = moments.m10 / moments.m00; //CENTROID X
                double y = moments.m01 / moments.m00; //CENTROID Y
                
                //HUE EXTRACTION PROCESS
                //TODO: ADD CIRCULAR MEAN ALGORITHM FOR HUE EXTRACTION AND CHECK BEHAVIOR WHEN DEALING WITH MONO8->HSV
                //BOUND THE CONTOUR
                cv::Rect boundingRect = cv::boundingRect(contour);
                cv::Mat BlobRegion = originalImageHSV(boundingRect);
                //DECLARE HSV MASK TO FILTER SATURATION
                cv::Mat maskHSV;
                //FILTER OUT BAD PIXELS
                cv::inRange(BlobRegion, cv::Scalar(0, saturation_threshold_, 0), cv::Scalar(180, 255, 255), maskHSV);
                //CALCULATE HUE --> meanHSV[0] 0-180
                cv::Scalar meanHSV = cv::mean(BlobRegion, maskHSV);
                // if (meanHSV[0] < 94 && meanHSV[0] > 92) {
                //     continue;
                // }
                if (meanHSV[0] < 145 || meanHSV[0] > 160) {
                    continue;
                }
                //CHECK HUE CONDITIONS PER BLOB COLOR


                BlobCarolus blobCarolus;
                blobCarolus.blob = {x, y};
                blobCarolus.properties = {perimeter, moments.m00, circularity, meanHSV[0], boundingRect};

                std::lock_guard<std::mutex> lock(blobs_mutex);
                blobs.emplace_back(std::move(blobCarolus));
            }
        }));
    }

    for (auto &fut : futures) {
        fut.get();
    }

    end = std::chrono::high_resolution_clock::now(); //TIME MEASUREMENT BLOBS END
    duration = end - start;
    // ROS_INFO("Calc time: %f seconds", duration.count());

    return blobs;
}

//SELECT BLOBS BASED ON CIRCULARITY AND VARIANCE --> NEEDS CLEANING AND OPTIMIZATION
std::vector<BlobCarolus> selectBlobs(const std::vector<BlobCarolus>& blobs, double min_circularity) {
    std::vector<BlobCarolus> filtered_blobs;
    filtered_blobs.reserve(blobs.size());

    if (blobs.size() < 4) {
        ROS_ERROR("Not enough blobs < 4.");
        return {};
    }

    for (const auto& blob : blobs) {
        if (blob.properties.circularity >= min_circularity) {
            filtered_blobs.emplace_back(blob);
        }
    }

    if (filtered_blobs.size() < 4) {
        ROS_ERROR("Not enough blobs with required circularity.");
        return {};
    }

    double min_variation = std::numeric_limits<double>::max();
    std::vector<BlobCarolus> best_group;

    for (size_t i = 0; i < filtered_blobs.size() - 3; ++i) {
        for (size_t j = i + 1; j < filtered_blobs.size() - 2; ++j) {
            for (size_t k = j + 1; k < filtered_blobs.size() - 1; ++k) {
                for (size_t l = k + 1; l < filtered_blobs.size(); ++l) {

                    
                    double dist_ij = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[j].blob.x, filtered_blobs[i].blob.y - filtered_blobs[j].blob.y);
                    double dist_ik = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[k].blob.x, filtered_blobs[i].blob.y - filtered_blobs[k].blob.y);
                    double dist_il = std::hypot(filtered_blobs[i].blob.x - filtered_blobs[l].blob.x, filtered_blobs[i].blob.y - filtered_blobs[l].blob.y);
                    double dist_jk = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[k].blob.x, filtered_blobs[j].blob.y - filtered_blobs[k].blob.y);
                    double dist_jl = std::hypot(filtered_blobs[j].blob.x - filtered_blobs[l].blob.x, filtered_blobs[j].blob.y - filtered_blobs[l].blob.y);
                    double dist_kl = std::hypot(filtered_blobs[k].blob.x - filtered_blobs[l].blob.x, filtered_blobs[k].blob.y - filtered_blobs[l].blob.y);

                    std::vector<double> distances = {dist_ij, dist_ik, dist_il, dist_jk, dist_jl, dist_kl};
                    double max_distance = *std::max_element(distances.begin(), distances.end());
                    double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
                    double distance_variance = std::accumulate(distances.begin(), distances.end(), 0.0,
                        [mean_distance](double sum, double distance) {
                            return sum + std::pow(distance - mean_distance, 2);
                        }) / distances.size();

                    
                    if (max_distance > 500) {
                        continue;
                    }

                    
                    std::vector<double> areas = {
                        filtered_blobs[i].properties.m00,
                        filtered_blobs[j].properties.m00,
                        filtered_blobs[k].properties.m00,
                        filtered_blobs[l].properties.m00
                    };
                    double mean_area = std::accumulate(areas.begin(), areas.end(), 0.0) / areas.size();
                    double area_variance = std::accumulate(areas.begin(), areas.end(), 0.0,
                        [mean_area](double sum, double area) {
                            return sum + std::pow(area - mean_area, 2);
                        }) / areas.size();


                    std::vector<double> hues = {
                        filtered_blobs[i].properties.hue,
                        filtered_blobs[j].properties.hue,
                        filtered_blobs[k].properties.hue,
                        filtered_blobs[l].properties.hue
                    };
                    double mean_hue = std::accumulate(hues.begin(), hues.end(), 0.0) / areas.size();
                    double intensity_hues = std::accumulate(hues.begin(), hues.end(), 0.0,
                        [mean_hue](double sum, double hues) {
                            return sum + std::pow(hues - mean_hue, 2);
                        }) / hues.size();

                    double combined_variance = distance_variance + 1.5*area_variance; //+ intensity_hues;

                    if (combined_variance < min_variation) {
                        min_variation = combined_variance;
                        best_group = {filtered_blobs[i], filtered_blobs[j], filtered_blobs[k], filtered_blobs[l]};
                    }
                }
            }
        }
    }

    return best_group;
}


 void processBlobs(const std::vector<Blob>& blobs, const ros::Time& timestamp) {
    if (blobs.size() < 4) {
        ROS_ERROR("Not enough blobs to calculate 6DoF state.");
        return;
    }

    std::vector<cv::Point2f> distortedPoints;
    distortedPoints.reserve(blobs.size());
    for (const auto& blob : blobs) {
        distortedPoints.emplace_back(blob.x, blob.y);
    }

    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(distortedPoints, undistortedPoints, cameraMatrix_, distCoeffs_);

    //UNDISTORTED POINTS ARE NORMALIZED, CONVERT BACK TO ORIGINAL IMAGE SPACE
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double cx = cameraMatrix_.at<double>(0, 2);
    double cy = cameraMatrix_.at<double>(1, 2);
    
 

    std::vector<Eigen::Vector3d> imagePoints;
    imagePoints.reserve(undistortedPoints.size());
    for (const auto& point : undistortedPoints) {
        imagePoints.emplace_back(Eigen::Vector3d(point.x, point.y, 1.0).normalized());
    }

    std::vector<Eigen::Vector3d> sortedImagePoints(4);
    bool success = SortTargetsUsingTetrahedronGeometry(imagePoints, sortedImagePoints);

    for (int i = 0; i < sortedImagePoints.size(); i++) {
        sortedImagePoints[i](0) = sortedImagePoints[i](0) * fx +cx;
        sortedImagePoints[i](1) = sortedImagePoints[i](1) * fy +cy;
    }
   

    // if (!success) {
    //     ROS_ERROR("Failed to sort targets using tetrahedron geometry.");
    //     return;
    // }

    CameraPose bestPose;
    int measType_ = 2;
    //COBRAS FUMANTES POSE SOLVER
    //THE SNAKE IS GOING TO SMOKE
    CobrasFumantes poseSolver(cameraMatrix_, measType_);
    poseSolver.computeAndValidatePosesWithRefinement(sortedImagePoints, knownPoints_, undistortedPoints, bestPose);

    if (bestPose.R.allFinite() && bestPose.t.allFinite()) {
        CameraPose filteredPose = getFilteredPose(bestPose);
        
        std::stringstream ssR;
        ssR << bestPose.R.format(Eigen::IOFormat());
        std::stringstream sst;
        sst << bestPose.t.transpose().format(Eigen::IOFormat());

        ROS_INFO("Filtered Rotation matrix R:\n%s", ssR.str().c_str());
        ROS_INFO("Filtered Translation vector t:\n%s", sst.str().c_str());


        //PUB ASTROBEE POSE
        geometry_msgs::PoseStamped PoseAstrobee;
        PoseAstrobee.header.stamp = timestamp;  
        PoseAstrobee.header.frame_id = "honey/body";


        //REMOVE STATIC CAST --> EIGEN ARE ALREADY DOUBLE
        PoseAstrobee.pose.position.x = bestPose.t(0);
        PoseAstrobee.pose.position.y = bestPose.t(1);
        PoseAstrobee.pose.position.z = bestPose.t(2);


        //TRANSPOSE TO GET THE CORRECT ROTATION MATRIX --> IF NOT TRANSPOSED, THE ROTATION MATRIX IS INVERTED
        //DOUBLE CHECK THIS ITS 5AM
        Eigen::Quaterniond q(bestPose.R.transpose()); 
        PoseAstrobee.pose.orientation.x = q.x();
        PoseAstrobee.pose.orientation.y = q.y();
        PoseAstrobee.pose.orientation.z = q.z();
        PoseAstrobee.pose.orientation.w = q.w();

        // Publish the pose
        pose_pub_.publish(PoseAstrobee);
    } else {
        ROS_ERROR("No valid pose found with the required constraints.");
    }
}


//NOT SUPPORTING YUV422 IMAGE IN THIS DEBUG VERSION, SINCE BENCHTEST BAGS ARE IN RGB8 OR BGR8
//SUPPORTING ASTROBEE BAYER IMAGES
    cv::Mat convertImageMessageToMat(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat mat;
        if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
            mat = cv::Mat(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
            mat = cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
        } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
            cv::Mat rgb(msg->height, msg->width, CV_8UC3, const_cast<uint8_t*>(&msg->data[0]), msg->step);
            cv::cvtColor(rgb, mat, cv::COLOR_RGB2BGR);
        } else if (msg->encoding == "bayer_grbg8"){ //HAVE TO USE STRING HERE...
            cv::Mat bayer(msg->height, msg->width, CV_8UC1, const_cast<uint8_t*>(&msg->data[0]), msg->step);
            cv::cvtColor(bayer, mat, cv::COLOR_BayerGR2BGR);
        } else {
            ROS_ERROR("Unsupported encoding type: %s", msg->encoding.c_str());
            return cv::Mat();
        }
        return mat.clone(); //FULL COPY CORRECTS ANY MEMORY ALIGNMENT ISSUES
    }

    sensor_msgs::ImagePtr convertMatToImageMessage(const cv::Mat& mat, const std_msgs::Header& header) {
        sensor_msgs::ImagePtr msg = boost::make_shared<sensor_msgs::Image>();
        msg->header = header;
        msg->height = mat.rows;
        msg->width = mat.cols;
        msg->encoding = sensor_msgs::image_encodings::BGR8;
        msg->is_bigendian = false;
        msg->step = mat.step;
        msg->data.assign(mat.datastart, mat.dataend);
        return msg;
    }

//=========================================================
//DO NOT MESS WITH THESE UNLESS YOU KNOW WHAT YOU ARE DOING
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher pose_pub_;
    std::queue<sensor_msgs::ImageConstPtr> producer_image_queue_;
    std::queue<sensor_msgs::ImageConstPtr> consumer_image_queue_;
    std::mutex queue_mutex_;
    std::thread process_thread_;
    bool keep_running_;
    std::condition_variable cv_;
    std::condition_variable space_available_cv_;  


//=========================================================
//ROS LAUNCH MODIFIABLE PARAMETERS 
    //EXECUTION PARAMETERS
    int num_threads_;
    //BLOB FILTERING PARAMETERS
    double min_circularity_;
    int saturation_threshold_;
    //CAMERA AND BEACON PARAMETERS
    std::vector<Eigen::Vector3d> knownPoints_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    //QUEUE STUFF
    size_t max_queue_size_;
    std::atomic<size_t> curr_queue_size_; 

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "carolus_rex_node");
    ros::NodeHandle nh;

    CarolusRexNode node(nh);

    ros::spin();

    return 0;
}
