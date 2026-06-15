#include "measurement_types.h"
#include <arpa/inet.h>
#include <assert.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <raylib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#define PORT 8080

#define UPDATE_RATE_HZ 30.0f

using namespace gtsam;

int main() {
    int sockfd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        std::cerr << "Socket creation failed" << std::endl;
        return -1;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&server_addr,
             sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return -1;
    }

    std::cout << "Listener running on UDP port " << PORT << "..." << std::endl;

    const std::chrono::milliseconds update_interval(
        static_cast<int>(1000.0f / UPDATE_RATE_HZ));

    auto landmark_prior_pose_noise =
        noiseModel::Diagonal::Sigmas((Vector(3) << 0.5, 0.5, 0.5).finished());

    // Add priors for landmarks

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    NonlinearFactorGraph graph;
    Values new_initial_estimates;

    auto odom_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << ODOMETRY_ANGULAR_NOISE_STD_DEV,
         ODOMETRY_ANGULAR_NOISE_STD_DEV, ODOMETRY_ANGULAR_NOISE_STD_DEV,
         ODOMETRY_X_NOISE_STD_DEV, ODOMETRY_Y_NOISE_STD_DEV,
         ODOMETRY_Z_NOISE_STD_DEV)
            .finished());

    auto gnss_noise = noiseModel::Diagonal::Sigmas(
        (Vector(3) << GNSS_X_NOISE_STD_DEV, GNSS_Y_NOISE_STD_DEV,
         GNSS_Z_NOISE_STD_DEV)
            .finished());

    auto radar_noise = noiseModel::Diagonal::Sigmas(
        (Vector(3) << RADAR_AZIMUTH_NOISE_STD_DEV,
         RADAR_ELEVATION_NOISE_STD_DEV, RADAR_RANGE_NOISE_STD_DEV)
            .finished());

    auto prior_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4).finished());

    Point3 landmarks[4];

    for (int i = 1; i <= 3; ++i) {
        switch (i) {
        case 1:
            landmarks[i] = Point3(LANDMARK_1_POSITION);
            break;
        case 2:
            landmarks[i] = Point3(LANDMARK_2_POSITION);
            break;
        case 3:
            landmarks[i] = Point3(LANDMARK_3_POSITION);
            break;
        }

        gtsam::Key landmark_key = Symbol('L', i);
        graph.add(PriorFactor<Point3>(landmark_key, landmarks[i],
                                      landmark_prior_pose_noise));
        new_initial_estimates.insert(landmark_key, landmarks[i]);
    }

    uint64_t current_pose_key = 0;
    Pose3 current_dead_reckoning_pose =
        Pose3(Rot3::Identity(),
              Point3(PLANE_INITIAL_X, PLANE_INITIAL_Y, PLANE_INITIAL_Z));

    graph.add(PriorFactor<Pose3>(current_pose_key, current_dead_reckoning_pose,
                                 prior_noise));

    new_initial_estimates.insert(current_pose_key, current_dead_reckoning_pose);

    SensorMessage msg;

    std::chrono::time_point last_update_time =
        std::chrono::high_resolution_clock::now();

    bool update_isam = false;
    while (true) {
        ssize_t bytes_read =
            recvfrom(sockfd, &msg, sizeof(msg), 0,
                     (struct sockaddr *)&client_addr, &client_len);

        if (bytes_read > 0) {
            switch (msg.type) {
            case ODOMETRY_MEASUREMENT: {
                double dx = msg.data.odom.delta_position.x;
                double dy = msg.data.odom.delta_position.y;
                double dz = msg.data.odom.delta_position.z;

                Eigen::Quaterniond delta_q(msg.data.odom.delta_orientation.w,
                                           msg.data.odom.delta_orientation.x,
                                           msg.data.odom.delta_orientation.y,
                                           msg.data.odom.delta_orientation.z);

                Pose3 odom_measurement(Rot3(delta_q), Point3(dx, dy, dz));

                graph.add(BetweenFactor<Pose3>(current_pose_key,
                                               current_pose_key + 1,
                                               odom_measurement, odom_noise));
                current_dead_reckoning_pose =
                    current_dead_reckoning_pose.compose(odom_measurement);

                new_initial_estimates.insert(current_pose_key + 1,
                                             current_dead_reckoning_pose);
                current_pose_key++;
                update_isam = true;
                break;
            }
            case RADAR_MEASUREMENT: {
                int n_landmarks = msg.data.radar.num_returns;
                assert(n_landmarks <= 3);
                for (int i = 0; i < n_landmarks; i++) {
                    RadarReturnData return_data = msg.data.radar.returns[i];
                    uint8_t landmark_id = return_data.landmark_id;
                    std::cout << "Received radar return for landmark ID: "
                              << static_cast<int>(landmark_id) << std::endl;

                    ASSERT_EX(landmark_id >= 1 && landmark_id <= 3,
                              std::cerr << "Invalid landmark ID: "
                                        << static_cast<int>(landmark_id)
                                        << std::endl;);
                    double range = return_data.range;
                    double azimuth = return_data.azimuth;
                    double elevation = return_data.elevation;

                    gtsam::Key landmark_key = Symbol('L', landmark_id);

                    Point3 ray(cos(elevation) * cos(azimuth),
                               cos(elevation) * sin(azimuth), sin(elevation));
                    Unit3 measured_bearing = Unit3(ray);

                    graph.add(BearingRangeFactor<Pose3, Point3, Unit3, double>(
                        current_pose_key, landmark_key, measured_bearing, range,
                        radar_noise));

                    update_isam = true;
                }
                break;
            }
            case GNSS_MEASUREMENT: {
                Point3 gps_measurement(msg.data.gnss.position.x,
                                       msg.data.gnss.position.y,
                                       msg.data.gnss.position.z);
                graph.add(
                    GPSFactor(current_pose_key, gps_measurement, gnss_noise));
                update_isam = true;
                break;
            }
            default:
                std::cout << "Unknown message received.\n";
            }

            std::chrono::time_point current_time =
                std::chrono::high_resolution_clock::now();
            auto elapsed_time =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - last_update_time);

            if (update_isam && elapsed_time >= update_interval) {
                update_isam = false;
                isam.update(graph, new_initial_estimates);

                graph.resize(0);
                new_initial_estimates.clear();
                isam.update();

                Values current_estimate = isam.calculateEstimate();
                Pose3 optimized_pose =
                    current_estimate.at<Pose3>(current_pose_key);

                current_dead_reckoning_pose = optimized_pose;

                Eigen::MatrixXd cov = isam.marginalCovariance(current_pose_key);

                PoseWithCovariance predicted_pose;

                predicted_pose.position.x = optimized_pose.x();
                predicted_pose.position.y = optimized_pose.y();
                predicted_pose.position.z = optimized_pose.z();
                predicted_pose.orientation.x =
                    optimized_pose.rotation().toQuaternion().x();
                predicted_pose.orientation.y =
                    optimized_pose.rotation().toQuaternion().y();
                predicted_pose.orientation.z =
                    optimized_pose.rotation().toQuaternion().z();
                predicted_pose.orientation.w =
                    optimized_pose.rotation().toQuaternion().w();

                for (int i = 0; i < 6; i++) {
                    for (int j = 0; j < 6; j++) {
                        predicted_pose.covariance[i][j] = cov(i, j);
                    }
                }

                Map map;

                for (int i = 1; i <= 3; ++i) {
                    gtsam::Key landmark_key = Symbol('L', i);
                    gtsam::Marginals marginals(graph, current_estimate);

                    Point3 landmark_position =
                        current_estimate.at<Point3>(landmark_key);
                    Eigen::Matrix3d landmark_cov =
                        isam.marginalCovariance(landmark_key);

                    map.landmarks[i - 1].id = i;
                    map.landmarks[i - 1].position.x = landmark_position.x();
                    map.landmarks[i - 1].position.y = landmark_position.y();
                    map.landmarks[i - 1].position.z = landmark_position.z();

                    for (int j = 0; j < 3; j++) {
                        for (int k = 0; k < 3; k++) {
                            map.covariance[i - 1][j][k] = landmark_cov(j, k);
                        }
                    }
                }

                SlamMessage slam_msg;
                slam_msg.pose_with_covariance = predicted_pose;
                slam_msg.map = map;

                int path_idx = 0;
                int start_key =
                    std::max(0, static_cast<int>(current_pose_key -
                                                 MAX_PATH_LENGTH + 1));

                for (uint64_t i = start_key; i <= current_pose_key; ++i) {
                    if (current_estimate.exists(i)) {
                        Pose3 p = current_estimate.at<Pose3>(i);
                        slam_msg.predicted_path[path_idx].x = p.x();
                        slam_msg.predicted_path[path_idx].y = p.y();
                        slam_msg.predicted_path[path_idx].z = p.z();
                        path_idx++;
                    }
                }

                slam_msg.path_length = path_idx;

                ASSERT_EX(slam_msg.map.landmarks[0].id == 1 &&
                              slam_msg.map.landmarks[1].id == 2 &&
                              slam_msg.map.landmarks[2].id == 3,
                          std::cerr << "Landmark IDs in map do not match "
                                       "expected values.\n";);

                sendto(sockfd, &slam_msg, sizeof(slam_msg), 0,
                       (struct sockaddr *)&client_addr, sizeof(client_addr));

                last_update_time = current_time;

                std::cout << "\033[2J\033[1;1H"; // Clear console and move
                                                 // cursor to top-left
                std::cout << "==============" << std::endl;
                std::cout << "Optimized Pos: (" << optimized_pose.x() << " (+ -"
                          << sqrt(cov(3, 3)) << "m), " << optimized_pose.y()
                          << " ( +-" << sqrt(cov(4, 4)) << "m), "
                          << optimized_pose.z() << " ( +-" << sqrt(cov(5, 5))
                          << "m)\n";
                std::cout << "Optimized Orient: ("
                          << optimized_pose.rotation().yaw() * 180.0 / M_PI
                          << " (+-" << sqrt(cov(0, 0)) * 180.0 / M_PI
                          << " deg), "
                          << optimized_pose.rotation().pitch() * 180.0 / M_PI
                          << " (+-" << sqrt(cov(1, 1)) * 180.0 / M_PI
                          << " deg), "
                          << optimized_pose.rotation().roll() * 180.0 / M_PI
                          << " (+-" << sqrt(cov(2, 2)) * 180.0 / M_PI
                          << " deg)\n";
            }
        }
    }

    close(sockfd);
    return 0;
}
