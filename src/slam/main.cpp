#include "measurement_types.h"
#include <arpa/inet.h>
#include <cstring>
#include <gtsam/geometry/BearingRange.h>
#include <iostream>
#include <netinet/in.h>
#include <raylib.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#define PORT 8080

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

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    NonlinearFactorGraph graph;
    Values new_initial_estimates;

    const double odom_dt = 1.0 / ODOM_HZ;
    auto odom_noise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << ODOMETRY_ROLL_NOISE_STD_DEV * odom_dt,
         ODOMETRY_PITCH_NOISE_STD_DEV * odom_dt,
         ODOMETRY_YAW_NOISE_STD_DEV * odom_dt,
         ODOMETRY_X_NOISE_STD_DEV * odom_dt, ODOMETRY_Y_NOISE_STD_DEV * odom_dt,
         ODOMETRY_Z_NOISE_STD_DEV * odom_dt)
            .finished());

    auto gnss_noise = noiseModel::Diagonal::Sigmas(
        (Vector(3) << GNSS_X_NOISE_STD_DEV, GNSS_Y_NOISE_STD_DEV,
         GNSS_Z_NOISE_STD_DEV)
            .finished());

    uint64_t current_pose_key = 0;
    Pose3 current_dead_reckoning_pose =
        Pose3(Rot3::RzRyRx(0, PI, 0), Point3(0, 50.0f, 0));

    graph.add(PriorFactor<Pose3>(current_pose_key, current_dead_reckoning_pose,
                                 odom_noise));
    new_initial_estimates.insert(current_pose_key, current_dead_reckoning_pose);

    SensorMessage msg;

    while (true) {
        ssize_t bytes_read =
            recvfrom(sockfd, &msg, sizeof(msg), 0,
                     (struct sockaddr *)&client_addr, &client_len);

        if (bytes_read > 0) {
            bool update_isam = false;

            switch (msg.type) {
            case ODOMETRY_MEASUREMENT: {
                double dx = msg.data.odom.velocity.x * odom_dt;
                double dy = msg.data.odom.velocity.y * odom_dt;
                double dz = msg.data.odom.velocity.z * odom_dt;

                double d_x_rot = msg.data.odom.twist.x * odom_dt;
                double d_y_rot = msg.data.odom.twist.y * odom_dt;
                double d_z_rot = msg.data.odom.twist.z * odom_dt;

                Pose3 odom_measurement =
                    Pose3(Rot3::Rodrigues(d_x_rot, d_y_rot, d_z_rot),
                          Point3(dx, dy,
                                 dz)); // because of the simulation coord system

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
            case RADAR_MEASUREMENT:
                // int n_landmarks = msg.data.radar.num_returns;
                // for (int i = 0; i < n_landmarks; i++) {
                //     RadarReturnData return_data = msg.data.radar.returns[i];
                //     double range = return_data.range;
                //     double azimuth = return_data.rot_x;
                //     double elevation = return_data.rot_z;

                //     Unit3 measured_bearing = 
                //         <ScrollWheelUp>

                // }

                break;
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

            if (update_isam) {
                isam.update(graph, new_initial_estimates);

                graph.resize(0);
                new_initial_estimates.clear();

                Values current_estimate = isam.calculateEstimate();
                Pose3 optimized_pose =
                    current_estimate.at<Pose3>(current_pose_key);

                current_dead_reckoning_pose = optimized_pose;

                Eigen::MatrixXd cov = isam.marginalCovariance(current_pose_key);

                PoseWithCovariance output_msg;

                output_msg.position.x = optimized_pose.x();
                output_msg.position.y = optimized_pose.y();
                output_msg.position.z = optimized_pose.z();
                output_msg.orientation.x =
                    optimized_pose.rotation().toQuaternion().x();
                output_msg.orientation.y =
                    optimized_pose.rotation().toQuaternion().y();
                output_msg.orientation.z =
                    optimized_pose.rotation().toQuaternion().z();
                output_msg.orientation.w =
                    optimized_pose.rotation().toQuaternion().w();

                sendto(sockfd, &output_msg, sizeof(output_msg), 0,
                       (struct sockaddr *)&client_addr, sizeof(client_addr));

                std::cout << "\033[2J\033[1;1H"; // Clear console and move
                                                 // cursor to top-left
                std::cout << "==============" << std::endl;
                std::cout << "Optimized Pos: (" << optimized_pose.x() << " ( +-"
                          << sqrt(cov(3, 3)) << "m), " << optimized_pose.y()
                          << " ( +-" << sqrt(cov(4, 4)) << "m), "
                          << optimized_pose.z() << " ( +-" << sqrt(cov(5, 5))
                          << "m)\n";
            }
        }
    }

    close(sockfd);
    return 0;
}
