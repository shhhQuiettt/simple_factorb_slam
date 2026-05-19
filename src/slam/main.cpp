#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "measurement_types.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
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

    if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        return -1;
    }

    std::cout << "Listener running on UDP port " << PORT << "..." << std::endl;


    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    ISAM2 isam(parameters);

    SensorMessage msg;

    while (true) {
        ssize_t bytes_read = recvfrom(sockfd, &msg, sizeof(msg), 0, 
                                     (struct sockaddr *)&client_addr, &client_len);
        
        if (bytes_read > 0) {
            switch (msg.type) {
                case ODOMETRY:
                    std::cout << "[24Hz ODOM] Pos: (" << msg.data.odom.velocity.x << ", " 
                              << msg.data.odom.velocity.y << ", " << msg.data.odom.velocity.z << ")\n";
                    break;
                case RADAR:
                    std::cout << "[10Hz RADAR] Landmark: " << msg.data.radar.landmark_id 
                              << " Range: " << msg.data.radar.range << "\n";
                    break;
                case GNSS:
                    std::cout << "[1Hz GNSS] Global Pos: (" << msg.data.gnss.position.x << ", " 
                              << msg.data.gnss.position.y << ", " << msg.data.gnss.position.z << ")\n";
                    break;
                default:
                    std::cout << "Unknown message received.\n";
            }
        }
    }

    close(sockfd);
    return 0;
}
