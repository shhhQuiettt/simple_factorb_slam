#include "airplane.h"
#include "make_measurements.h"
#include "measurement_types.h"
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <fcntl.h>

#define FPS 60

#define MOUSE_SENSITIVITY 0.0f
// #define MOUSE_SENSITIVITY 0.1f
#define FORWARD_SPEED 10.0f
#define N_LANDMARKS 3

float global_covariance[3];

void DrawVectorXYZ(Vector3 position, float length, float thickness) {
    Vector3 endX = Vector3Add(position, (Vector3){length, 0, 0});
    Vector3 endY = Vector3Add(position, (Vector3){0, length, 0});
    Vector3 endZ = Vector3Add(position, (Vector3){0, 0, length});

    DrawCylinderEx(position, endX, thickness / 2, thickness / 2, 8, RED);
    DrawCylinderEx(position, endY, thickness / 2, thickness / 2, 8, GREEN);
    DrawCylinderEx(position, endZ, thickness / 2, thickness / 2, 8, BLUE);
}

void DrawTF(Vector3 position, Quaternion rotation, float length,
            float thickness) {
    Vector3 localX = Vector3RotateByQuaternion((Vector3){1, 0, 0}, rotation);
    Vector3 localY = Vector3RotateByQuaternion((Vector3){0, 1, 0}, rotation);
    Vector3 localZ = Vector3RotateByQuaternion((Vector3){0, 0, 1}, rotation);

    Vector3 endX = Vector3Add(position, Vector3Scale(localX, length));
    Vector3 endY = Vector3Add(position, Vector3Scale(localY, length));
    Vector3 endZ = Vector3Add(position, Vector3Scale(localZ, length));

    DrawCylinderEx(position, endX, thickness / 2, thickness / 2, 8, RED);
    DrawCylinderEx(position, endY, thickness / 2, thickness / 2, 8, GREEN);
    DrawCylinderEx(position, endZ, thickness / 2, thickness / 2, 8, BLUE);
}

Airplane createAirplane(void) {
    Airplane plane = {0};
    plane.forward_speed = FORWARD_SPEED;
    plane.position = (Vector3){0, 50.0f, 0.0f};
    plane.rotation = QuaternionFromAxisAngle((Vector3){0, 1, 0}, PI);
    // plane.rotation = QuaternionIdentity();
    plane.pitch = QuaternionToEuler(plane.rotation).y;
    plane.yaw = QuaternionToEuler(plane.rotation).z;

    plane.pitch_dot = 0.0f;
    plane.yaw_dot = PI / 8;

    return plane;
}

void drawLandmarks(Landmark *landmarks, int n_landmarks) {
    for (int i = 0; i < n_landmarks; i++) {
        DrawSphere(landmarks[i].position, 3.0f, DARKBLUE);
    }
}

void drawAirplane(Airplane plane) {
    const float cabinLength = 8.0f;
    const float cabinWidth = 4.0f;
    const float wingSpan = 16.0f;
    const float wingWidth = cabinLength / 3;
    const float wingHeight = 0.5f;

    rlPushMatrix();
    rlTranslatef(plane.position.x, plane.position.y, plane.position.z);

    rlMultMatrixf(MatrixToFloatV(QuaternionToMatrix(plane.rotation)).v);

    Vector3 startPosition = (Vector3){0, 0, -cabinLength / 2};
    Vector3 endPosition = (Vector3){0, 0, cabinLength / 2};
    DrawCylinderEx(startPosition, endPosition, cabinWidth / 3, cabinWidth / 3,
                   24, GRAY);
    DrawCylinderWiresEx(startPosition, endPosition, cabinWidth / 3,
                        cabinWidth / 3, 24, DARKGRAY);

    DrawCube((Vector3){0, 0, cabinLength / 5}, wingSpan, wingHeight, wingWidth,
             DARKGRAY);

    rlPopMatrix();

    DrawTF(plane.position, plane.rotation, 8, 0.5f);
}

void drawPrediction(PoseWithCovariance prediction) {
    DrawTF(prediction.position, prediction.orientation, 8, 0.5f);
    float covariance_x = prediction.covariance[0][0];
    float covariance_y = prediction.covariance[1][1];
    float covariance_z = prediction.covariance[2][2];

    global_covariance[0] = covariance_x;
    global_covariance[1] = covariance_y;
    global_covariance[2] = covariance_z;

    Mesh sphereMesh = GenMeshSphere(1.0f, 16, 16);
    Model ellipsoid = LoadModelFromMesh(sphereMesh);


    ellipsoid.transform = MatrixScale(covariance_x, covariance_y, covariance_z);
    // DrawModel(ellipsoid, prediction.position, 1.0f, Fade(RED, 0.5f));


}

float getPitch(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.y;
}

float getYaw(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.z;
}

float getRoll(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.x;
}

void updateAirplane(Airplane *plane, float deltaTime) {
    Vector2 mouse_delta = GetMouseDelta();

    plane->pitch_dot = plane->pitch_dot * (1 - MOUSE_SENSITIVITY) +
                       (mouse_delta.y * MOUSE_SENSITIVITY);

    plane->yaw_dot = plane->yaw_dot * (1 - MOUSE_SENSITIVITY) +
                     (-mouse_delta.x * MOUSE_SENSITIVITY);

    plane->pitch += plane->pitch_dot * deltaTime;
    plane->yaw += plane->yaw_dot * deltaTime;

    plane->rotation = QuaternionFromEuler(plane->pitch, plane->yaw, 0);

    Vector3 forward = Vector3Transform((Vector3){0, 0, 1},
                                       QuaternionToMatrix(plane->rotation));
    plane->position =
        Vector3Add(plane->position,
                   Vector3Scale(forward, plane->forward_speed * deltaTime));
}

void updateCamera(Camera3D *camera, Airplane plane) {
    const Vector3 offset = {0, 15.0f, -30.0f};

    Matrix rotation = QuaternionToMatrix(plane.rotation);

    Vector3 rotated_offset = Vector3Transform(offset, rotation);
    camera->position = Vector3Add(plane.position, rotated_offset);
    camera->target = plane.position;
    camera->up = Vector3Transform((Vector3){0, 1, 0}, rotation);
}

int main(void) {
    srand(time(NULL));

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        return -1;
    }
    fcntl(sockfd, F_SETFL, O_NONBLOCK);
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(8080);
    dest_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    float odom_timer = 0.0f;
    float radar_timer = 0.0f;
    float gnss_timer = 0.0f;

    const int screenWidth = 800;
    const int screenHeight = 450;

    Landmark landmarks[3] = {
        (Landmark){.id = 1, .position = (Vector3){-80, 40, -80}},
        (Landmark){.id = 1, .position = (Vector3){-120, 40, -120}},
        (Landmark){.id = 1, .position = (Vector3){-130, 45, -50}},
    };

    InitWindow(screenWidth, screenHeight,
               "raylib [core] example - 3d camera mode");

    Camera3D camera = {0};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(FPS);
    SetExitKey(KEY_Q);

    Airplane plane = createAirplane();

    bool slam_active = false;
    PoseWithCovariance slam_pose = {0};

    DisableCursor();
    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        DrawGrid(128, 10.0f);

        float deltaTime = GetFrameTime();
        updateAirplane(&plane, deltaTime);

        odom_timer += deltaTime;
        radar_timer += deltaTime;
        gnss_timer += deltaTime;

        SensorMessage msg;

        if (odom_timer >= 1.0f / ODOM_HZ) {
            msg = measure_odometry(&plane);
            sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr));
            odom_timer = 0.0f;
        }

        if (radar_timer >= 1.0f / RADAR_HZ) {
            // msg = measure_radar(&plane, landmarks, N_LANDMARKS);
            // sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr
            // *)&dest_addr,
            //        sizeof(dest_addr));

            radar_timer = 0.0f;
        }

        if (gnss_timer >= 1.0f / GNSS_HZ) {
            msg = measure_gnss(&plane);
            sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr));
            gnss_timer = 0.0f;
        }

        DrawVectorXYZ((Vector3){0, 0, 0}, 100.0f, 5.0f);

        drawAirplane(plane);
        drawLandmarks(landmarks, N_LANDMARKS);

        updateCamera(&camera, plane);

        while (recv(sockfd, &slam_pose, sizeof(slam_pose), 0) > 0) {
            slam_active = true;
        }

        if (slam_active) {
            drawPrediction(slam_pose);
        }

        EndMode3D();
        DrawFPS(10, 10);
        // Vector2 mouse_delta = GetMouseDelta();
        // DrawText(TextFormat("Mouse Delta: (%.2f, %.2f)", mouse_delta.x,

        //                     mouse_delta.y),
        //          10, 31, 20, BLACK);
        DrawText(TextFormat("Position: (%.2f, %.2f, %.2f)", plane.position.x,
                            plane.position.y, plane.position.z),
                 10, 31, 20, BLACK);
        DrawText(TextFormat("Pitch: %.2f, Yaw: %.2f, Roll: %.2f",
                            getPitch(plane) * RAD2DEG, getYaw(plane) * RAD2DEG,
                            getRoll(plane) * RAD2DEG),
                 10, 51, 20, BLACK);

        if (slam_active) {
            DrawText("SLAM Active", 10, 71, 20, RED);
        } else {
            DrawText("SLAM Inactive", 10, 71, 20, DARKGRAY);
        }
        DrawText(TextFormat("Covariance: (%.2f, %.2f, %.2f)", global_covariance[0], global_covariance[1], global_covariance[2]),
             10, 91, 20, BLACK);
        EndDrawing();
    }

    close(sockfd);
    CloseWindow();
    return 0;
}
