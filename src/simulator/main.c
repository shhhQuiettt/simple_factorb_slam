#include "airplane.h"

#include "make_measurements.h"
#include "measurement_types.h"
#include <assert.h>
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

#define X_AXIS (Vector3){1, 0, 0}
#define Y_AXIS (Vector3){0, 1, 0}
#define Z_AXIS (Vector3){0, 0, 1}

float global_covariance[3];

bool almost_equal(float a, float b) {
    const float epsilon = 1e-6f;
    return fabs(a - b) < epsilon;
}

Vector3 ToRaylibPosition(Vector3 world_position) {
    Vector3 raylib_position;
    raylib_position.x = -world_position.y;
    raylib_position.y = world_position.z;
    raylib_position.z = -world_position.x;
    return raylib_position;
}

Quaternion ToRaylibRotation(Quaternion world_rotation) {
    Quaternion raylib_rotation;
    raylib_rotation.x = -world_rotation.y;
    raylib_rotation.y = world_rotation.z;
    raylib_rotation.z = -world_rotation.x;
    raylib_rotation.w = world_rotation.w;
    return raylib_rotation;
}

void DrawVectorXYZ(Vector3 position, float length, float thickness) {
    Vector3 endX = Vector3Add(position, (Vector3){length, 0, 0});
    Vector3 endY = Vector3Add(position, (Vector3){0, length, 0});
    Vector3 endZ = Vector3Add(position, (Vector3){0, 0, length});

    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endX),
                   thickness / 2, thickness / 2, 8, RED);
    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endY),
                   thickness / 2, thickness / 2, 8, GREEN);
    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endZ),
                   thickness / 2, thickness / 2, 8, BLUE);
}

void DrawTF(Vector3 position, Quaternion rotation, float length,
            float thickness) {
    Vector3 localX = Vector3RotateByQuaternion(X_AXIS, rotation);

    Vector3 localY = Vector3RotateByQuaternion(Y_AXIS, rotation);
    Vector3 localZ = Vector3RotateByQuaternion(Z_AXIS, rotation);

    Vector3 endX = Vector3Add(position, Vector3Scale(localX, length));
    Vector3 endY = Vector3Add(position, Vector3Scale(localY, length));
    Vector3 endZ = Vector3Add(position, Vector3Scale(localZ, length));

    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endX),
                   thickness / 2, thickness / 2, 8, RED);
    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endY),
                   thickness / 2, thickness / 2, 8, GREEN);
    DrawCylinderEx(ToRaylibPosition(position), ToRaylibPosition(endZ),
                   thickness / 2, thickness / 2, 8, BLUE);
}

Airplane createAirplane(void) {
    Airplane plane = {0};
    plane.velocity = (Vector3){FORWARD_SPEED, 0.0f, 0.0f};
    plane.position =
        (Vector3){PLANE_INITIAL_X, PLANE_INITIAL_Y, PLANE_INITIAL_Z};

    plane.rotation = QuaternionIdentity();
    plane.twist = (Vector3){0.0f, 0.0f, PI / 8};
    // plane.twist = (Vector3){0.0f, 0.0f, 0.0f};

    return plane;
}

void drawLandmarks(Landmark *landmarks, int n_landmarks) {
    const Color colors[3] = {RED, GREEN, BLUE};
    for (int i = 0; i < n_landmarks; i++) {
        DrawSphere(ToRaylibPosition(landmarks[i].position), 3.0f,
                   colors[i % 3]);
    }
}

void drawAirplane(Airplane plane) {
    const float cabinLength = 8.0f;
    const float cabinWidth = 4.0f;
    const float wingSpan = 16.0f;
    const float wingWidth = cabinLength / 3;
    const float wingHeight = 0.5f;

    rlPushMatrix();

    Vector3 raylib_position = ToRaylibPosition(plane.position);
    rlTranslatef(raylib_position.x, raylib_position.y, raylib_position.z);

    Quaternion raylib_rotation = ToRaylibRotation(plane.rotation);
    Matrix rotation_matrix = QuaternionToMatrix(raylib_rotation);
    rlMultMatrixf(MatrixToFloat(rotation_matrix));

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

void drawPrediction(PoseWithCovariance prediction, Model prediction_ellipsoid) {
    DrawTF(prediction.position, prediction.orientation, 5.0f, 0.5f);
    float covariance_x = prediction.covariance[3][3];
    float covariance_y = prediction.covariance[4][4];
    float covariance_z = prediction.covariance[5][5];

    global_covariance[0] = covariance_x;
    global_covariance[1] = covariance_y;
    global_covariance[2] = covariance_z;

    prediction_ellipsoid.transform =
        MatrixScale(3 * sqrtf(covariance_y), 3 * sqrtf(covariance_z),
                    3 * sqrtf(covariance_x));

    prediction_ellipsoid.materials[0].maps[MATERIAL_MAP_DIFFUSE].color =
        Fade(RED, 0.2f);

    // rlEnableColorBlend();
    // rlDisableDepthMask();
    // rlDisableBackfaceCulling();

    DrawModel(prediction_ellipsoid, ToRaylibPosition(prediction.position), 1.0f,
              WHITE);

    // rlDrawRenderBatchActive();
    // rlEnableBackfaceCulling();
    // rlEnableDepthMask();
}

float getRoll(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.x;
}

float getPitch(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.y;
}

float getYaw(Airplane plane) {
    Vector3 forward = QuaternionToEuler(plane.rotation);
    return forward.z;
}

void updateAirplane(Airplane *plane, float deltaTime) {
    float roll_delta = plane->twist.x * deltaTime;
    float pitch_delta = plane->twist.y * deltaTime;
    float yaw_delta = plane->twist.z * deltaTime;

    Quaternion delta_pitch = QuaternionFromAxisAngle(Y_AXIS, pitch_delta);
    Quaternion delta_yaw = QuaternionFromAxisAngle(Z_AXIS, yaw_delta);
    Quaternion delta_roll = QuaternionFromAxisAngle(X_AXIS, roll_delta);

    plane->rotation = QuaternionMultiply(plane->rotation, delta_yaw);
    plane->rotation = QuaternionMultiply(plane->rotation, delta_pitch);
    plane->rotation = QuaternionMultiply(plane->rotation, delta_roll);

    plane->rotation = QuaternionNormalize(plane->rotation);

    Vector3 velocity_global =
        Vector3RotateByQuaternion(plane->velocity, plane->rotation);
    plane->position =
        Vector3Add(plane->position, Vector3Scale(velocity_global, deltaTime));
}

void updateCamera(Camera3D *camera, Airplane plane) {
    const Vector3 offset = {-30.0f, 0.0f, 15.0f};

    Matrix rotation = QuaternionToMatrix(plane.rotation);

    Vector3 rotated_offset = Vector3Transform(offset, rotation);
    camera->position =
        ToRaylibPosition(Vector3Add(plane.position, rotated_offset));
    camera->target = ToRaylibPosition(plane.position);
    camera->up = ToRaylibPosition(Vector3Transform(Z_AXIS, rotation));
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

    InitWindow(screenWidth, screenHeight,
               "raylib [core] example - 3d camera mode");

    Mesh ellipsoid_mesh = GenMeshSphere(1.0f, 16, 16);
    Model prediction_ellipsoid = LoadModelFromMesh(ellipsoid_mesh);

    Camera3D camera = {0};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    SetTargetFPS(FPS);
    SetExitKey(KEY_Q);

    Airplane plane = createAirplane();

    Landmark landmarks[3] = {
        {LANDMARK_1_ID, (Vector3)LANDMARK_1_POSITION},
        {LANDMARK_2_ID, (Vector3)LANDMARK_2_POSITION},
        {LANDMARK_3_ID, (Vector3)LANDMARK_3_POSITION},
    };

    bool slam_active = false;
    SlamMessage slam_msg = {0};
    PoseWithCovariance *slam_pose =
        (PoseWithCovariance *)&slam_msg.pose_with_covariance;
    Map *slam_map = (Map *)&slam_msg.map;

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

        SensorMessage msg = {0};

        if (odom_timer >= 1.0f / ODOM_HZ) {
            msg = measure_odometry(&plane);
            sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr));
            odom_timer -= 1.0f / ODOM_HZ;
        }

        if (radar_timer >= 1.0f / RADAR_HZ) {
            msg = measure_radar(&plane, landmarks, N_LANDMARKS);
            sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr));

            radar_timer -= 1.0f / RADAR_HZ;
        }

        if (gnss_timer >= 1.0f / GNSS_HZ) {
            msg = measure_gnss(&plane);
            sendto(sockfd, &msg, sizeof(msg), 0, (struct sockaddr *)&dest_addr,
                   sizeof(dest_addr));
            gnss_timer -= 1.0f / GNSS_HZ;
        }

        DrawVectorXYZ((Vector3){0, 0, 0}, 100.0f, 5.0f);

        drawLandmarks(landmarks, N_LANDMARKS);

        updateCamera(&camera, plane);

        while (recv(sockfd, &slam_msg, sizeof(slam_msg), 0) > 0) {
            slam_active = true;
        }

        if (slam_active) {
            drawPrediction(*slam_pose, prediction_ellipsoid);
        }

        drawAirplane(plane);

        EndMode3D();
        DrawFPS(10, 10);
        // Vector2 mouse_delta = GetMouseDelta();
        // DrawText(TextFormat("Mouse Delta: (%.2f, %.2f)", mouse_delta.x,

        //                     mouse_delta.y),
        //          10, 31, 20, BLACK);
        DrawText(TextFormat("Position: (%.2f, %.2f, %.2f)", plane.position.x,
                            plane.position.y, plane.position.z),
                 10, 31, 20, BLACK);

        DrawText(TextFormat("Predicted Position: (%.2f, %.2f, %.2f)",
                            slam_pose->position.x, slam_pose->position.y,
                            slam_pose->position.z),
                 10, 51, 20, BLACK);
        DrawText(TextFormat("Covariance: (%.2f, %.2f, %.2f)",
                            slam_pose->covariance[3][3],
                            slam_pose->covariance[4][4],
                            slam_pose->covariance[5][5]),

                 10, 71, 20, BLACK);
        DrawText(TextFormat("Pitch: %.2f, Yaw: %.2f, Roll: %.2f",
                            getPitch(plane) * RAD2DEG, getYaw(plane) * RAD2DEG,
                            getRoll(plane) * RAD2DEG),
                 10, 91, 20, BLACK);

        if (slam_active) {
            DrawText("SLAM Active", 10, 111, 20, RED);
        } else {
            DrawText("SLAM Inactive", 10, 101, 20, DARKGRAY);
        }
        EndDrawing();
    }

    UnloadModel(prediction_ellipsoid);
    close(sockfd);
    CloseWindow();
    return 0;
}
