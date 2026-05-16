#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#define FPS 60

#define MOUSE_SENSITIVITY 0.02f
#define FORWARD_SPEED 10.0f

void DrawVectorXYZ(Vector3 position) {
    const float length = 100.0f;
    DrawCylinderEx(position, (Vector3){length, 0, 0}, 0.05f, 0.05f, 8, RED);
    DrawCylinderEx(position, (Vector3){0, length, 0}, 0.05f, 0.05f, 8, GREEN);
    DrawCylinderEx(position, (Vector3){0, 0, length}, 0.05f, 0.05f, 8, BLUE);
}

typedef struct Airplane {
    Vector3 position;
    float forward_speed;

    float pitch_dot;
    float yaw_dot;
    float pitch;
    float yaw;

    Quaternion rotation;
} Airplane;

Airplane createAirplane() {
    Airplane plane = {0};
    plane.forward_speed = FORWARD_SPEED;
    plane.position = (Vector3){0, 50.0f, 10.0f};
    plane.rotation = QuaternionFromEuler(0, 0, 0);
    return plane;
}

void drawAirplane(Airplane plane) {
    const float cabinLength = 8.0f;
    const float cabinWidth = 4.0f;
    const float wingSpan = 16.0f;
    const float wingWidth = cabinLength / 3;
    const float wingHeight = 0.5f;

    rlPushMatrix();
    rlTranslatef(plane.position.x, plane.position.y, plane.position.z);

    Matrix rotation = QuaternionToMatrix(plane.rotation);
    rlMultMatrixf(MatrixToFloatV(rotation).v);

    Vector3 startPosition = (Vector3){0, 0, -cabinLength / 2};
    Vector3 endPosition = (Vector3){0, 0, cabinLength / 2};
    DrawCylinderEx(startPosition, endPosition, cabinWidth / 3, cabinWidth / 3,
                   24, GRAY);
    DrawCylinderWiresEx(startPosition, endPosition, cabinWidth / 3,
                        cabinWidth / 3, 24, DARKGRAY);

    DrawCube((Vector3){0, 0, -cabinLength / 5}, wingSpan, wingHeight, wingWidth,
             DARKGRAY);

    rlPopMatrix();
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
                       (-mouse_delta.y * MOUSE_SENSITIVITY);

    plane->yaw_dot = plane->yaw_dot * (1 - MOUSE_SENSITIVITY) +
                     (-mouse_delta.x * MOUSE_SENSITIVITY);

    plane->pitch += plane->pitch_dot * deltaTime;
    plane->yaw += plane->yaw_dot * deltaTime;

    plane->rotation = QuaternionFromEuler(plane->pitch, plane->yaw, 0);

    Vector3 forward = Vector3Transform((Vector3){0, 0, -1},
                                      QuaternionToMatrix(plane->rotation));
    plane->position = Vector3Add(plane->position, Vector3Scale(forward, plane->forward_speed * deltaTime));
}

void updateCamera(Camera3D *camera, Airplane plane) {
    const Vector3 offset = {0, 30.0f, 30.0f};

    Matrix rotation = QuaternionToMatrix(plane.rotation);

    Vector3 rotated_offset = Vector3Transform(offset, rotation);
    camera->position = Vector3Add(plane.position, rotated_offset);
    camera->target = plane.position;
    camera->up = Vector3Transform((Vector3){0, 1, 0}, rotation);
}

typedef struct Landmark {
    Vector3 position;
} Landmark;

int main(void) {
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight,
               "raylib [core] example - 3d camera mode");

    Camera3D camera = {0};
    camera.up = (Vector3){0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    DisableCursor();
    SetTargetFPS(FPS);
    SetExitKey(KEY_Q);

    Airplane plane = createAirplane();
    plane.position = (Vector3){0, 50.0, 10.0f};

    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        BeginDrawing();
        ClearBackground(RAYWHITE);
        BeginMode3D(camera);

        DrawGrid(128, 10.0f);

        DrawVectorXYZ((Vector3){0, 0, 0});

        float deltaTime = GetFrameTime();

        updateAirplane(&plane, deltaTime);
        drawAirplane(plane);

        updateCamera(&camera, plane);
        EndMode3D();
        DrawFPS(10, 10);
        Vector2 mouse_delta = GetMouseDelta();
        DrawText(TextFormat("Mouse Delta: (%.2f, %.2f)", mouse_delta.x,

                            mouse_delta.y),
                 10, 31, 20, BLACK);
        DrawText(TextFormat("Pitch: %.2f, Yaw: %.2f, Roll: %.2f",
                            getPitch(plane) * RAD2DEG, getYaw(plane) * RAD2DEG,
                            getRoll(plane) * RAD2DEG),
                 10, 51, 20, BLACK);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}
