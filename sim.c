#include <raylib.h>

void DrawVectorXYZ(Vector3 position) {
    const float lenght = 3.0f;
    // X-axis (Red) - Points right
    DrawLine3D(position, (Vector3){position.x + length, position.y, position.z},
               RED);

    // Y-axis (Green) - Points up
    DrawLine3D(position, (Vector3){position.x, position.y + length, position.z},
               GREEN);

    // Z-axis (Blue) - Points forward/backward depending on coordinate system
    DrawLine3D(position, (Vector3){position.x, position.y, position.z + length},
               BLUE);
}

typedef struct Plane {
    Vector3 position;
    Vector3 velocity;
    Matrix rotation;
} Plane;

void drawPlane(Plane plane) {
    const float length = 8.0f;
    const float width = 4.0f;

    // push matrix
    rlPushMatrix();
}

int main(void) {
    // Initialization
    //--------------------------------------------------------------------------------------
    const int screenWidth = 800;
    const int screenHeight = 450;

    InitWindow(screenWidth, screenHeight,
               "raylib [core] example - 3d camera mode");

    // Define the camera to look into our 3d world
    Camera3D camera = {0};
    camera.position = (Vector3){0.0f, 10.0f, 10.0f}; // Camera position
    camera.target = (Vector3){0.0f, 0.0f, 0.0f};     // Camera looking at point
    camera.up = (Vector3){0.0f, 1.0f,
                          0.0f}; // Camera up vector (rotation towards target)
    camera.fovy = 45.0f;         // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE; // Camera mode type

    Vector3 cubePosition = {0.0f, 0.0f, 0.0f};

    // exit on q
    SetTargetFPS(60); // Set our game to run at 60 frames-per-second
    //--------------------------------------------------------------------------------------

    // Main game loop
    SetExitKey(KEY_Q);
    while (!WindowShouldClose()) // Detect window close button or ESC key
    {
        BeginDrawing();

        ClearBackground(RAYWHITE);

        BeginMode3D(camera);

        DrawCube(cubePosition, 2.0f, 2.0f, 2.0f, RED);
        DrawCubeWires(cubePosition, 2.0f, 2.0f, 2.0f, MAROON);

        DrawGrid(10, 1.0f);

        EndMode3D();

        DrawText("Welcome to the third dimension!", 10, 40, 20, DARKGRAY);

        DrawFPS(10, 10);

        EndDrawing();
        //----------------------------------------------------------------------------------
    }

    // De-Initialization
    //--------------------------------------------------------------------------------------
    CloseWindow(); // Close window and OpenGL context
    //--------------------------------------------------------------------------------------

    return 0;
}
