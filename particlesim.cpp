// can remove define if not linking static version
#define RAYLIB_STATIC
#include "raylib.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstring>

//---------------------------------------------------------------------------
// Simulation and Window Parameters
//---------------------------------------------------------------------------

const int screenWidth = 1920;
const int screenHeight = 1080;
const int simulationAreaWidth = 1000;
const int simulationAreaHeight = 1000;

// Particle rendering/collision radius.
const float particleRadius = 3.0f;

// Initial particle count.
int particleCount = 10000;

//---------------------------------------------------------------------------
// Interaction and Spatial Partitioning Parameters
//---------------------------------------------------------------------------

// Global neighbor interaction distance.
// (Also used as the grid cell size; we update it dynamically.)
float neighborRadius = 25.0f;
int gridCellSize = (int)neighborRadius;

//---------------------------------------------------------------------------
// Multiple Particle Types Setup
//---------------------------------------------------------------------------

const int numTypes = 7;

// Colors for each particle type.
Color particleColors[numTypes] = { RED, GREEN, BLUE, GRAY, VIOLET, BEIGE, YELLOW };

// Attraction force matrix (per pair of particle types).
float attractionMatrix[numTypes][numTypes] = {
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f}
};

// Repulsion force matrix (per pair of particle types).
float repulsionMatrix[numTypes][numTypes] = {
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f},
    {  0.050f,  0.100f,  0.000f, 0.050f,  0.100f,  0.000f, 0.000f}
};

//---------------------------------------------------------------------------
// Additional Global Toggles and Variables
//---------------------------------------------------------------------------

bool showLines = false;   // Toggle for drawing interaction lines.
bool showUI = true;       // Toggle for showing on-screen text.
float simulatedTime = 0.0f;

//---------------------------------------------------------------------------
// Data Structures
//---------------------------------------------------------------------------

// Particle now stores its position, velocity, and a type index.
struct Particle {
    Vector2 pos;
    Vector2 vel;
    int type;
};

// Structure for a line segment used to visualize interactions.
struct LineSegment {
    Vector2 start;
    Vector2 end;
};

std::vector<Particle> particles;
std::vector<std::vector<int>> grid;           // Spatial partition grid.
std::vector<LineSegment> interactionLines;      // Lines drawn between interacting particles.

//---------------------------------------------------------------------------
// Function: RandomizeMatrices
// Randomizes every element in both force matrices (using values between 0.0 and 0.200)
// then restarts the simulation.
//---------------------------------------------------------------------------
void RandomizeMatrices() {
    // For each particle type pair, assign a random value.
    // Modify the range here as desired.
    for (int i = 0; i < numTypes; i++) {
        for (int j = 0; j < numTypes; j++) {
            attractionMatrix[i][j] = (float)GetRandomValue(-900, 900) / 1000.0f;
            repulsionMatrix[i][j]  = (float)GetRandomValue(200, 1900) / 1000.0f;
        }
    }
}

//---------------------------------------------------------------------------
// Simulation Initialization: Create particles with random positions, velocities, and types.
//---------------------------------------------------------------------------
void InitSimulation(int count) {
    particles.clear();
    particles.reserve(count);
    for (int i = 0; i < count; i++) {
        Particle p;
        // Random position in simulation area.
        p.pos = { (float)GetRandomValue(0, simulationAreaWidth),
                  (float)GetRandomValue(0, simulationAreaHeight) };
        // Small random initial velocity.
        p.vel = { (float)GetRandomValue(-100, 100) / 100.0f,
                  (float)GetRandomValue(-100, 100) / 100.0f };
        // Randomly assign a type between 0 and numTypes-1.
        p.type = GetRandomValue(0, numTypes - 1);
        particles.push_back(p);
    }
    simulatedTime = 0.0f;
}

//---------------------------------------------------------------------------
// BuildGrid: Build a spatial partitioning grid using the current cell size.
// Uses modular arithmetic for the toroidal world.
//---------------------------------------------------------------------------
void BuildGrid() {
    int gridColumns = simulationAreaWidth / gridCellSize;
    int gridRows    = simulationAreaHeight / gridCellSize;
    grid.clear();
    grid.resize(gridColumns * gridRows);

    for (int i = 0; i < particles.size(); i++) {
        int cellX = ((int)(particles[i].pos.x / gridCellSize)) % gridColumns;
        int cellY = ((int)(particles[i].pos.y / gridCellSize)) % gridRows;
        if (cellX < 0) cellX += gridColumns;
        if (cellY < 0) cellY += gridRows;
        int index = cellY * gridColumns + cellX;
        grid[index].push_back(i);
    }
}

//---------------------------------------------------------------------------
// UpdateParticles: Process interactions (forces, collisions) and update positions.
// Uses type-specific matrices for the force calculations.
//---------------------------------------------------------------------------
void UpdateParticles(float dt) {
    interactionLines.clear();

    // Update grid cell size based on current neighborRadius.
    gridCellSize = (int)neighborRadius;
    BuildGrid();

    int gridColumns = simulationAreaWidth / gridCellSize;
    int gridRows    = simulationAreaHeight / gridCellSize;

    for (int i = 0; i < particles.size(); i++) {
        Particle &p = particles[i];
        int cellX = ((int)(p.pos.x / gridCellSize)) % gridColumns;
        int cellY = ((int)(p.pos.y / gridCellSize)) % gridRows;
        if (cellX < 0) cellX += gridColumns;
        if (cellY < 0) cellY += gridRows;

        Vector2 force = { 0.0f, 0.0f };

        // Loop over neighboring grid cells (with wrap-around).
        for (int offsetY = -1; offsetY <= 1; offsetY++) {
            for (int offsetX = -1; offsetX <= 1; offsetX++) {
                int neighborX = (cellX + offsetX + gridColumns) % gridColumns;
                int neighborY = (cellY + offsetY + gridRows) % gridRows;
                int index = neighborY * gridColumns + neighborX;
                for (int j : grid[index]) {
                    if (j == i) continue;
                    Particle &other = particles[j];

                    float dx = other.pos.x - p.pos.x;
                    float dy = other.pos.y - p.pos.y;

                    // Wrap distances for toroidal world.
                    if (dx > simulationAreaWidth / 2.0f)  dx -= simulationAreaWidth;
                    else if (dx < -simulationAreaWidth / 2.0f) dx += simulationAreaWidth;
                    if (dy > simulationAreaHeight / 2.0f) dy -= simulationAreaHeight;
                    else if (dy < -simulationAreaHeight / 2.0f) dy += simulationAreaHeight;

                    float distSq = dx*dx + dy*dy;
                    if (distSq < neighborRadius * neighborRadius && distSq > 2.0f) {
                        float dist = sqrtf(distSq);
                        float normX = dx / dist;
                        float normY = dy / dist;

                        // Optionally add a line if it doesn't span the wrapped boundary.
                        if (showLines) {
                            float rawDx = other.pos.x - p.pos.x;
                            float rawDy = other.pos.y - p.pos.y;
                            if (fabs(rawDx) <= simulationAreaWidth/2.0f &&
                                fabs(rawDy) <= simulationAreaHeight/2.0f)
                            {
                                LineSegment seg;
                                seg.start = p.pos;
                                seg.end   = other.pos;
                                interactionLines.push_back(seg);
                            }
                        }

                        int typeA = p.type;
                        int typeB = other.type;
                        float attCoeff = attractionMatrix[typeA][typeB];
                        float repCoeff = repulsionMatrix[typeA][typeB];

                        if (dist < neighborRadius * 0.3f) {
                            force.x -= repCoeff * normX;
                            force.y -= repCoeff * normY;
                        } else {
                            force.x += attCoeff * normX;
                            force.y += attCoeff * normY;
                        }

                        // Collision response.
                        if (dist < particleRadius * 2.0f && dist > 0.0f) {
                            float overlap = particleRadius * 2.0f - dist;
                            float sepX = (dx / dist) * overlap * 0.5f;
                            float sepY = (dy / dist) * overlap * 0.5f;
                            p.pos.x      -= sepX;
                            p.pos.y      -= sepY;
                            other.pos.x  += sepX;
                            other.pos.y  += sepY;
                            // p.vel.x *= 0.99f; p.vel.y *= 0.99f;
                            // other.vel.x *= 0.99f; other.vel.y *= 0.99f;
                        }
                    }
                }
            }
        }

        p.vel.x += force.x;
        p.vel.y += force.y;
        const float maxSpeed = 50.0f;
        float speed = sqrtf(p.vel.x * p.vel.x + p.vel.y * p.vel.y);
        if (speed > maxSpeed) {
            p.vel.x = (p.vel.x / speed) * maxSpeed;
            p.vel.y = (p.vel.y / speed) * maxSpeed;
        }
        p.pos.x += p.vel.x * dt;
        p.pos.y += p.vel.y * dt;

        if (p.pos.x < 0) p.pos.x += simulationAreaWidth;
        if (p.pos.x >= simulationAreaWidth) p.pos.x -= simulationAreaWidth;
        if (p.pos.y < 0) p.pos.y += simulationAreaHeight;
        if (p.pos.y >= simulationAreaHeight) p.pos.y -= simulationAreaHeight;
    }
    simulatedTime += dt;
}

//---------------------------------------------------------------------------
// Main: Setup window, handle input, update simulation and draw.
//---------------------------------------------------------------------------
int main(void)
{
    InitWindow(screenWidth, screenHeight, "Particle Life Simulation");
    SetTargetFPS(0);

    Camera2D camera = { 0 };
    camera.target = { simulationAreaWidth / 2.0f, simulationAreaHeight / 2.0f };
    camera.offset = { screenWidth / 2.0f, screenHeight / 2.0f };
    camera.rotation = 0.0f;
    camera.zoom = 5.0f;

    InitSimulation(particleCount);

    const float dt = 1.0f / 60.0f;
    const float panSpeed = 500.0f; // Pixels per second for camera panning

    while (!WindowShouldClose())
    {
        // --- Handle User Input ---
        float wheelMovement = GetMouseWheelMove();
        camera.zoom += wheelMovement * 0.1f;
        if (camera.zoom < 0.1f) camera.zoom = 0.1f;

        // Pan camera using WASD.
        if (IsKeyDown(KEY_W)) camera.target.y -= panSpeed * dt;
        if (IsKeyDown(KEY_S)) camera.target.y += panSpeed * dt;
        if (IsKeyDown(KEY_A)) camera.target.x -= panSpeed * dt;
        if (IsKeyDown(KEY_D)) camera.target.x += panSpeed * dt;

        // Restart simulation.
        if (IsKeyPressed(KEY_R)) InitSimulation(particleCount);

        // Adjust particle count.
        if (IsKeyPressed(KEY_UP)) { particleCount += 1000; InitSimulation(particleCount); }
        if (IsKeyPressed(KEY_DOWN)) { if (particleCount > 1000) { particleCount -= 1000; InitSimulation(particleCount); } }

        // Toggle interaction lines and UI text.
        if (IsKeyPressed(KEY_L)) showLines = !showLines;
        if (IsKeyPressed(KEY_T)) showUI = !showUI;

        // Adjust neighbor radius (and grid cell size) with I/K.
        if (IsKeyPressed(KEY_I)) { neighborRadius += 1.0f; }
        if (IsKeyPressed(KEY_K)) { neighborRadius -= 1.0f; if (neighborRadius < 5.0f) neighborRadius = 5.0f; }

        // Randomize matrices (and restart simulation) with M.
        if (IsKeyPressed(KEY_M)) { RandomizeMatrices(); InitSimulation(particleCount); }

        // --- Update Simulation ---
        UpdateParticles(dt);

        // --- Draw Everything ---
        BeginDrawing();
            ClearBackground(BLACK);

            BeginMode2D(camera);
                for (const Particle &p : particles)
                    //DrawCircleV(p.pos, particleRadius, particleColors[p.type]);
                    DrawPixelV(p.pos, particleColors[p.type]);
                if (showLines)
                    for (const LineSegment &line : interactionLines)
                        DrawLineV(line.start, line.end, LIGHTGRAY);
            EndMode2D();

            if (showUI)
            {
                DrawFPS(10, 10);
                int y = 40;
                DrawText(TextFormat("Particle Count: %d", particleCount), 10, y, 20, RAYWHITE); y += 20;
                DrawText(TextFormat("Neighbor Radius: %.2f", neighborRadius), 10, y, 20, RAYWHITE); y += 20;
                DrawText(TextFormat("Simulated Time: %.2f s", simulatedTime), 10, y, 20, RAYWHITE); y += 20;
                DrawText("WASD: Pan | Mouse wheel: Zoom", 10, y, 20, RAYWHITE); y += 20;
                DrawText("R: Restart | UP/DOWN: Change Count", 10, y, 20, RAYWHITE); y += 20;
                DrawText("I/K: Adjust Radius | L: Toggle Lines | T: Toggle UI", 10, y, 20, RAYWHITE); y += 20;
                DrawText("M: Randomize Matrices & Restart", 10, y, 20, RAYWHITE); y += 30;

                // Display the attraction matrix.
                DrawText("Attraction Matrix:", 10, y, 20, RAYWHITE); y += 20;
                for (int i = 0; i < numTypes; i++)
                {
                    char row[256] = "";
                    snprintf(row, sizeof(row), " [%d]: ", i);
                    for (int j = 0; j < numTypes; j++) {
                        char val[16];
                        snprintf(val, sizeof(val), "%.3f ", attractionMatrix[i][j]);
                        strcat(row, val);
                    }
                    DrawText(row, 10, y, 20, RAYWHITE);
                    y += 20;
                }
                y += 10;
                // Display the repulsion matrix.
                DrawText("Repulsion Matrix:", 10, y, 20, RAYWHITE); y += 20;
                for (int i = 0; i < numTypes; i++)
                {
                    char row[256] = "";
                    snprintf(row, sizeof(row), " [%d]: ", i);
                    for (int j = 0; j < numTypes; j++) {
                        char val[16];
                        snprintf(val, sizeof(val), "%.3f ", repulsionMatrix[i][j]);
                        strcat(row, val);
                    }
                    DrawText(row, 10, y, 20, RAYWHITE);
                    y += 20;
                }
            }
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
