#pragma once
// Turns a seam + motion formula into tool poses, solves IK along the path (warm started from
// the previous point, Blender-style SDLS with a DLS fallback) and builds a timed trajectory:
//   joint move -> approach (along the normal) -> process path -> retreat

#include "motion/MotionLibrary.h"
#include "motion/Seam.h"
#include "robot/IkSolver.h"

#include <string>
#include <vector>

namespace pegasus {

class Mesh;

// A workpiece the tool must not pass through (world placement of a mesh).
struct PathObstacle {
    const Mesh* mesh = nullptr;
    glm::mat4 world{1.0f};
};

// True if the segment a-b (and copies of it `clear` mm around it) does not cross any obstacle.
bool segmentClear(const std::vector<PathObstacle>& obstacles, const glm::vec3& a, const glm::vec3& b, float clear);

struct PlanSettings {
    float step = 0.5f;          // mm between path samples
    float speed = 5.0f;         // process speed, mm/s
    float approach = 20.0f;     // approach / retreat distance along the normal, mm
    float approachSpeed = 30.0f;
    float jointSpeed = 60.0f;   // deg/s for the free joint move
    float posTolerance = 0.1f;  // mm, a point counts as reachable below this error
    float dirTolerance = 1.0f;  // deg
    // Optional: tool-off moves (links, swings, the first joint move) are checked against these and
    // routed around them (backing off along the tool axis / lifting).
    const std::vector<PathObstacle>* obstacles = nullptr;
};

// Process = working (painting / welding), Link = moving between passes or faces (tool off)
enum class PathPhase { Joint = 0, Approach = 1, Process = 2, Retreat = 3, Link = 4 };

struct PathPoint {
    float s = 0.0f;            // distance along the seam (process points)
    glm::vec3 pos{0.0f};       // tool tip target (world)
    glm::vec3 dir{0, 0, -1};   // tool axis target (world)
    PathPhase phase = PathPhase::Process;
    // Tilt tolerant points (spray): the tool axis may deviate from `dir` by up to maxTilt degrees,
    // but it always aims at the target point pos + dir * aim from exactly `aim` mm (position is the
    // primary IK task, the direction a secondary one). After solving, pos / dir hold the actual pose.
    float aim = 0.0f, maxTilt = 0.0f;
    glm::vec3 wantDir{0.0f}; // requested direction of a tilt tolerant point (kept when dir becomes the actual one)
    float time = 0.0f;
    bool solved = false, reachable = false;
    float posErr = 0.0f, dirErr = 0.0f; // dirErr: tilt from the requested direction for tilt tolerant points
    std::vector<float> q;      // joint angles (deg), IK DoF order
};

struct Trajectory {
    std::vector<PathPoint> keys;
    float duration = 0.0f;
    bool valid() const { return keys.size() >= 2; }
    // Interpolated joint angles at time t; `index` receives the segment start key.
    std::vector<float> sample(float t, int* index = nullptr) const;
};

// Tool axis for a seam frame and tilt angles: -N tilted by work (towards +X) and travel (+Y) angles.
glm::vec3 toolDirection(const Seam& seam, float workDeg, float travelDeg);

bool samplePath(const Seam& seam, const MotionDef& def, const CompiledMotion& motion, const PlanSettings& settings,
                std::vector<PathPoint>& out, std::string& err);

// Solves IK for every point (in order). Returns the number of reachable points.
// With budgetMs > 0 solving stops after that time; the remaining points stay unsolved (preview).
int solvePath(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& startQ,
              std::vector<PathPoint>& points, const PlanSettings& settings, float budgetMs = -1.0f);

// Approach / retreat move along the tool axis by settings.approach.
bool buildTrajectory(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& startQ,
                     const std::vector<PathPoint>& path, const PlanSettings& settings, Trajectory& out, std::string& err);

} // namespace pegasus
