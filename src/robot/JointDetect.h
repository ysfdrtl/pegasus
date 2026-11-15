#pragma once
// Automatic joint detection for robot arms exported as an assembly (one STL per part, all in
// assembly coordinates) and geometric fitting helpers used when editing joint axes by hand.
//
// Idea: shafts, bearings and pins show up as circular feature-edge loops. Two neighbouring
// parts that share a circle axis are connected there:
//   - one shared axis           -> revolute joint about that axis
//   - two or more shared axes   -> fixed (two pins lock rotation)
//   - part mostly inside parent -> fixed (motor in a housing)
// Closed loops between parts are classified as parallelogram linkages or gear pairs and turned
// into mimic joints so that a mechanism (e.g. a gripper) is driven by a single angle.

#include "robot/Robot.h"

#include <string>
#include <vector>

namespace pegasus {

struct Circle {
    glm::vec3 center{0.0f};
    glm::vec3 normal{0.0f, 0.0f, 1.0f};
    float radius = 0.0f;
};

std::vector<Circle> findCircles(const Mesh& mesh);

// Loads every STL in `folder` and builds a robot with detected joints. `log` receives a human
// readable description of what was detected.
std::unique_ptr<Robot> detectRobot(const std::string& folder, std::string& err, std::vector<std::string>& log);

// Runs the structure detection on already loaded parts (meshes in model coordinates).
void detectRobotStructure(Robot& robot, std::vector<std::string>& log);

// Fits a circle through >= 3 points: centre, plane normal and radius.
bool fitCircle(const std::vector<glm::vec3>& pts, glm::vec3& center, glm::vec3& normal, float& radius);

// Fits an axis to a selected surface: a cylindrical patch gives its axis, a planar patch gives
// its normal through the centroid. `cylindrical` reports which case was used.
bool fitAxisFromSurface(const std::vector<glm::vec3>& pts, const std::vector<glm::vec3>& normals,
                        const std::vector<float>& weights, glm::vec3& origin, glm::vec3& axis, bool& cylindrical);

// Strips a common prefix such as "Robotic Arm - " from part names.
std::vector<std::string> prettyPartNames(const std::vector<std::string>& stems);

} // namespace pegasus
