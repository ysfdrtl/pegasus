#pragma once
// Robot arm model: a tree of rigid parts (one STL each) connected by joints.
//
// All joint data is stored in *model coordinates* = the assembly coordinates of the STL files
// in the home pose (every joint angle 0). Forward kinematics uses the product-of-exponentials
// form:   L(part) = L(parent) * T(o) * R(axis, angle) * T(-o)
// and a part's world matrix is   mount * modelToMount * L(part).
//
// Mechanisms (gears, parallelogram linkages) are expressed with "mimic" joints whose angle is
// multiplier * angle(driver) + offset, like URDF mimic joints.

#include "scene/Mesh.h"

#include <glm/glm.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace pegasus {

enum class JointType { Fixed = 0, Revolute = 1 };

struct RobotPart {
    std::string file;      // STL file name inside the robot folder
    std::string name;      // display name
    MeshPtr mesh;          // vertices in model coordinates
    uint32_t objectId = 0; // scene object showing this part
    int parent = -1;       // parent part (-1 = root)
    int joint = -1;        // joint connecting this part to its parent
};

struct RobotJoint {
    std::string name;
    int part = -1; // child part moved by this joint
    JointType type = JointType::Revolute;
    glm::vec3 origin{0.0f};       // a point on the axis (model coords, home pose)
    glm::vec3 axis{0.0f, 0.0f, 1.0f};
    float minDeg = -90.0f, maxDeg = 90.0f;
    float angleDeg = 0.0f;        // commanded angle (drivers); effective angle is computed for mimics
    int mimic = -1;               // driver joint index, -1 = independent
    float multiplier = 1.0f, offsetDeg = 0.0f;
    bool motorized = false;       // detection hint: the axis is shared with a fixed (motor) part

    bool isDriver() const { return type == JointType::Revolute && mimic < 0; }
};

class Robot {
public:
    std::string name = "Robot Kol";
    std::string folder;          // folder containing the STL parts
    int upAxis = 1;              // model up axis: 0 = X, 1 = Y, 2 = Z
    glm::mat4 mount{1.0f};       // placement of the robot base in the world (rigid)
    int tcpPart = -1;            // tool centre point, attached to this part
    glm::vec3 tcpPoint{0.0f};    // model coords

    std::vector<RobotPart> parts;
    std::vector<RobotJoint> joints;

    // ---- structure helpers
    int root() const;
    std::vector<int> topoOrder() const; // parents before children
    std::vector<int> drivers() const;   // independent joints, ordered from base to tip
    int depth(int part) const;
    bool isAncestor(int ancestor, int part) const;
    int jointOfPart(int part) const { return part >= 0 && part < int(parts.size()) ? parts[part].joint : -1; }
    std::string validate() const; // empty if OK, otherwise a description of the problem

    // ---- kinematics
    float effectiveAngle(int joint) const;
    void setDriverAngle(int joint, float deg); // clamps to limits
    void resetPose();
    void solveFK();
    const glm::mat4& partModel(int part) const { return model_[part]; } // L(part)
    glm::mat4 modelToMount() const;                                      // up-axis + base centering
    glm::mat4 modelToWorld() const { return mount * modelToMount(); }
    glm::mat4 partWorld(int part) const { return modelToWorld() * model_[part]; }
    glm::vec3 jointOriginWorld(int joint) const;
    glm::vec3 jointAxisWorld(int joint) const;
    glm::mat4 tcpWorld() const; // TCP frame; equals the mount orientation in the home pose

    // Converts a world point/direction to model coordinates of the given part's frame.
    glm::vec3 worldToPartModel(int part, const glm::vec3& p) const;
    glm::vec3 worldDirToPartModel(int part, const glm::vec3& d) const;

    // Reverses the positive direction of a driver (and all of its mimic joints).
    void flipDriverDirection(int joint);

    // ---- collision helpers (pairs of part indices that are never reported)
    std::vector<std::pair<int, int>> ignoredPairs;
    bool ignores(int a, int b) const;
    void computeIgnoredPairs(); // uses the home pose
    bool selfCollides() const;  // with the current pose, ignoring ignoredPairs
    bool belowGround(float tolerance = 1.0f) const;
    // Sweeps a driver in both directions (within +-range) and narrows its limits to the
    // last pose without self collision / ground contact.
    void autoLimits(int driver, float rangeDeg = 90.0f, float stepDeg = 2.0f);

    // ---- persistence (text file "robot.pegasus" next to the STL parts)
    bool save(const std::string& path, std::string& err) const;
    // Parses a config; meshes are loaded from `partsFolder` (defaults to the config's folder).
    static std::unique_ptr<Robot> load(const std::string& path, const std::string& partsFolder, std::string& err);

private:
    std::vector<glm::mat4> model_;
};

} // namespace pegasus
