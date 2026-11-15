#pragma once
// Inverse kinematics solver modelled after Blender's legacy IK solver (intern/iksolver):
//
//   Blender                         Pegasus
//   IK_QSegment (revolute DoF)      IkSolver::Dof      - axis, limits, weight, lock state
//   IK_QPositionTask                IkTask::Position   - tool tip position (clamped step)
//   IK_QOrientationTask             IkTask::Orientation- full tool orientation
//   (5-DoF arms)                    IkTask::Direction  - tool axis direction, free spin about it
//   IK_QJacobian::InvertSDLS/DLS    invertSDLS / invertDLS on a Jacobi SVD
//   IK_QJacobian::SubTask           secondary tasks projected into the null space
//   IK_QJacobianSolver::Solve       solve(): scale, iterate, clamp + lock violating joints
//
// The chain is every independent joint between the robot base and the TCP part. Mimic joints
// inside the chain contribute to the column of their driver (multiplier applied).

#include "robot/Robot.h"

#include <glm/glm.hpp>

#include <vector>

namespace pegasus {

struct IkTool {
    glm::vec3 tipOffset{0.0f}; // tool tip in the TCP frame (mm); tool axis = TCP frame +Z
};

struct IkTask {
    enum Type { Position, Direction, Orientation };
    Type type = Position;
    bool primary = true;            // false: solved as well as possible in the null space of the primary tasks
    float weight = 1.0f;
    glm::vec3 goal{0.0f};           // Position: world point; Direction: world unit vector (tool +Z)
    glm::mat3 goalRotation{1.0f};   // Orientation: world rotation of the tool frame
};

struct IkResult {
    bool converged = false;
    int iterations = 0;
    float positionError = 0.0f;  // mm
    float directionError = 0.0f; // degrees (primary tasks)
    float secondaryError = 0.0f; // degrees: remaining error of secondary Direction tasks
    std::vector<float> angles;   // degrees, one per DoF (same order as dofJoints())
};

class IkSolver {
public:
    enum class Method { SDLS, DLS };

    // Builds the chain from the robot base to the TCP part. Returns false if there is none.
    bool setup(const Robot& robot);
    const std::vector<int>& dofJoints() const { return dofJoints_; }
    int dofCount() const { return int(dofJoints_.size()); }

    // Current angles of the chain DoFs taken from the robot (degrees).
    std::vector<float> currentAngles(const Robot& robot) const;
    void applyAngles(Robot& robot, const std::vector<float>& anglesDeg) const;

    // Tool frame (tip position, columns = tool axes) for the given DoF angles.
    glm::mat4 toolFrame(const Robot& robot, const std::vector<float>& anglesDeg, const IkTool& tool) const;

    IkResult solve(const Robot& robot, const std::vector<float>& startDeg, const std::vector<IkTask>& tasks,
                   const IkTool& tool, int maxIterations = 120) const;

    // Optional secondary preference (null-space task): pulls joints towards these angles.
    std::vector<float> restPoseDeg;
    float restWeight = 0.05f;
    Method method = Method::SDLS;

private:
    struct ChainJoint {
        glm::vec3 origin{0.0f}, axis{0.0f, 0.0f, 1.0f}; // model coordinates
        int dof = -1;                                  // -1: fixed / not driven
        float multiplier = 1.0f, offsetDeg = 0.0f;
    };
    // FK of the chain; fills per-joint world origin/axis (pre-rotation) and returns the TCP part matrix.
    glm::mat4 chainFK(const Robot& robot, const std::vector<double>& rad, std::vector<glm::vec3>* jointPos,
                      std::vector<glm::vec3>* jointAxis) const;

    std::vector<int> dofJoints_;
    std::vector<ChainJoint> chain_;
    std::vector<float> minRad_, maxRad_;
};

} // namespace pegasus
