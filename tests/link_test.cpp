// Whole chain with the real robot model: scan two adjacent faces of a cube, solve IK, build the
// trajectory and sample it finely. The tool tip must never enter the cube, and the arm must not
// swing over (> 90 degrees between two keys) while the tool is on.
#include "check.h"

#include "motion/PathPlanner.h"
#include "motion/SurfaceScan.h"
#include "robot/IkSolver.h"
#include "robot/Robot.h"
#include "scene/Mesh.h"
#include "scene/Primitives.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <map>

using namespace pegasus;

int main() {
    const std::string folder = PEGASUS_ROBOT_DIR;
    std::string err;
    auto robot = Robot::load(folder + "/robot.pegasus", folder, err);
    CHECK(robot != nullptr, "robot.pegasus yuklendi " + err);
    if (!robot) return pegasus_test::finish("link_test");
    robot->solveFK();
    IkSolver ik;
    ik.setup(*robot);
    IkTool tool;
    tool.tipOffset = glm::vec3(0, 0, 25); // spray gun
    MeshPtr box = makeBox(glm::vec3(40.0f));

    for (float standoff : {20.0f, 40.0f, 60.0f}) {
        char title[64];
        std::snprintf(title, sizeof(title), "uzaklik %.0f mm: 24 yerlesim", standoff);
        section(title);
        int through = 0, swingsOn = 0, blocked = 0, cases = 0, reachSum = 0, ptsSum = 0;
        for (float dist : {300.0f, 340.0f, 380.0f})
            for (float yaw : {0.0f, 30.0f, 45.0f, -30.0f})
                for (float side : {-1.0f, 1.0f}) {
                    const glm::mat4 W = glm::translate(glm::mat4(1.0f), glm::vec3(dist, 0, 20)) *
                                        glm::rotate(glm::mat4(1.0f), glm::radians(yaw), glm::vec3(0, 0, 1));
                    // faces: the one looking at the robot (-X) and an adjacent side (+-Y)
                    ScanFace f;
                    std::map<uint32_t, uint32_t> remap;
                    for (uint32_t t = 0; t < box->triangles.size(); ++t) {
                        const glm::vec3 n = box->triNormals[t];
                        if (glm::dot(n, glm::vec3(-1, 0, 0)) < 0.9f && glm::dot(n, glm::vec3(0, side, 0)) < 0.9f) continue;
                        glm::uvec3 idx;
                        for (int k = 0; k < 3; ++k) {
                            const uint32_t v = box->triangles[t][k];
                            auto it = remap.find(v);
                            if (it == remap.end()) {
                                it = remap.emplace(v, uint32_t(f.verts.size())).first;
                                f.verts.push_back(glm::vec3(W * glm::vec4(box->positions[v], 1.0f)));
                            }
                            idx[k] = it->second;
                        }
                        f.tris.push_back(idx);
                    }
                    const std::vector<PathObstacle> obs = {{box.get(), W}};
                    MotionDef d;
                    d.pattern = "zigzag";
                    for (auto [n, v] : std::initializer_list<std::pair<const char*, double>>{
                             {"offset", 5}, {"spacing", 10}, {"standoff", standoff}, {"angle", 0}, {"lift", 20}, {"crease", 60}, {"tilt", 30}}) {
                        MotionParam p;
                        p.name = n;
                        p.value = v;
                        d.params.push_back(p);
                    }
                    CompiledMotion cm;
                    std::string errs[CompiledMotion::Count];
                    cm.compile(d, errs);
                    PlanSettings ps;
                    ps.step = 2.0f;
                    ps.speed = 40.0f;
                    ps.approachSpeed = 60.0f;
                    ps.obstacles = &obs;
                    std::vector<PathPoint> pts;
                    ScanStats st;
                    const glm::vec3 from(0, 0, 250);
                    if (!planSurfaceScan({f}, d, cm, ps, pts, st, err, &from, &obs)) {
                        CHECK(false, "plan: " + err);
                        continue;
                    }
                    const std::vector<float> q0 = ik.currentAngles(*robot);
                    reachSum += solvePath(ik, *robot, tool, q0, pts, ps);
                    ptsSum += int(pts.size());
                    Trajectory tr;
                    std::string e2;
                    buildTrajectory(ik, *robot, tool, q0, pts, ps, tr, e2);
                    blocked += e2.find("bulunamad") != std::string::npos;
                    const glm::mat4 inv = glm::inverse(W);
                    bool in = false;
                    for (size_t k = 1; k < tr.keys.size(); ++k) {
                        const PathPoint &a = tr.keys[k - 1], &b = tr.keys[k];
                        float dq = 0.0f;
                        for (size_t j = 0; j < a.q.size() && j < b.q.size(); ++j) dq = std::max(dq, std::abs(a.q[j] - b.q[j]));
                        if (dq > 90.0f && a.phase == PathPhase::Process && b.phase == PathPhase::Process) ++swingsOn;
                        const int n = std::max(10, int(dq / 2.0f));
                        for (int s = 0; s <= n && !in; ++s) {
                            const float t = a.time + (b.time - a.time) * float(s) / float(n);
                            const glm::vec3 l = glm::vec3(inv * ik.toolFrame(*robot, tr.sample(t), tool)[3]);
                            in = std::abs(l.x) < 20 && std::abs(l.y) < 20 && l.z > 0 && l.z < 40;
                        }
                    }
                    if (in || e2.find("bulunamad") != std::string::npos) std::printf("    -> dist %.0f yaw %.0f side %.0f: icinde %d, %s\n", dist, yaw, side, int(in), e2.c_str());
                    through += in;
                    ++cases;
                }
        CHECK(through == 0, "takim ucu hicbir yerlesimde kupun icine girmiyor (" + std::to_string(through) + " / " + std::to_string(cases) + ")");
        CHECK(swingsOn == 0, "takim acikken 90 dereceden buyuk donus yok");
        CHECK(blocked == 0, "her gecis icin engelsiz rota bulundu");
        std::printf("    erisilebilir nokta: %d / %d\n", reachSum, ptsSum);
    }
    return pegasus_test::finish("link_test");
}
