#include "motion/PathPlanner.h"

#include "scene/Mesh.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

namespace pegasus {

bool segmentClear(const std::vector<PathObstacle>& obstacles, const glm::vec3& a, const glm::vec3& b, float clear) {
    const glm::vec3 d = b - a;
    const float len = glm::length(d);
    const glm::vec3 dn = len > 1e-6f ? d / len : glm::vec3(0, 0, 1);
    const glm::vec3 u = glm::normalize(glm::cross(dn, std::abs(dn.z) < 0.9f ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0)));
    const glm::vec3 v = glm::cross(dn, u);
    const glm::vec3 offs[5] = {glm::vec3(0.0f), u * clear, -u * clear, v * clear, -v * clear};
    const int nOffs = clear > 0.0f ? 5 : 1;
    for (const auto& o : obstacles) {
        if (!o.mesh) continue;
        AABB box = o.mesh->bounds.transformed(o.world);
        const glm::vec3 lo = glm::min(a, b) - glm::vec3(clear * 2.0f), hi = glm::max(a, b) + glm::vec3(clear * 2.0f);
        if (glm::any(glm::lessThan(hi, box.min)) || glm::any(glm::greaterThan(lo, box.max))) continue;
        const glm::mat4 inv = glm::inverse(o.world);
        for (int i = 0; i < nOffs; ++i) {
            const glm::vec3 la = glm::vec3(inv * glm::vec4(a + offs[i], 1.0f));
            const glm::vec3 lb = glm::vec3(inv * glm::vec4(b + offs[i], 1.0f));
            RayHit hit;
            if (len > 1e-6f && o.mesh->raycast(Ray{la, lb - la}, hit, 1.0f)) return false;
            if (o.mesh->outwardOriented && o.mesh->containsPoint(la)) return false;
        }
    }
    return true;
}

glm::vec3 toolDirection(const Seam& seam, float workDeg, float travelDeg) {
    const float w = glm::radians(workDeg), t = glm::radians(travelDeg);
    const glm::vec3 d = -seam.N * (std::cos(w) * std::cos(t)) + seam.X * std::sin(w) + seam.Y * (std::sin(t) * std::cos(w));
    return glm::normalize(d);
}

bool samplePath(const Seam& seam, const MotionDef& def, const CompiledMotion& motion, const PlanSettings& settings,
                std::vector<PathPoint>& out, std::string& err) {
    out.clear();
    if (!seam.valid) {
        err = "Önce yol (dikiş) oluşturun.";
        return false;
    }
    const float step = std::max(settings.step, 0.01f);
    const int n = std::clamp(int(std::ceil(seam.length / step)), 1, 20000);
    out.reserve(size_t(n) + 1);
    for (int i = 0; i <= n; ++i) {
        const float s = seam.length * float(i) / float(n);
        glm::vec3 off;
        float work, travel;
        motion.evaluate(def, s, seam.length > 0 ? s / seam.length : 0.0, seam.length, settings.speed, off, work, travel);
        PathPoint p;
        p.s = s;
        p.phase = PathPhase::Process;
        p.pos = seam.start + seam.Y * (s + off.y) + seam.X * off.x + seam.N * off.z;
        p.dir = toolDirection(seam, work, travel);
        out.push_back(std::move(p));
    }
    return true;
}

namespace {

IkResult solveLocal(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& start, const PathPoint& p) {
    std::vector<IkTask> tasks(2);
    IkTool t = tool;
    tasks[0].type = IkTask::Position;
    tasks[0].goal = p.pos;
    tasks[1].type = IkTask::Direction;
    tasks[1].goal = p.dir;
    if (p.maxTilt > 0.0f) { // aim the (extended) tool axis at the target, direction as close as possible
        t.tipOffset += glm::vec3(0.0f, 0.0f, p.aim);
        tasks[0].goal = p.pos + p.dir * p.aim;
        if (glm::length(p.wantDir) > 0.5f) tasks[1].goal = p.wantDir;
        tasks[1].primary = false;
    }
    IkResult r = ik.solve(robot, start, tasks, t);
    if (!r.converged) { // Blender's DLS as a fallback, then a second try from the result
        IkSolver dls = ik;
        dls.method = IkSolver::Method::DLS;
        IkResult r2 = dls.solve(robot, r.angles, tasks, t, 200);
        if (r2.positionError + r2.directionError < r.positionError + r.directionError) r = r2;
    }
    if (p.maxTilt > 0.0f) r.directionError = r.secondaryError; // report the tilt
    return r;
}

constexpr float kJumpDeg = 10.0f; // a joint change larger than this between neighbouring points is a swing

bool pointGood(const PathPoint& p, const IkResult& r, const PlanSettings& settings) {
    const float dirTol = p.maxTilt > 0.0f ? std::max(settings.dirTolerance, p.maxTilt) : settings.dirTolerance;
    return r.positionError <= settings.posTolerance && r.directionError <= dirTol;
}

float pointCost(const PathPoint& p, const IkResult& r) {
    return r.positionError + (p.maxTilt > 0.0f ? std::max(0.0f, r.directionError - p.maxTilt) : r.directionError);
}

float jointDistance(const std::vector<float>& a, const std::vector<float>& b) {
    float d = 0.0f;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) d = std::max(d, std::abs(a[i] - b[i]));
    return d;
}

// Like Blender, the Jacobian solver is local: from a bad start it can get stuck on joint
// limits. If the warm start fails, retry from a fixed set of seeds and keep the converged
// solution closest to the start configuration (predictable motion).
IkResult solvePoint(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& start, const PathPoint& p,
                    const PlanSettings& settings, bool allowRestarts) {
    auto good = [&](const IkResult& r) { return pointGood(p, r, settings); };
    IkResult best = solveLocal(ik, robot, tool, start, p);
    if (good(best) || !allowRestarts) return best;
    const auto& dofs = ik.dofJoints();
    std::mt19937 rng(1234);
    bool haveGood = false;
    for (int k = 0; k < 24; ++k) {
        std::vector<float> seed(dofs.size());
        for (size_t i = 0; i < dofs.size(); ++i) {
            const RobotJoint& J = robot.joints[dofs[i]];
            std::uniform_real_distribution<float> U(J.minDeg, J.maxDeg);
            seed[i] = k == 0 ? 0.5f * (J.minDeg + J.maxDeg) : U(rng);
        }
        IkResult r = solveLocal(ik, robot, tool, seed, p);
        if (good(r)) {
            if (!haveGood || jointDistance(r.angles, start) < jointDistance(best.angles, start)) best = r;
            haveGood = true;
            if (k >= 8) break; // enough candidates to pick a close one
        } else if (!haveGood && pointCost(p, r) < pointCost(p, best)) {
            best = r;
        }
    }
    return best;
}

float maxJointStep(const std::vector<float>& a, const std::vector<float>& b) {
    float d = 0.0f;
    for (size_t k = 0; k < a.size() && k < b.size(); ++k) d = std::max(d, std::abs(a[k] - b[k]));
    return d;
}

// The warm-started forward pass keeps one IK branch until it can no longer reach a point; the
// restart then lands on another branch and the arm swings over (wrist roll from one limit to the
// other, base turned around ...). Re-solving the stretch *before* such a jump backwards from the
// new configuration moves the swing to the start of the stretch, where the tool is off anyway.
void repairJumps(const IkSolver& ik, const Robot& robot, const IkTool& tool, std::vector<PathPoint>& pts, const PlanSettings& settings) {
    for (size_t i = 1; i < pts.size(); ++i) {
        if (!pts[i].solved || !pts[i - 1].solved || !pts[i].reachable || !pts[i - 1].reachable) continue;
        if (maxJointStep(pts[i].q, pts[i - 1].q) <= kJumpDeg) continue;
        // stretch start: first point of the reachable run that ends at i-1
        size_t a = i - 1;
        while (a > 0 && pts[a - 1].solved && pts[a - 1].reachable && maxJointStep(pts[a].q, pts[a - 1].q) <= kJumpDeg) --a;
        std::vector<PathPoint> trial(pts.begin() + std::ptrdiff_t(a), pts.begin() + std::ptrdiff_t(i));
        std::vector<float> q = pts[i].q;
        bool ok = true;
        for (size_t k = trial.size(); k-- > 0;) {
            PathPoint& t = trial[k];
            const IkResult r = solveLocal(ik, robot, tool, q, t);
            if (!pointGood(t, r, settings) || maxJointStep(r.angles, q) > kJumpDeg) {
                ok = false;
                break;
            }
            t.q = r.angles;
            t.posErr = r.positionError;
            t.dirErr = r.directionError;
            q = r.angles;
        }
        if (!ok) continue;
        for (size_t k = 0; k < trial.size(); ++k) {
            PathPoint& t = trial[k];
            if (t.maxTilt > 0.0f) { // actual pose of the new solution
                const glm::mat4 F = ik.toolFrame(robot, t.q, tool);
                const glm::vec3 target = pts[a + k].pos + pts[a + k].dir * t.aim;
                t.dir = glm::normalize(glm::vec3(F[2]));
                t.pos = target - t.dir * t.aim;
            }
            pts[a + k] = t;
        }
    }
}

} // namespace

int solvePath(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& startQ,
              std::vector<PathPoint>& points, const PlanSettings& settings, float budgetMs) {
    std::vector<float> q = startQ;
    int reachable = 0;
    bool prevReachable = true;
    const auto t0 = std::chrono::steady_clock::now();
    for (auto& p : points) {
        if (budgetMs > 0.0f &&
            std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count() > budgetMs)
            break; // remaining points stay unsolved
        // restarts only where a jump may help: at the start, or when leaving a reachable stretch
        if (p.maxTilt > 0.0f && glm::length(p.wantDir) < 0.5f) p.wantDir = p.dir;
        const IkResult r = solvePoint(ik, robot, tool, q, p, settings, prevReachable);
        p.solved = true;
        p.q = r.angles;
        p.posErr = r.positionError;
        p.dirErr = r.directionError;
        p.reachable = pointGood(p, r, settings);
        if (p.maxTilt > 0.0f && p.reachable) { // store the actual (possibly tilted) pose
            const glm::mat4 F = ik.toolFrame(robot, r.angles, tool);
            const glm::vec3 target = p.pos + p.dir * p.aim;
            p.dir = glm::normalize(glm::vec3(F[2]));
            p.pos = target - p.dir * p.aim;
        }
        reachable += p.reachable;
        prevReachable = p.reachable;
        if (p.reachable || reachable == 0) q = r.angles; // warm start: keeps the configuration continuous along the path
    }
    repairJumps(ik, robot, tool, points, settings);
    return reachable;
}

std::vector<float> Trajectory::sample(float t, int* index) const {
    if (keys.empty()) return {};
    if (t <= keys.front().time) {
        if (index) *index = 0;
        return keys.front().q;
    }
    if (t >= keys.back().time) {
        if (index) *index = int(keys.size()) - 1;
        return keys.back().q;
    }
    const auto it = std::upper_bound(keys.begin(), keys.end(), t, [](float v, const PathPoint& k) { return v < k.time; });
    const size_t i = size_t(it - keys.begin()) - 1;
    const PathPoint &a = keys[i], &b = keys[i + 1];
    const float f = b.time > a.time ? (t - a.time) / (b.time - a.time) : 1.0f;
    std::vector<float> q(a.q.size());
    for (size_t k = 0; k < q.size(); ++k) q[k] = a.q[k] + (b.q[k] - a.q[k]) * f;
    if (index) *index = int(i);
    return q;
}

bool buildTrajectory(const IkSolver& ik, const Robot& robot, const IkTool& tool, const std::vector<float>& startQ,
                     const std::vector<PathPoint>& process, const PlanSettings& settings, Trajectory& out, std::string& err) {
    out = Trajectory{};
    if (process.empty()) {
        err = "Yol boş.";
        return false;
    }
    const float step = std::max(settings.step, 0.05f);
    auto linePoints = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& dir, PathPhase phase, std::vector<PathPoint>& dst) {
        const int n = std::max(1, int(std::ceil(glm::length(b - a) / (step * 2.0f))));
        for (int i = 1; i <= n; ++i) {
            PathPoint p;
            p.phase = phase;
            p.pos = glm::mix(a, b, float(i) / float(n));
            p.dir = dir;
            dst.push_back(p);
        }
    };
    // Cartesian points: approach -> process -> retreat
    std::vector<PathPoint> pts;
    PathPoint approachStart;
    approachStart.phase = PathPhase::Approach;
    approachStart.pos = process.front().pos - process.front().dir * settings.approach;
    approachStart.dir = process.front().dir;
    pts.push_back(approachStart);
    linePoints(approachStart.pos, process.front().pos, process.front().dir, PathPhase::Approach, pts);
    pts.pop_back(); // the last approach point is the first process point
    for (const auto& p : process) pts.push_back(p);
    linePoints(process.back().pos, process.back().pos - process.back().dir * settings.approach, process.back().dir, PathPhase::Retreat, pts);

    const int reachable = solvePath(ik, robot, tool, startQ, pts, settings);
    if (reachable < int(pts.size())) {
        int bad = 0;
        for (const auto& p : pts) bad += !p.reachable;
        err = std::to_string(bad) + " nokta robotun erişemediği konumda (yol yine de çalıştırılabilir, en yakın pozlar kullanılır).";
    }

    // tilt tolerant work (spray): a pose that was not reached is not used at all (it may be any
    // configuration the solver gave up in); the arm moves from the last reached pose straight to the
    // next one instead, tool off (that move is checked against the obstacles below)
    {
        std::vector<PathPoint> kept;
        kept.reserve(pts.size());
        bool gap = false;
        for (auto& p : pts) {
            if (!p.reachable && p.maxTilt > 0.0f) {
                gap = true;
                continue;
            }
            if (gap && !kept.empty() && p.maxTilt > 0.0f) p.phase = PathPhase::Link;
            gap = false;
            kept.push_back(p);
        }
        if (!kept.empty()) pts.swap(kept);
    }
    // switch the tool off while the arm swings into another configuration, and back off along the
    // tool axis before / after the swing so the tool does not sweep through the workpiece
    {
        std::vector<PathPoint> withRetreats;
        withRetreats.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i > 0 && pts[i].maxTilt > 0.0f && pts[i - 1].maxTilt > 0.0f && maxJointStep(pts[i].q, pts[i - 1].q) > kJumpDeg) {
                for (int side = 0; side < 2; ++side) {
                    const PathPoint& base = side == 0 ? pts[i - 1] : pts[i];
                    PathPoint r = base;
                    r.phase = PathPhase::Link;
                    r.maxTilt = 0.0f;
                    r.pos = base.pos - base.dir * settings.approach;
                    const IkResult res = solveLocal(ik, robot, tool, base.q, r);
                    if (!pointGood(r, res, settings) || maxJointStep(res.angles, base.q) > 3.0f * kJumpDeg) continue;
                    r.q = res.angles;
                    r.solved = r.reachable = true;
                    r.maxTilt = base.maxTilt;
                    withRetreats.push_back(r);
                }
                pts[i].phase = PathPhase::Link; // the swing itself: tool off
            }
            withRetreats.push_back(pts[i]);
        }
        pts.swap(withRetreats);
    }
    for (size_t i = 0; i < pts.size(); ++i) {
        PathPoint& p = pts[i];
        if (p.phase != PathPhase::Process || p.maxTilt <= 0.0f) continue;
        bool off = !p.reachable;
        if (!off && i > 0 && pts[i - 1].phase == PathPhase::Process) {
            float dq = 0.0f;
            for (size_t k = 0; k < p.q.size() && k < pts[i - 1].q.size(); ++k) dq = std::max(dq, std::abs(p.q[k] - pts[i - 1].q[k]));
            off = dq > kJumpDeg;
        }
        if (off) p.phase = PathPhase::Link;
    }

    // tool-off moves must not pass through the workpieces: sample the joint-space motion and, if
    // the tool tip would enter an obstacle, back off along the tool axis (or lift) before and after
    int blocked = 0;
    if (settings.obstacles && !settings.obstacles->empty()) {
        const auto& obs = *settings.obstacles;
        const float clear = 2.0f;
        auto tipOf = [&](const std::vector<float>& q) { return glm::vec3(ik.toolFrame(robot, q, tool)[3]); };
        auto moveClear = [&](const std::vector<float>& qa, const std::vector<float>& qb) {
            const int n = std::max(4, int(std::ceil(maxJointStep(qa, qb) / 2.0f)));
            glm::vec3 prev = tipOf(qa);
            std::vector<float> q(qa.size());
            for (int s = 1; s <= n; ++s) {
                for (size_t k = 0; k < q.size(); ++k) q[k] = qa[k] + (qb[k] - qa[k]) * float(s) / float(n);
                const glm::vec3 t = tipOf(q);
                if (!segmentClear(obs, prev, t, clear)) return false;
                prev = t;
            }
            return true;
        };
        auto offsetPose = [&](const PathPoint& b, const glm::vec3& off, PathPoint& outP) {
            outP = b;
            outP.phase = PathPhase::Link;
            outP.maxTilt = 180.0f; // tool off: only the tip position matters
            outP.aim = 0.0f;
            outP.pos = b.pos + off;
            const IkResult r = solveLocal(ik, robot, tool, b.q, outP);
            if (!pointGood(outP, r, settings)) return false;
            outP.q = r.angles;
            outP.solved = outP.reachable = true;
            outP.maxTilt = std::max(b.maxTilt, 1.0f);
            const glm::mat4 F = ik.toolFrame(robot, r.angles, tool);
            outP.pos = glm::vec3(F[3]);
            outP.dir = glm::normalize(glm::vec3(F[2]));
            outP.posErr = r.positionError;
            outP.dirErr = r.directionError;
            return true;
        };
        std::vector<PathPoint> safe;
        safe.reserve(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) {
                const PathPoint &a = pts[i - 1], &b = pts[i];
                const bool toolOff = a.phase == PathPhase::Link || b.phase == PathPhase::Link ||
                                     ((a.maxTilt > 0.0f || b.maxTilt > 0.0f) && maxJointStep(a.q, b.q) > kJumpDeg);
                if (toolOff && !a.q.empty() && !b.q.empty() && !moveClear(a.q, b.q)) {
                    bool fixed = false;
                    const float base = std::max(settings.approach, 10.0f);
                    for (float k : {1.0f, 2.0f, 4.0f, 7.0f}) {
                        for (int mode = 0; mode < 3 && !fixed; ++mode) {
                            // 0: back off along the tool axes, 1: lift, 2: back off + over a high midpoint
                            const glm::vec3 offA = mode == 1 ? glm::vec3(0, 0, base * k) : -a.dir * base * k;
                            const glm::vec3 offB = mode == 1 ? glm::vec3(0, 0, base * k) : -b.dir * base * k;
                            PathPoint A2, B2, M;
                            if (!offsetPose(a, offA, A2) || !offsetPose(b, offB, B2)) continue;
                            if (mode < 2) {
                                if (moveClear(a.q, A2.q) && moveClear(A2.q, B2.q) && moveClear(B2.q, b.q)) {
                                    safe.push_back(A2);
                                    safe.push_back(B2);
                                    fixed = true;
                                }
                                continue;
                            }
                            PathPoint mid = A2;
                            mid.pos = 0.5f * (A2.pos + B2.pos);
                            if (!offsetPose(mid, glm::vec3(0, 0, base * k), M)) continue;
                            if (moveClear(a.q, A2.q) && moveClear(A2.q, M.q) && moveClear(M.q, B2.q) && moveClear(B2.q, b.q)) {
                                safe.push_back(A2);
                                safe.push_back(M);
                                safe.push_back(B2);
                                fixed = true;
                            }
                        }
                        if (fixed) break;
                    }
                    if (!fixed && !startQ.empty()) {
                        // last resort, like an industrial "safe position": back off, swing over through the
                        // pose the program started from (usually high and away from the workpieces), come back
                        PathPoint home = a;
                        home.q = startQ;
                        const glm::mat4 F = ik.toolFrame(robot, startQ, tool);
                        home.pos = glm::vec3(F[3]);
                        home.dir = glm::normalize(glm::vec3(F[2]));
                        home.phase = PathPhase::Link;
                        home.solved = home.reachable = true;
                        for (float k : {0.0f, 1.0f, 2.0f, 4.0f}) {
                            PathPoint A2 = a, B2 = b;
                            if (k > 0.0f && (!offsetPose(a, -a.dir * base * k, A2) || !offsetPose(b, -b.dir * base * k, B2))) continue;
                            if (moveClear(a.q, A2.q) && moveClear(A2.q, home.q) && moveClear(home.q, B2.q) && moveClear(B2.q, b.q)) {
                                if (k > 0.0f) safe.push_back(A2);
                                safe.push_back(home);
                                if (k > 0.0f) safe.push_back(B2);
                                fixed = true;
                                break;
                            }
                        }
                    }
                    if (!fixed) ++blocked;
                }
            }
            safe.push_back(pts[i]);
        }
        pts.swap(safe);
    }
    if (blocked > 0) {
        if (!err.empty()) err += " ";
        err += std::to_string(blocked) + " geçişte parçalara değmeyen bir yol bulunamadı (çarpışma riski).";
    }

    // timing
    PathPoint first;
    first.phase = PathPhase::Joint;
    first.q = startQ;
    first.pos = glm::vec3(ik.toolFrame(robot, startQ, tool)[3]);
    first.time = 0.0f;
    first.solved = first.reachable = true;
    out.keys.push_back(first);
    float maxDelta = 0.0f;
    for (size_t k = 0; k < startQ.size() && k < pts.front().q.size(); ++k) maxDelta = std::max(maxDelta, std::abs(pts.front().q[k] - startQ[k]));
    float t = std::max(0.3f, maxDelta / std::max(settings.jointSpeed, 1.0f));
    pts.front().time = t;
    out.keys.push_back(pts.front());
    for (size_t i = 1; i < pts.size(); ++i) {
        const float v = pts[i].phase == PathPhase::Process ? settings.speed : settings.approachSpeed;
        const float dist = glm::length(pts[i].pos - pts[i - 1].pos);
        // also respect the joint speed (large reorientations)
        float dq = 0.0f;
        for (size_t k = 0; k < pts[i].q.size(); ++k) dq = std::max(dq, std::abs(pts[i].q[k] - pts[i - 1].q[k]));
        t += std::max(dist / std::max(v, 0.01f), dq / std::max(settings.jointSpeed, 1.0f));
        pts[i].time = t;
        out.keys.push_back(pts[i]);
    }
    out.duration = t;
    return true;
}

} // namespace pegasus
