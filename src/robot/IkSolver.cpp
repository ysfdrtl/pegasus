#include "robot/IkSolver.h"

#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace pegasus {

namespace {

constexpr double kEps = 1e-10;

// Small dense matrix, row major.
struct Mat {
    int rows = 0, cols = 0;
    std::vector<double> a;
    Mat() = default;
    Mat(int r, int c) : rows(r), cols(c), a(size_t(r) * c, 0.0) {}
    double& operator()(int r, int c) { return a[size_t(r) * cols + c]; }
    double operator()(int r, int c) const { return a[size_t(r) * cols + c]; }
};

// One-sided Jacobi SVD: J (m x n) = U (m x n) * diag(w) * V^T (n x n).
void svd(const Mat& J, Mat& U, std::vector<double>& w, Mat& V) {
    const int m = J.rows, n = J.cols;
    U = J;
    V = Mat(n, n);
    for (int i = 0; i < n; ++i) V(i, i) = 1.0;
    for (int sweep = 0; sweep < 60; ++sweep) {
        double off = 0.0;
        for (int p = 0; p < n - 1; ++p) {
            for (int q = p + 1; q < n; ++q) {
                double alpha = 0.0, beta = 0.0, gamma = 0.0;
                for (int i = 0; i < m; ++i) {
                    alpha += U(i, p) * U(i, p);
                    beta += U(i, q) * U(i, q);
                    gamma += U(i, p) * U(i, q);
                }
                if (alpha < 1e-22 || beta < 1e-22) continue; // (numerically) zero column: nothing to rotate
                if (std::abs(gamma) <= 1e-15 * std::sqrt(alpha * beta) || gamma == 0.0) continue;
                off = std::max(off, std::abs(gamma) / std::sqrt(alpha * beta));
                const double zeta = (beta - alpha) / (2.0 * gamma);
                const double t = (zeta >= 0 ? 1.0 : -1.0) / (std::abs(zeta) + std::sqrt(1.0 + zeta * zeta));
                const double c = 1.0 / std::sqrt(1.0 + t * t), s = c * t;
                for (int i = 0; i < m; ++i) {
                    const double up = U(i, p), uq = U(i, q);
                    U(i, p) = c * up - s * uq;
                    U(i, q) = s * up + c * uq;
                }
                for (int i = 0; i < n; ++i) {
                    const double vp = V(i, p), vq = V(i, q);
                    V(i, p) = c * vp - s * vq;
                    V(i, q) = s * vp + c * vq;
                }
            }
        }
        if (off < 1e-14) break;
    }
    w.assign(n, 0.0);
    for (int j = 0; j < n; ++j) {
        double norm = 0.0;
        for (int i = 0; i < m; ++i) norm += U(i, j) * U(i, j);
        norm = std::sqrt(norm);
        w[j] = norm;
        if (norm > kEps)
            for (int i = 0; i < m; ++i) U(i, j) /= norm;
    }
}

// Blender IK_QJacobian::InvertSDLS (selectively damped least squares). Rows come in blocks of 3.
std::vector<double> invertSDLS(const Mat& J, const std::vector<double>& beta, const std::vector<double>& weight,
                               const Mat& U, const std::vector<double>& w, const Mat& V) {
    const int m = J.rows, n = J.cols;
    const double maxAngleChange = M_PI / 4.0;
    std::vector<double> dtheta(n, 0.0), tmp(n, 0.0), norm(n, 0.0);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j + 2 < m; j += 3) {
            const double v = J(j, i) * J(j, i) + J(j + 1, i) * J(j + 1, i) + J(j + 2, i) * J(j + 2, i);
            norm[i] += std::sqrt(v);
        }
    for (int i = 0; i < n; ++i) {
        if (w[i] <= kEps) continue;
        const double wInv = 1.0 / w[i];
        double alpha = 0.0, N = 0.0;
        for (int j = 0; j + 2 < m; j += 3) {
            alpha += U(j, i) * beta[j] + U(j + 1, i) * beta[j + 1] + U(j + 2, i) * beta[j + 2];
            N += std::sqrt(U(j, i) * U(j, i) + U(j + 1, i) * U(j + 1, i) + U(j + 2, i) * U(j + 2, i));
        }
        alpha *= wInv;
        double M = 0.0, maxDtheta = 0.0;
        for (int j = 0; j < n; ++j) {
            const double v = V(j, i);
            M += std::abs(v) * norm[j];
            tmp[j] = v * alpha;
            maxDtheta = std::max(maxDtheta, std::abs(tmp[j]) * std::sqrt(weight[j]));
        }
        M *= wInv;
        double gamma = maxAngleChange;
        if (N < M) gamma *= N / M;
        const double damp = gamma < maxDtheta ? gamma / maxDtheta : 1.0;
        for (int j = 0; j < n; ++j) dtheta[j] += 0.80 * std::min(damp / weight[j], 1.0) * tmp[j];
    }
    double maxAngle = 0.0;
    for (int j = 0; j < n; ++j) {
        dtheta[j] *= weight[j];
        maxAngle = std::max(maxAngle, std::abs(dtheta[j]));
    }
    if (maxAngle > maxAngleChange) {
        const double damp = maxAngleChange / (maxAngleChange + maxAngle);
        for (auto& d : dtheta) d *= damp;
    }
    return dtheta;
}

// Blender IK_QJacobian::InvertDLS (damping only near singularities).
std::vector<double> invertDLS(const std::vector<double>& beta, const std::vector<double>& weight, const Mat& U,
                              const std::vector<double>& w, const Mat& V) {
    const int m = U.rows, n = V.rows;
    const double maxAngleChange = 0.1;
    double xLength = 0.0;
    for (double b : beta) xLength += b * b;
    xLength = std::sqrt(xLength);
    double wMin = std::numeric_limits<double>::max();
    for (double s : w)
        if (s > kEps && s < wMin) wMin = s;
    const double d = xLength / maxAngleChange;
    double lambda = 0.0;
    if (wMin <= d / 2) lambda = d / 2;
    else if (wMin < d) lambda = std::sqrt(wMin * (d - wMin));
    lambda = std::min(lambda * lambda, 10.0);

    std::vector<double> dtheta(n, 0.0);
    for (int i = 0; i < n; ++i) {
        if (w[i] <= kEps) continue;
        double ub = 0.0;
        for (int j = 0; j < m; ++j) ub += U(j, i) * beta[j];
        ub *= w[i] / (w[i] * w[i] + lambda);
        for (int j = 0; j < n; ++j) dtheta[j] += V(j, i) * ub;
    }
    for (int j = 0; j < n; ++j) dtheta[j] *= weight[j];
    return dtheta;
}

glm::vec3 rotationVectorBetween(const glm::vec3& from, const glm::vec3& to) {
    const glm::vec3 c = glm::cross(from, to);
    const float s = glm::length(c), co = glm::dot(from, to);
    const float ang = std::atan2(s, co);
    if (s > 1e-7f) return c / s * ang;
    if (co > 0.0f) return glm::vec3(0.0f);
    glm::vec3 perp = std::abs(from.x) < 0.9f ? glm::cross(from, glm::vec3(1, 0, 0)) : glm::cross(from, glm::vec3(0, 1, 0));
    return glm::normalize(perp) * float(M_PI);
}

} // namespace

// =============================================================================================

bool IkSolver::setup(const Robot& robot) {
    dofJoints_.clear();
    chain_.clear();
    minRad_.clear();
    maxRad_.clear();
    const int tcp = robot.tcpPart >= 0 ? robot.tcpPart : -1;
    if (tcp < 0) return false;
    std::vector<int> parts;
    for (int p = tcp; p >= 0; p = robot.parts[p].parent) parts.push_back(p);
    std::reverse(parts.begin(), parts.end());
    auto dofOf = [&](int driver) {
        auto it = std::find(dofJoints_.begin(), dofJoints_.end(), driver);
        if (it != dofJoints_.end()) return int(it - dofJoints_.begin());
        dofJoints_.push_back(driver);
        const RobotJoint& J = robot.joints[driver];
        minRad_.push_back(glm::radians(J.minDeg));
        maxRad_.push_back(glm::radians(J.maxDeg));
        return int(dofJoints_.size() - 1);
    };
    for (int p : parts) {
        const int j = robot.parts[p].joint;
        if (j < 0) continue;
        const RobotJoint& J = robot.joints[j];
        ChainJoint cj;
        cj.origin = J.origin;
        cj.axis = glm::length(J.axis) > 0.0f ? glm::normalize(J.axis) : glm::vec3(0, 0, 1);
        if (J.type == JointType::Revolute) {
            if (J.mimic >= 0) {
                cj.dof = dofOf(J.mimic);
                cj.multiplier = J.multiplier;
                cj.offsetDeg = J.offsetDeg;
            } else {
                cj.dof = dofOf(j);
            }
        }
        chain_.push_back(cj);
    }
    return !dofJoints_.empty();
}

std::vector<float> IkSolver::currentAngles(const Robot& robot) const {
    std::vector<float> a;
    for (int j : dofJoints_) a.push_back(robot.joints[j].angleDeg);
    return a;
}

void IkSolver::applyAngles(Robot& robot, const std::vector<float>& anglesDeg) const {
    for (size_t i = 0; i < dofJoints_.size() && i < anglesDeg.size(); ++i) robot.joints[dofJoints_[i]].angleDeg = anglesDeg[i];
}

glm::mat4 IkSolver::chainFK(const Robot& robot, const std::vector<double>& rad, std::vector<glm::vec3>* jointPos,
                            std::vector<glm::vec3>* jointAxis) const {
    glm::mat4 L(1.0f);
    const glm::mat4 W = robot.modelToWorld();
    if (jointPos) jointPos->resize(chain_.size());
    if (jointAxis) jointAxis->resize(chain_.size());
    for (size_t k = 0; k < chain_.size(); ++k) {
        const ChainJoint& cj = chain_[k];
        const glm::mat4 pre = W * L;
        if (jointPos) (*jointPos)[k] = glm::vec3(pre * glm::vec4(cj.origin, 1.0f));
        if (jointAxis) (*jointAxis)[k] = glm::normalize(glm::mat3(pre) * cj.axis);
        if (cj.dof < 0) continue;
        const float a = float(cj.multiplier * rad[cj.dof]) + glm::radians(cj.offsetDeg);
        if (a != 0.0f)
            L = L * glm::translate(glm::mat4(1.0f), cj.origin) * glm::rotate(glm::mat4(1.0f), a, cj.axis) *
                glm::translate(glm::mat4(1.0f), -cj.origin);
    }
    return W * L;
}

glm::mat4 IkSolver::toolFrame(const Robot& robot, const std::vector<float>& anglesDeg, const IkTool& tool) const {
    std::vector<double> rad(dofJoints_.size(), 0.0);
    for (size_t i = 0; i < rad.size() && i < anglesDeg.size(); ++i) rad[i] = glm::radians(double(anglesDeg[i]));
    const glm::mat4 P = chainFK(robot, rad, nullptr, nullptr);
    glm::mat4 F = P * glm::inverse(robot.modelToMount()); // same convention as Robot::tcpWorld()
    const glm::vec3 tcp = glm::vec3(P * glm::vec4(robot.tcpPoint, 1.0f));
    F[3] = glm::vec4(tcp + glm::mat3(F) * tool.tipOffset, 1.0f);
    return F;
}

IkResult IkSolver::solve(const Robot& robot, const std::vector<float>& startDeg, const std::vector<IkTask>& tasks,
                         const IkTool& tool, int maxIterations) const {
    IkResult res;
    const int n = dofCount();
    if (n == 0) return res;
    std::vector<double> theta(n, 0.0);
    for (int i = 0; i < n && i < int(startDeg.size()); ++i)
        theta[i] = std::clamp(double(glm::radians(startDeg[i])), double(minRad_[i]), double(maxRad_[i]));

    // ---- scale (Blender: ComputeScale) and step clamp (IK_QPositionTask::m_clamp_length)
    std::vector<glm::vec3> jp, ja;
    {
        const std::vector<double> zero(n, 0.0);
        const glm::mat4 F = chainFK(robot, zero, &jp, &ja);
        (void)F;
    }
    const glm::vec3 tip0 = glm::vec3(toolFrame(robot, std::vector<float>(n, 0.0f), tool)[3]);
    double reach = 0.0;
    int segments = 0;
    for (size_t k = 1; k < jp.size(); ++k, ++segments) reach += glm::length(jp[k] - jp[k - 1]);
    if (!jp.empty()) {
        reach += glm::length(tip0 - jp.back());
        ++segments;
    }
    const double scale = reach > 1e-6 ? 1.0 / reach : 1.0;
    const double clampLength = segments > 0 ? reach * scale / (2.0 * segments) : 0.1;

    // ---- task layout and weights (primary weights sum to 1)
    double primaryW = 0.0;
    int primaryRows = 0;
    for (const auto& t : tasks)
        if (t.primary) {
            primaryW += t.weight;
            primaryRows += 3;
        }
    if (primaryRows == 0 || primaryW <= 0.0) return res;
    int secondaryRows = 0;
    for (const auto& t : tasks)
        if (!t.primary) secondaryRows += 3;
    const bool secondary = !restPoseDeg.empty() && int(restPoseDeg.size()) == n && restWeight > 0.0f;
    std::vector<double> dofWeight(n, 1.0);

    auto evaluate = [&](const std::vector<double>& th, glm::mat4& frame) {
        const glm::mat4 P = chainFK(robot, th, &jp, &ja);
        frame = P * glm::inverse(robot.modelToMount());
        const glm::vec3 tcp = glm::vec3(P * glm::vec4(robot.tcpPoint, 1.0f));
        frame[3] = glm::vec4(tcp + glm::mat3(frame) * tool.tipOffset, 1.0f);
    };

    for (int iter = 0; iter < maxIterations; ++iter) {
        res.iterations = iter + 1;
        glm::mat4 F;
        evaluate(theta, F);
        const glm::vec3 tip = glm::vec3(F[3]);
        const glm::vec3 zAxis = glm::normalize(glm::vec3(F[2]));
        const glm::mat3 R = glm::mat3(F);

        // ---- Jacobian + betas (IK_QTask::ComputeJacobian)
        auto fillTask = [&](const IkTask& t, double w, Mat& Jm, std::vector<double>& bm, int row) {
            glm::dvec3 b(0.0);
            if (t.type == IkTask::Position) {
                b = glm::dvec3(t.goal - tip) * scale;
                const double len = glm::length(b);
                if (len > clampLength) b *= clampLength / len;
            } else if (t.type == IkTask::Direction) {
                b = glm::dvec3(rotationVectorBetween(zAxis, glm::normalize(t.goal)));
            } else {
                const glm::mat3 d = glm::transpose(t.goalRotation * glm::transpose(R));
                b = -0.5 * glm::dvec3(d[1][2] - d[2][1], d[2][0] - d[0][2], d[0][1] - d[1][0]);
            }
            for (int c = 0; c < 3; ++c) bm[row + c] = w * b[c];
            for (size_t k = 0; k < chain_.size(); ++k) {
                const ChainJoint& cj = chain_[k];
                if (cj.dof < 0) continue;
                glm::dvec3 col;
                const glm::dvec3 a = glm::dvec3(ja[k]);
                if (t.type == IkTask::Position) col = glm::cross(a, glm::dvec3(tip - jp[k]) * scale);
                else if (t.type == IkTask::Direction) col = a - glm::dot(a, glm::dvec3(zAxis)) * glm::dvec3(zAxis);
                else col = a;
                for (int c = 0; c < 3; ++c) Jm(row + c, cj.dof) += w * cj.multiplier * col[c];
            }
        };
        Mat J(primaryRows, n);
        std::vector<double> beta(primaryRows, 0.0);
        int row = 0;
        for (const auto& t : tasks) {
            if (!t.primary) continue;
            fillTask(t, t.weight / primaryW, J, beta, row);
            row += 3;
        }
        // secondary tasks (e.g. "aim as close as possible to this direction"): solved in the null
        // space of the primary tasks, so they never disturb them (IK_QJacobian::SubTask)
        Mat J2s(std::max(secondaryRows, 1), n);
        std::vector<double> beta2s(std::max(secondaryRows, 1), 0.0);
        row = 0;
        for (const auto& t : tasks) {
            if (t.primary) continue;
            fillTask(t, t.weight, J2s, beta2s, row);
            row += 3;
        }
        for (int r = 0; r < secondaryRows; ++r) beta2s[r] = std::clamp(beta2s[r], -0.2, 0.2);

        // ---- invert, clamp to limits and lock violating DoFs (IK_QJacobianSolver::UpdateAngles)
        std::vector<char> locked(n, 0);
        std::vector<double> lockedDelta(n, 0.0);
        std::vector<double> dtheta;
        Mat U, V;
        std::vector<double> sw;
        for (int inner = 0; inner <= n; ++inner) {
            svd(J, U, sw, V);
            dtheta = method == Method::SDLS ? invertSDLS(J, beta, dofWeight, U, sw, V) : invertDLS(beta, dofWeight, U, sw, V);

            if (secondaryRows > 0) {
                Mat Nul(n, n);
                for (int i = 0; i < n; ++i) Nul(i, i) = locked[i] ? 0.0 : 1.0;
                for (int c = 0; c < n; ++c) {
                    if (sw[c] <= kEps) continue;
                    for (int i = 0; i < n; ++i)
                        for (int j = 0; j < n; ++j) Nul(i, j) -= V(i, c) * V(j, c);
                }
                Mat A(secondaryRows, n);
                std::vector<double> e(secondaryRows, 0.0);
                for (int r = 0; r < secondaryRows; ++r) {
                    double jd = 0.0;
                    for (int j = 0; j < n; ++j) {
                        jd += J2s(r, j) * dtheta[j];
                        double acc = 0.0;
                        for (int k = 0; k < n; ++k) acc += J2s(r, k) * Nul(k, j);
                        A(r, j) = acc;
                    }
                    e[r] = beta2s[r] - jd;
                }
                Mat U2, V2;
                std::vector<double> w2;
                svd(A, U2, w2, V2);
                const std::vector<double> d2 = invertDLS(e, dofWeight, U2, w2, V2);
                for (int i = 0; i < n; ++i) {
                    double acc = 0.0;
                    for (int j = 0; j < n; ++j) acc += Nul(i, j) * d2[j];
                    dtheta[i] += acc;
                }
            }
            // rest pose preference in the null space of the primary one (IK_QJacobian::SubTask)
            if (secondary && secondaryRows == 0) {
                int rank = 0;
                for (double s : sw) rank += s > kEps;
                if (rank >= std::min(primaryRows, n) && rank < n) {
                    Mat Nul(n, n);
                    for (int i = 0; i < n; ++i) Nul(i, i) = 1.0;
                    for (int c = 0; c < n; ++c) {
                        if (sw[c] <= kEps) continue;
                        for (int i = 0; i < n; ++i)
                            for (int j = 0; j < n; ++j) Nul(i, j) -= V(i, c) * V(j, c);
                    }
                    Mat J2(n, n);
                    std::vector<double> b2(n, 0.0);
                    for (int i = 0; i < n; ++i) {
                        if (locked[i]) continue;
                        b2[i] = restWeight * (glm::radians(double(restPoseDeg[i])) - theta[i]) - restWeight * dtheta[i];
                        for (int j = 0; j < n; ++j) J2(i, j) = restWeight * Nul(i, j);
                    }
                    Mat U2, V2;
                    std::vector<double> w2;
                    svd(J2, U2, w2, V2);
                    const std::vector<double> d2 = invertDLS(b2, dofWeight, U2, w2, V2);
                    for (int i = 0; i < n; ++i) dtheta[i] += d2[i];
                }
            }
            for (int i = 0; i < n; ++i)
                if (locked[i]) dtheta[i] = lockedDelta[i];

            int minDof = -1;
            double minAbs = 1e10;
            bool lockedNow = false;
            for (int i = 0; i < n; ++i) {
                if (locked[i]) continue;
                const double nw = theta[i] + dtheta[i];
                double delta;
                if (nw > maxRad_[i]) delta = maxRad_[i] - theta[i];
                else if (nw < minRad_[i]) delta = minRad_[i] - theta[i];
                else continue;
                if (std::abs(delta) < 1e-12) {
                    locked[i] = 1;
                    lockedDelta[i] = delta;
                    lockedNow = true;
                } else if (std::abs(delta) < minAbs) {
                    minAbs = std::abs(delta);
                    minDof = i;
                }
                (void)delta;
            }
            if (minDof >= 0) {
                const double nw = theta[minDof] + dtheta[minDof];
                locked[minDof] = 1;
                lockedDelta[minDof] = (nw > maxRad_[minDof] ? maxRad_[minDof] : minRad_[minDof]) - theta[minDof];
                lockedNow = true;
            }
            if (!lockedNow) break;
            // IK_QJacobian::Lock: move the locked DoF to its limit, remove its column
            for (int i = 0; i < n; ++i) {
                if (!locked[i]) continue;
                for (int r = 0; r < primaryRows; ++r) {
                    if (J(r, i) == 0.0) continue;
                    beta[r] -= J(r, i) * lockedDelta[i];
                    J(r, i) = 0.0;
                }
            }
        }
        double norm = 0.0;
        for (int i = 0; i < n; ++i) {
            theta[i] = std::clamp(theta[i] + dtheta[i], double(minRad_[i]), double(maxRad_[i]));
            norm = std::max(norm, std::abs(dtheta[i]));
        }
        if (norm < 1e-6 && iter > 3) break;
    }

    // ---- final errors
    glm::mat4 F;
    evaluate(theta, F);
    const glm::vec3 tip = glm::vec3(F[3]);
    const glm::vec3 zAxis = glm::normalize(glm::vec3(F[2]));
    res.positionError = 0.0f;
    res.directionError = 0.0f;
    for (const auto& t : tasks) {
        if (!t.primary) {
            if (t.type == IkTask::Direction)
                res.secondaryError = std::max(res.secondaryError,
                                              glm::degrees(std::acos(std::clamp(glm::dot(zAxis, glm::normalize(t.goal)), -1.0f, 1.0f))));
            continue;
        }
        if (t.type == IkTask::Position) res.positionError = std::max(res.positionError, glm::length(t.goal - tip));
        else if (t.type == IkTask::Direction)
            res.directionError = std::max(res.directionError,
                                          glm::degrees(std::acos(std::clamp(glm::dot(zAxis, glm::normalize(t.goal)), -1.0f, 1.0f))));
        else {
            const glm::mat3 d = t.goalRotation * glm::transpose(glm::mat3(F));
            const float tr = std::clamp((d[0][0] + d[1][1] + d[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
            res.directionError = std::max(res.directionError, glm::degrees(std::acos(tr)));
        }
    }
    res.converged = res.positionError < 0.05f && res.directionError < 0.25f;
    res.angles.resize(n);
    for (int i = 0; i < n; ++i) res.angles[i] = float(glm::degrees(theta[i]));
    return res;
}

} // namespace pegasus
