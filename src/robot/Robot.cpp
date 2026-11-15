#include "robot/Robot.h"

#include "geom/Collision.h"
#include "scene/StlLoader.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace pegasus {

namespace fs = std::filesystem;

// ---------------------------------------------------------------------------------------------
// Structure
// ---------------------------------------------------------------------------------------------

int Robot::root() const {
    for (int i = 0; i < int(parts.size()); ++i)
        if (parts[i].parent < 0) return i;
    return -1;
}

std::vector<int> Robot::topoOrder() const {
    std::vector<int> order;
    std::vector<char> done(parts.size(), 0);
    order.reserve(parts.size());
    // repeated passes are fine for a few dozen parts
    bool progress = true;
    while (order.size() < parts.size() && progress) {
        progress = false;
        for (int i = 0; i < int(parts.size()); ++i) {
            if (done[i]) continue;
            const int p = parts[i].parent;
            if (p < 0 || done[p]) {
                done[i] = 1;
                order.push_back(i);
                progress = true;
            }
        }
    }
    return order; // parts in a cycle (invalid) are left out
}

int Robot::depth(int part) const {
    int d = 0;
    for (int p = parts[part].parent; p >= 0 && d < 1000; p = parts[p].parent) ++d;
    return d;
}

bool Robot::isAncestor(int ancestor, int part) const {
    for (int p = parts[part].parent, guard = 0; p >= 0 && guard < 1000; p = parts[p].parent, ++guard)
        if (p == ancestor) return true;
    return false;
}

std::vector<int> Robot::drivers() const {
    std::vector<int> d;
    for (int j = 0; j < int(joints.size()); ++j)
        if (joints[j].isDriver() && joints[j].part >= 0) d.push_back(j);
    std::stable_sort(d.begin(), d.end(), [&](int a, int b) { return depth(joints[a].part) < depth(joints[b].part); });
    return d;
}

std::string Robot::validate() const {
    if (parts.empty()) return "Robotta parça yok.";
    int roots = 0;
    for (const auto& p : parts) roots += p.parent < 0;
    if (roots != 1) return "Robotun tam olarak bir kök (sabit taban) parçası olmalı.";
    if (topoOrder().size() != parts.size()) return "Parça hiyerarşisinde döngü var.";
    for (int j = 0; j < int(joints.size()); ++j) {
        const RobotJoint& jt = joints[j];
        if (jt.mimic >= 0 && (jt.mimic >= int(joints.size()) || !joints[jt.mimic].isDriver()))
            return "Bağımlı eklem '" + jt.name + "' bağımsız bir eklemi takip etmeli.";
    }
    return {};
}

// ---------------------------------------------------------------------------------------------
// Kinematics
// ---------------------------------------------------------------------------------------------

float Robot::effectiveAngle(int j) const {
    const RobotJoint& jt = joints[j];
    if (jt.type == JointType::Fixed) return 0.0f;
    if (jt.mimic >= 0 && jt.mimic < int(joints.size())) return jt.multiplier * joints[jt.mimic].angleDeg + jt.offsetDeg;
    return jt.angleDeg;
}

void Robot::setDriverAngle(int j, float deg) {
    if (j < 0 || j >= int(joints.size())) return;
    joints[j].angleDeg = std::clamp(deg, joints[j].minDeg, joints[j].maxDeg) + 0.0f;
}

void Robot::resetPose() {
    for (auto& j : joints) j.angleDeg = std::clamp(0.0f, j.minDeg, j.maxDeg);
}

void Robot::solveFK() {
    model_.assign(parts.size(), glm::mat4(1.0f));
    for (int p : topoOrder()) {
        const RobotPart& part = parts[p];
        if (part.parent < 0) continue;
        glm::mat4 local(1.0f);
        if (part.joint >= 0) {
            const RobotJoint& j = joints[part.joint];
            const float a = effectiveAngle(part.joint);
            if (j.type == JointType::Revolute && a != 0.0f && glm::length(j.axis) > 0.0f) {
                local = glm::translate(glm::mat4(1.0f), j.origin) *
                        glm::rotate(glm::mat4(1.0f), glm::radians(a), glm::normalize(j.axis)) *
                        glm::translate(glm::mat4(1.0f), -j.origin);
            }
        }
        model_[p] = model_[part.parent] * local;
    }
}

glm::mat4 Robot::modelToMount() const {
    const int r = root();
    glm::vec3 c(0.0f);
    if (r >= 0 && parts[r].mesh) {
        const AABB& b = parts[r].mesh->bounds;
        c = b.center();
        c[upAxis] = b.min[upAxis];
    }
    glm::mat4 rot(1.0f);
    if (upAxis == 1) rot = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1, 0, 0));       // Y -> Z
    else if (upAxis == 0) rot = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(0, 1, 0)); // X -> Z
    return rot * glm::translate(glm::mat4(1.0f), -c);
}

glm::vec3 Robot::jointOriginWorld(int j) const {
    const int parent = parts[joints[j].part].parent;
    const glm::mat4 m = modelToWorld() * (parent >= 0 ? model_[parent] : glm::mat4(1.0f));
    return glm::vec3(m * glm::vec4(joints[j].origin, 1.0f));
}

glm::vec3 Robot::jointAxisWorld(int j) const {
    const int parent = parts[joints[j].part].parent;
    const glm::mat4 m = modelToWorld() * (parent >= 0 ? model_[parent] : glm::mat4(1.0f));
    const glm::vec3 a = glm::mat3(m) * joints[j].axis;
    return glm::length(a) > 0.0f ? glm::normalize(a) : glm::vec3(0, 0, 1);
}

glm::mat4 Robot::tcpWorld() const {
    const int p = tcpPart >= 0 && tcpPart < int(parts.size()) ? tcpPart : root();
    const glm::mat4 pw = p >= 0 ? partWorld(p) : modelToWorld();
    // Orientation is expressed so that it equals the base (mount) frame in the home pose:
    // tool Z = approach direction of an upright arm.
    glm::mat4 m = pw * glm::inverse(modelToMount());
    m[3] = pw * glm::vec4(tcpPoint, 1.0f);
    return m;
}

glm::vec3 Robot::worldToPartModel(int part, const glm::vec3& p) const {
    const glm::mat4 m = modelToWorld() * (part >= 0 ? model_[part] : glm::mat4(1.0f));
    return glm::vec3(glm::inverse(m) * glm::vec4(p, 1.0f));
}

glm::vec3 Robot::worldDirToPartModel(int part, const glm::vec3& d) const {
    const glm::mat4 m = modelToWorld() * (part >= 0 ? model_[part] : glm::mat4(1.0f));
    return glm::normalize(glm::inverse(glm::mat3(m)) * d);
}

void Robot::flipDriverDirection(int j) {
    RobotJoint& d = joints[j];
    if (!d.isDriver()) return;
    d.axis = -d.axis;
    d.angleDeg = -d.angleDeg + 0.0f; // avoid -0
    std::swap(d.minDeg, d.maxDeg);
    d.minDeg = -d.minDeg;
    d.maxDeg = -d.maxDeg;
    for (auto& m : joints)
        if (m.mimic == j) m.multiplier = -m.multiplier;
}

// ---------------------------------------------------------------------------------------------
// Collision helpers
// ---------------------------------------------------------------------------------------------

bool Robot::ignores(int a, int b) const {
    if (a > b) std::swap(a, b);
    return std::find(ignoredPairs.begin(), ignoredPairs.end(), std::make_pair(a, b)) != ignoredPairs.end();
}

void Robot::computeIgnoredPairs() {
    std::vector<float> saved;
    for (auto& j : joints) {
        saved.push_back(j.angleDeg);
        j.angleDeg = 0.0f;
    }
    solveFK();
    auto group = [&](int p) {
        const int j = parts[p].joint;
        if (j < 0 || joints[j].type != JointType::Revolute) return -1;
        return joints[j].mimic >= 0 ? joints[j].mimic : j;
    };
    ignoredPairs.clear();
    for (int a = 0; a < int(parts.size()); ++a) {
        for (int b = a + 1; b < int(parts.size()); ++b) {
            const glm::mat4 ma = model_[a], mb = model_[b];
            if (!parts[a].mesh->bounds.transformed(ma).overlaps(parts[b].mesh->bounds.transformed(mb))) continue;
            // Already interpenetrating in the assembly (press fits, servo horns, ...): never report.
            bool ignore = CollisionSystem::testPair(*parts[a].mesh, ma, *parts[b].mesh, mb, nullptr, 0, nullptr, nullptr);
            // Parts of the same mechanism that touch (gear teeth, linkage pins) move together.
            if (!ignore && group(a) >= 0 && group(a) == group(b))
                ignore = CollisionSystem::touching(*parts[a].mesh, ma, *parts[b].mesh, mb, 0.3f);
            if (ignore) ignoredPairs.emplace_back(a, b);
        }
    }
    for (size_t i = 0; i < joints.size(); ++i) joints[i].angleDeg = saved[i];
    solveFK();
}

bool Robot::selfCollides() const {
    const size_t n = parts.size();
    std::vector<AABB> boxes(n);
    for (size_t i = 0; i < n; ++i) boxes[i] = parts[i].mesh->bounds.transformed(model_[i]);
    for (size_t a = 0; a < n; ++a)
        for (size_t b = a + 1; b < n; ++b) {
            if (!boxes[a].overlaps(boxes[b]) || ignores(int(a), int(b))) continue;
            if (CollisionSystem::testPair(*parts[a].mesh, model_[a], *parts[b].mesh, model_[b], nullptr, 0, nullptr, nullptr))
                return true;
        }
    return false;
}

bool Robot::belowGround(float tolerance) const {
    const glm::mat4 toWorld = modelToMount(); // ground = mount plane, independent of the placement
    const int r = root();
    for (int i = 0; i < int(parts.size()); ++i) {
        if (i == r) continue;
        const glm::mat4 m = toWorld * model_[i];
        if (parts[i].mesh->bounds.transformed(m).min.z >= -tolerance) continue;
        for (const auto& p : parts[i].mesh->positions)
            if ((m * glm::vec4(p, 1.0f)).z < -tolerance) return true;
    }
    return false;
}

void Robot::autoLimits(int driver, float rangeDeg, float stepDeg) {
    if (driver < 0 || !joints[driver].isDriver()) return;
    const float saved = joints[driver].angleDeg;
    float limits[2] = {0.0f, 0.0f};
    for (int dir = 0; dir < 2; ++dir) {
        const float sign = dir == 0 ? -1.0f : 1.0f;
        float lastOk = 0.0f;
        for (float a = stepDeg; a <= rangeDeg + 1e-3f; a += stepDeg) {
            joints[driver].angleDeg = sign * a;
            solveFK();
            if (selfCollides() || belowGround()) break;
            lastOk = a;
        }
        limits[dir] = sign * lastOk;
    }
    joints[driver].minDeg = limits[0];
    joints[driver].maxDeg = limits[1];
    joints[driver].angleDeg = std::clamp(saved, limits[0], limits[1]);
    solveFK();
}

// ---------------------------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------------------------

namespace {

std::string quote(const std::string& s) {
    std::string out = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out + "\"";
}

std::vector<std::string> tokenize(const std::string& line) {
    std::vector<std::string> t;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if (i >= line.size() || line[i] == '#') break;
        std::string tok;
        if (line[i] == '"') {
            ++i;
            while (i < line.size() && line[i] != '"') {
                if (line[i] == '\\' && i + 1 < line.size()) ++i;
                tok += line[i++];
            }
            ++i;
        } else {
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) tok += line[i++];
        }
        t.push_back(tok);
    }
    return t;
}

} // namespace

bool Robot::save(const std::string& path, std::string& err) const {
    std::ofstream f(path);
    if (!f) {
        err = "Dosya yazılamadı: " + path;
        return false;
    }
    f.precision(9);
    f << "# Pegasus robot tanimi. Koordinatlar STL (montaj) koordinatlarinda, mm ve derece.\n";
    f << "pegasus_robot 1\n";
    f << "name " << quote(name) << "\n";
    f << "up_axis " << upAxis << "\n";
    f << "mount";
    const float* m = glm::value_ptr(mount);
    for (int i = 0; i < 16; ++i) f << " " << m[i];
    f << "\n";
    f << "tcp " << tcpPart << " " << tcpPoint.x << " " << tcpPoint.y << " " << tcpPoint.z << "\n";
    f << "# part <dosya> <ad> <ust parca indeksi>\n";
    for (const auto& p : parts) f << "part " << quote(p.file) << " " << quote(p.name) << " " << p.parent << "\n";
    f << "# joint <parca> <ad> <tip 0=sabit 1=doner> <orijin xyz> <eksen xyz> <min> <max> <aci> <takip edilen eklem> <carpan> <ofset> <motorlu>\n";
    for (const auto& j : joints) {
        f << "joint " << j.part << " " << quote(j.name) << " " << int(j.type) << " " << j.origin.x << " " << j.origin.y << " "
          << j.origin.z << " " << j.axis.x << " " << j.axis.y << " " << j.axis.z << " " << j.minDeg << " " << j.maxDeg << " "
          << j.angleDeg << " " << j.mimic << " " << j.multiplier << " " << j.offsetDeg << " " << int(j.motorized) << "\n";
    }
    return bool(f);
}

std::unique_ptr<Robot> Robot::load(const std::string& path, const std::string& partsFolder, std::string& err) {
    std::ifstream f(path);
    if (!f) {
        err = "Robot dosyası açılamadı: " + path;
        return nullptr;
    }
    auto robot = std::make_unique<Robot>();
    robot->folder = partsFolder.empty() ? fs::path(path).parent_path().string() : partsFolder;
    std::string line;
    int lineNo = 0;
    bool header = false;
    try {
        while (std::getline(f, line)) {
            ++lineNo;
            const auto t = tokenize(line);
            if (t.empty()) continue;
            const std::string& k = t[0];
            auto num = [&](size_t i) { return std::stof(t.at(i)); };
            if (k == "pegasus_robot") header = true;
            else if (k == "name") robot->name = t.at(1);
            else if (k == "up_axis") robot->upAxis = std::clamp(std::stoi(t.at(1)), 0, 2);
            else if (k == "mount") {
                float* m = glm::value_ptr(robot->mount);
                for (int i = 0; i < 16; ++i) m[i] = num(1 + i);
            } else if (k == "tcp") {
                robot->tcpPart = std::stoi(t.at(1));
                robot->tcpPoint = glm::vec3(num(2), num(3), num(4));
            } else if (k == "part") {
                RobotPart p;
                p.file = t.at(1);
                p.name = t.at(2);
                p.parent = std::stoi(t.at(3));
                robot->parts.push_back(p);
            } else if (k == "joint") {
                RobotJoint j;
                j.part = std::stoi(t.at(1));
                j.name = t.at(2);
                j.type = std::stoi(t.at(3)) == 0 ? JointType::Fixed : JointType::Revolute;
                j.origin = glm::vec3(num(4), num(5), num(6));
                j.axis = glm::vec3(num(7), num(8), num(9));
                j.minDeg = num(10);
                j.maxDeg = num(11);
                j.angleDeg = num(12);
                j.mimic = std::stoi(t.at(13));
                j.multiplier = num(14);
                j.offsetDeg = num(15);
                j.motorized = t.size() > 16 && t[16] == "1";
                robot->joints.push_back(j);
            }
        }
    } catch (const std::exception&) {
        err = "Robot dosyası okunamadı (satır " + std::to_string(lineNo) + ").";
        return nullptr;
    }
    if (!header) {
        err = "Geçerli bir Pegasus robot dosyası değil.";
        return nullptr;
    }
    const int np = int(robot->parts.size());
    for (int i = 0; i < np; ++i)
        if (robot->parts[i].parent >= np) {
            err = "Robot dosyasında geçersiz parça indeksi.";
            return nullptr;
        }
    for (int j = 0; j < int(robot->joints.size()); ++j) {
        const int p = robot->joints[j].part;
        if (p < 0 || p >= np) {
            err = "Robot dosyasında geçersiz eklem.";
            return nullptr;
        }
        robot->parts[p].joint = j;
    }
    if (robot->tcpPart >= np) robot->tcpPart = -1;
    for (auto& p : robot->parts) {
        std::string e;
        p.mesh = loadStl((fs::path(robot->folder) / p.file).string(), e);
        if (!p.mesh) {
            err = e;
            return nullptr;
        }
    }
    if (std::string v = robot->validate(); !v.empty()) {
        err = v;
        return nullptr;
    }
    robot->solveFK();
    return robot;
}

} // namespace pegasus
