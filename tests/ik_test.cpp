// Robot model (saved + auto-detected) and the Blender-style IK solver.
#include "check.h"

#include "robot/IkSolver.h"
#include "robot/JointDetect.h"
#include "robot/Robot.h"

#include <chrono>
#include <cmath>
#include <random>

using namespace pegasus;

int main() {
    const std::string folder = PEGASUS_ROBOT_DIR;
    std::string err;

    section("1) robot modeli");
    auto robot = Robot::load(folder + "/robot.pegasus", folder, err);
    CHECK(robot != nullptr, "robot.pegasus yuklendi " + err);
    if (!robot) return pegasus_test::finish("ik_test");
    const std::vector<int> drivers = robot->drivers();
    const char* names[] = {"Bel", "Omuz", "Dirsek", "Bilek dönüş", "Bilek eğim", "Kıskaç"};
    bool namesOk = drivers.size() == 6;
    for (size_t i = 0; namesOk && i < 6; ++i) namesOk = robot->joints[size_t(drivers[i])].name == names[i];
    CHECK(namesOk, "6 bagimsiz eklem, tabandan uca: Bel .. Kiskac");
    std::vector<std::string> log;
    auto detected = detectRobot(folder, err, log);
    CHECK(detected && detected->drivers().size() == 6, "STL'lerden otomatik algilama da 6 bagimsiz eklem buluyor");

    IkSolver ik;
    CHECK(ik.setup(*robot) && ik.dofCount() == 5, "IK zinciri: 5 serbestlik derecesi (kiskac haric)");

    section("2) ileri kinematik tutarliligi");
    {
        const std::vector<float> a = {20, 30, -40, 15, 25};
        ik.applyAngles(*robot, a);
        robot->solveFK();
        const glm::mat4 A = robot->tcpWorld(), B = ik.toolFrame(*robot, a, IkTool{});
        CHECK(glm::length(glm::vec3(A[3] - B[3])) < 1e-3f, "IK zincirinin FK'si robotun FK'siyle ayni");
        ik.applyAngles(*robot, std::vector<float>(5, 0.0f));
        robot->solveFK();
    }

    section("3) konum + takim yonu: 300 rastgele poz");
    for (int method = 0; method < 2; ++method) {
        ik.method = method == 0 ? IkSolver::Method::SDLS : IkSolver::Method::DLS;
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> U(-70.0f, 70.0f);
        IkTool tool;
        tool.tipOffset = glm::vec3(0, 0, 45);
        int ok = 0;
        double us = 0.0;
        for (int k = 0; k < 300; ++k) {
            std::vector<float> q(5), start(5);
            for (auto& v : q) v = U(rng);
            for (int i = 0; i < 5; ++i) start[size_t(i)] = q[size_t(i)] + U(rng) * 0.4f;
            const glm::mat4 F = ik.toolFrame(*robot, q, tool);
            std::vector<IkTask> tasks(2);
            tasks[0].type = IkTask::Position;
            tasks[0].goal = glm::vec3(F[3]);
            tasks[1].type = IkTask::Direction;
            tasks[1].goal = glm::normalize(glm::vec3(F[2]));
            const auto t0 = std::chrono::steady_clock::now();
            const IkResult r = ik.solve(*robot, start, tasks, tool);
            us += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
            ok += r.converged;
        }
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s: %d / 300 yakinsadi (>= 285), ortalama %.0f us", method == 0 ? "SDLS" : "DLS", ok, us / 300.0);
        CHECK(ok >= 285, buf);
    }
    ik.method = IkSolver::Method::SDLS;

    section("4) egim: konum birincil, yon ikincil gorev");
    for (int variant = 0; variant < 2; ++variant) {
        std::mt19937 rng(7);
        std::uniform_real_distribution<float> U(-70.0f, 70.0f), T(-25.0f, 25.0f);
        IkTool tool;
        tool.tipOffset = glm::vec3(0, 0, 25 + 40); // tool tip + 40 mm aim distance
        int posOk = 0;
        float maxTilt = 0.0f, maxExcess = 0.0f;
        for (int k = 0; k < 200; ++k) {
            std::vector<float> q(5), start(5);
            for (auto& v : q) v = U(rng);
            for (int i = 0; i < 5; ++i) start[size_t(i)] = q[size_t(i)] + U(rng) * 0.3f;
            const glm::mat4 F = ik.toolFrame(*robot, q, tool);
            const glm::vec3 d = glm::normalize(glm::vec3(F[2]));
            // variant 0: the requested direction is reachable; 1: tilted by up to 25 degrees
            const float ang = variant == 0 ? 0.0f : glm::radians(T(rng));
            const glm::vec3 ax = glm::normalize(glm::cross(d, glm::vec3(0.3f, 0.7f, 0.2f)));
            const glm::vec3 want = glm::normalize(d * std::cos(ang) + glm::cross(ax, d) * std::sin(ang));
            std::vector<IkTask> tasks(2);
            tasks[0].type = IkTask::Position;
            tasks[0].goal = glm::vec3(F[3]);
            tasks[1].type = IkTask::Direction;
            tasks[1].goal = want;
            tasks[1].primary = false;
            const IkResult r = ik.solve(*robot, start, tasks, tool);
            posOk += r.positionError < 0.05f;
            maxTilt = std::max(maxTilt, r.secondaryError);
            maxExcess = std::max(maxExcess, r.secondaryError - std::abs(glm::degrees(ang)));
        }
        if (variant == 0) {
            CHECK(posOk >= 195, "ulasilabilir yon: konum " + std::to_string(posOk) + " / 200");
            CHECK(maxTilt < 0.1f, "ulasilabilir yon: egim kalmiyor (en fazla " + std::to_string(maxTilt) + " derece)");
        } else {
            CHECK(posOk >= 195, "egik istenen yon: konum yine tam (" + std::to_string(posOk) + " / 200)");
            CHECK(maxExcess < 0.5f, "egim hicbir zaman istenen sapmadan buyuk degil");
        }
    }
    return pegasus_test::finish("ik_test");
}
