#pragma once
// Motion and tool libraries, stored as small text files so new ones can be written without
// recompiling:
//
//   library/motions/*.pmotion          library/tools/*.ptool
//
// A motion describes an offset in the *local seam frame* as a function of the distance s along
// the selected path:
//     x = lateral offset (across the seam)        y = extra offset along the travel direction
//     z = offset along the surface normal (up)    work_angle / travel_angle = tool tilt (deg)
// Example (straight line 1 mm above the seam):   x = 0   y = 0   z = h   with  param h = 1

#include "motion/Expr.h"

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace pegasus {

struct MotionParam {
    std::string name, label, unit;
    double value = 0.0, defaultValue = 0.0, minValue = -1000.0, maxValue = 1000.0;
};

struct MotionDef {
    std::string id, path, name, description;
    std::vector<MotionParam> params;
    std::string fx = "0", fy = "0", fz = "0", fWork = "0", fTravel = "0";
    // surface scans only (library/scans/*.pscan)
    std::string pattern = "zigzag"; // zigzag | oneway | crosshatch
    std::string link = "surface";   // surface | lift   (how consecutive passes are connected)
    std::string orient = "surface"; // surface (local normal) | fixed (mean face normal)

    const MotionParam* param(const std::string& n) const {
        for (const auto& p : params)
            if (p.name == n) return &p;
        return nullptr;
    }
    double paramValue(const std::string& n, double def) const {
        const MotionParam* p = param(n);
        return p ? p->value : def;
    }
};

struct ToolDef {
    std::string id, path, name, description;
    glm::vec3 tipOffset{0.0f}; // tool tip in the robot TCP frame (mm); tool axis = TCP +Z
    float radius = 3.0f;       // only for drawing
    glm::vec3 color{0.85f, 0.85f, 0.9f};
};

class MotionLibrary {
public:
    std::string dir;
    std::vector<MotionDef> motions;
    std::vector<MotionDef> scans; // surface scan patterns
    std::vector<ToolDef> tools;
    std::vector<std::string> warnings;

    bool load(const std::string& libraryDir);
    static std::string findLibraryDir();
    static bool parseMotion(const std::string& path, MotionDef& out, std::string& err);
    static bool parseTool(const std::string& path, ToolDef& out, std::string& err);
    static bool saveMotion(const MotionDef& m, const std::string& path, std::string& err, bool scan = false);
    static std::string fileIdFromName(const std::string& name);
};

// Formulas of a motion compiled against the variables  s u L t v  + parameter names.
class CompiledMotion {
public:
    enum { X, Y, Z, Work, Travel, Count };
    // Returns false if any formula failed; errors[i] describes each failure.
    bool compile(const MotionDef& def, std::string errors[Count]);
    // offset in the local seam frame (mm) and tool tilt angles (deg)
    // k = pass index, n = number of passes (surface scans; 0 and 1 for edge paths)
    void evaluate(const MotionDef& def, double s, double u, double L, double speed, glm::vec3& offset, float& workDeg,
                  float& travelDeg, double k = 0.0, double n = 1.0) const;
    static std::vector<std::string> baseVariables() { return {"s", "u", "L", "t", "v", "k", "n"}; }

private:
    Expr expr_[Count];
};

} // namespace pegasus
