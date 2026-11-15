#pragma once
// Surface coverage ("scan") paths for spray painting and similar processes.
//
// The selected triangles are first grouped into *patches*: connected triangles that are not
// separated by a crease sharper than the `crease` parameter (tiny pieces are merged into their
// neighbour). A patch is painted as ONE continuous surface no matter how many small polygons it
// is made of (spheres, scanned figures, fillets ...):
//
//   1. frame: N = mean outward normal (for closed / wrapped patches: the direction towards the
//      robot), U = main extent of the patch (PCA) rotated by `angle`, D = N x U
//   2. the patch is cut with parallel planes  dot(p, D) = h  ->  surface curves (iso-lines);
//      the plane distance adapts to the slope so that neighbouring curves are `spacing` mm
//      apart *on the surface*; the layout is centred on the patch
//   3. along each curve only the parts at least `offset` mm away from the patch boundary are kept
//   4. points are moved `standoff` mm out along the smoothed (vertex-interpolated, window
//      averaged) surface normal, so the tool does not wobble from facet to facet
//   5. motion formulas of the pattern are added to every pass (x = sideways, y = along the
//      pass, z = extra distance), like for edge paths
//
// Triangles facing away from N (the back of a closed object) are skipped and reported.
// Links between passes / faces (tool off) are checked against the obstacles (the workpieces) with a
// clearance and re-routed around them (e.g. over the shared edge of two faces of a cube).
//
// Required pattern parameters: offset, spacing, standoff, angle   (optional: lift, crease, tilt)
// tilt > 0: the spray gun may lean up to `tilt` degrees away from the normal where the arm cannot
// hold it perpendicular (5-DoF arms); its axis still hits the planned surface point from `standoff` mm.

#include "motion/MotionLibrary.h"
#include "motion/PathPlanner.h"

#include <glm/glm.hpp>

#include <string>
#include <vector>

namespace pegasus {

struct ScanFace {
    std::vector<glm::vec3> verts; // world positions (shared between adjacent triangles)
    std::vector<glm::uvec3> tris; // indices into verts (outward winding)
};

// Objects the tool must not pass through while moving between passes / faces (tool off).
using ScanObstacle = PathObstacle;

struct ScanStats {
    int faces = 0, passes = 0;  // faces = painted patches
    int inputTriangles = 0;
    float area = 0.0f;          // mm^2 that is painted
    float skippedArea = 0.0f;   // mm^2 facing away (not reachable by spraying from one side)
    float processLength = 0.0f; // mm of working passes
    int detours = 0;            // links that were re-routed around an obstacle
    std::vector<std::string> warnings;
};

// sprayFrom (optional): world point the spray comes from (e.g. above the robot base); used for
// patches without a clear outward direction (closed objects such as a whole sphere).
bool planSurfaceScan(const std::vector<ScanFace>& faces, const MotionDef& def, const CompiledMotion& motion,
                     const PlanSettings& settings, std::vector<PathPoint>& out, ScanStats& stats, std::string& err,
                     const glm::vec3* sprayFrom = nullptr, const std::vector<ScanObstacle>* obstacles = nullptr);

} // namespace pegasus
