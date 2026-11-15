#pragma once
// The workspace editor: owns the scene, camera, selection, collision system and all UI.

#include "editor/Camera.h"
#include "editor/FileBrowser.h"
#include "editor/Picking.h"
#include "hw/ArduinoLink.h"
#include "geom/Collision.h"
#include "render/Renderer.h"
#include "motion/PathPlanner.h"
#include "motion/SurfaceScan.h"
#include "robot/IkSolver.h"
#include "robot/Robot.h"
#include "scene/Scene.h"

#include <imgui.h>
#include <ImGuizmo.h>

#include <array>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pegasus {

enum class Tool { Object = 0, Vertex = 1, Edge = 2, Face = 3 };

class Editor {
public:
    Editor();

    // Builds the UI and processes viewport input. Call between ImGui::NewFrame() and ImGui::Render().
    void update(float dt);
    // Fills render data for this frame. fbScale = framebuffer pixels per ImGui (window) unit.
    void buildFrameScene(FrameScene& fs, const glm::vec2& fbScale);

    void queueFile(const std::string& path) { pendingFiles_.push_back(path); }
    void queueRobot(const std::string& folder) { pendingRobot_ = folder; }
    void enableScanSelfTest(const std::string& folder, const std::string& outDir) {
        pendingRobot_ = folder;
        selfTestDir_ = outDir;
        scanSelfTest_ = true;
    }
    void enableSelectSelfTest(const std::string& folder, const std::string& outDir) {
        pendingRobot_ = folder;
        selfTestDir_ = outDir;
        selectSelfTest_ = true;
    }
    void enableArduinoSelfTest(const std::string& folder, const std::string& outDir) {
        pendingRobot_ = folder;
        selfTestDir_ = outDir;
        arduinoSelfTest_ = true;
    }
    void enableMotionSelfTest(const std::string& folder, const std::string& outDir) {
        pendingRobot_ = folder;
        selfTestDir_ = outDir;
        motionSelfTest_ = true;
    }
    void enableRobotSelfTest(const std::string& folder, const std::string& outDir) {
        pendingRobot_ = folder;
        selfTestDir_ = outDir;
        robotSelfTest_ = true;
    }

    // Automated UI test: injects mouse/keyboard events and requests screenshots (see SelfTest.cpp).
    void enableSelfTest(const std::string& outDir) { selfTestDir_ = outDir; }
    bool selfTestActive() const { return !selfTestDir_.empty(); }
    void selfTestInject(); // call after the platform NewFrame, before ImGui::NewFrame()
    std::string takeCaptureRequest() { std::string s; s.swap(captureRequest_); return s; }
    bool quitRequested() const { return quitRequested_; }

private:
    // --- actions
    bool openStl(const std::string& path);
    // kind: 0 = box (default 10 mm cube), 1 = cylinder, 2 = sphere
    void addPrimitive(int kind, const glm::vec3& boxSize = glm::vec3(10.0f));
    void placeBesideScene(SceneObject& obj);
    void dropToGround(SceneObject& obj);
    void deleteObject(uint32_t id);
    void duplicateActive();
    void setTool(Tool t);
    void clearSelection();
    void frameSelection();
    void frameAll();
    // Applies a new world matrix, honouring "block on collision". Returns false if blocked completely.
    bool applyTransform(SceneObject& obj, const glm::mat4& target);
    void notify(const std::string& msg, bool error = false);

    // --- robot (Editor_Robot.cpp)
    bool loadRobot(const std::string& folder);
    void removeRobot();
    void syncRobot();               // FK -> scene object matrices
    bool robotColliding() const;    // any robot part currently colliding (non-ignored)
    void setRobotAngle(int joint, float deg);
    void drawRobotPanel();
    void drawJointEditor();
    void drawRobotPartProperties(SceneObject& o);
    void addRobotRenderData(FrameScene& fs);
    void drawRobotOverlay(ImDrawList* dl);
    bool axisFromSelection(glm::vec3& originW, glm::vec3& axisW, std::string& how) const;
    void refreshRobotFolders();
    void selfTestRobotInject();

    // --- motion paths (Editor_Motion.cpp)
    void updateMotion(float dt);
    void recomputePlan();
    void drawMotionPanel();
    void addMotionRenderData(FrameScene& fs);
    void drawMotionOverlay(ImDrawList* dl);
    bool seamFromSelection();
    std::vector<SeamEdgeGeom> seamGeometry() const;
    bool scanFromSelection();
    std::vector<ScanFace> scanGeometry() const;
    std::vector<MotionDef>& activeDefs() { return motionMode_ == 1 ? library_.scans : library_.motions; }
    PlanSettings& activeSettings() { return motionMode_ == 1 ? scanSettings_ : planSettings_; }
    void setMotionMode(int mode);
    void selectMotion(int index);
    void recompileMotion();
    IkTool currentTool() const;
    void startRun();
    void stopRun();
    void selfTestMotionInject();
    void selfTestScanInject();
    void selfTestSelectInject();
    void selfTestArduinoInject();

    // --- UI
    void setupDockLayout(ImGuiID dockId);
    void drawMenuBar();
    void drawStatusBar();
    void drawToolsPanel();
    void drawScenePanel();
    void drawPropertiesPanel();
    void drawSelectionPanel();

    // --- Blender-like face selection helpers (Editor_Select.cpp)
    struct TopoCache {
        size_t triCount = 0;
        std::vector<uint32_t> vtOffsets, vtTris; // vertex -> triangles
        std::vector<uint32_t> quadMate;          // triangle paired into a quad with (or kInvalid)
    };
    const TopoCache& topology(const Mesh& m) const;
    void toTriangleSelection();
    void addTriangles(uint32_t objectId, const std::vector<uint32_t>& tris, bool deselect);
    std::vector<uint32_t> linkedTriangles(const Mesh& m, const std::vector<uint32_t>& seeds) const;
    bool selectLinkedUnderCursor(bool deselect);
    void selectLinkedFromSelection();
    void growShrinkSelection(bool grow);
    void selectAllFaces(uint32_t objectId);
    void invertFaceSelection();
    bool loopSelect(const SceneHit& hit, bool extend);
    void beginBrush(bool erase);
    void updateBrush(const glm::vec2& mouse);
    void brushPaint(const glm::vec2& mouse);
    void drawFaceSelectionTools();

    // --- Arduino (Editor_Arduino.cpp)
    struct ServoChannel {
        int joint = -1;       // robot joint driving this servo
        float center = 90.0f; // servo degrees when the joint is at 0
        float dir = 1.0f;     // +1 / -1
        float minDeg = 0.0f, maxDeg = 180.0f;
        float jog = 90.0f;    // manual slider
    };
    // --- undo / redo (Editor_Undo.cpp)
    struct UndoState {
        std::vector<SceneObject> objects;
        bool hasRobot = false;
        std::vector<RobotJoint> joints;
        glm::mat4 mount{1.0f};
        int tcpPart = -1;
        glm::vec3 tcpPoint{0.0f};
    };
    UndoState captureState() const;
    bool sameState(const UndoState& a, const UndoState& b) const;
    std::string describeChange(const UndoState& from, const UndoState& to) const;
    void restoreState(const UndoState& s);
    void updateUndo();
    void clearHistory();
    void undo();
    void redo();

    void drawArduinoWindow();
    void emergencyStop(); // Space / button / menu: stop the real arm and the run animation
    void updateArduino();
    float servoTarget(int channel) const;
    void sendServoLimits();
    void resetServoChannels();
    bool loadArduinoConfig();
    bool saveArduinoConfig(std::string& err) const;
    std::string arduinoConfigPath() const;
    void drawCollisionPanel();
    void drawHelpWindow();
    void drawViewport();
    void handleShortcuts();

    // --- viewport helpers
    void handleViewportInput();
    void updateHover();
    void drawOverlay(ImDrawList* dl);
    bool selectionContains(const ElementRef& r) const;
    glm::vec3 vertexWorld(const ElementRef& r) const;
    void edgeWorld(const ElementRef& r, glm::vec3& a, glm::vec3& b) const;

    Scene scene_;
    Camera camera_;
    CollisionSystem collision_;
    FileBrowser fileBrowser_;

    Tool tool_ = Tool::Object;
    FaceMode faceMode_ = FaceMode::Planar;
    EdgeSet edgeSet_ = EdgeSet::Feature;
    ImGuizmo::OPERATION gizmoOp_ = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE gizmoMode_ = ImGuizmo::WORLD;
    bool gizmoEnabled_ = true;

    uint32_t activeObject_ = 0;
    Tool selKind_ = Tool::Object;
    FaceMode selFaceMode_ = FaceMode::Planar;
    std::vector<ElementRef> selection_;
    ElementRef hover_;
    uint32_t hoverObject_ = 0;

    // viewport rect (ImGui units)
    ImVec2 vpPos_{0, 0}, vpSize_{1, 1};
    bool vpHovered_ = false;

    // free dragging
    bool dragCandidate_ = false, dragging_ = false, dragVertical_ = false;
    uint32_t dragObject_ = 0;
    ImVec2 pressPos_{0, 0};
    glm::vec3 grabPoint_{0.0f}, dragStartPos_{0.0f}, dragPlaneN_{0, 0, 1};
    glm::mat4 dragStartMatrix_{1.0f};
    bool cameraDrag_ = false;

    // hover cache key
    glm::vec2 lastHoverMouse_{-1.0f};
    uint64_t lastHoverVersion_ = 0;
    glm::mat4 lastHoverView_{0.0f};
    Tool lastHoverTool_ = Tool::Object;

    // undo / redo
    std::vector<UndoState> undoStack_, redoStack_;
    UndoState committed_;
    const Robot* undoRobot_ = nullptr;

    // Arduino
    std::unique_ptr<ArduinoLink> link_;
    bool showArduino_ = false, followRobot_ = false, arduinoCfgLoaded_ = false;
    std::vector<SerialPortInfo> ports_;
    char portBuf_[256] = {};
    char cmdBuf_[128] = {};
    int baud_ = 115200;
    float servoSpeed_ = 60.0f;
    bool jogActive_ = false;
    std::array<ServoChannel, kServoChannels> servoChannels_{};
    LinkState lastLinkState_ = LinkState::Disconnected;

    // face selection helpers
    float linkAngle_ = 45.0f;
    bool brushMode_ = false, brushActive_ = false, brushErase_ = false;
    float brushRadius_ = 30.0f;
    glm::vec2 brushLast_{-1.0f};
    std::unordered_set<uint64_t> brushSet_;
    mutable std::unordered_map<uint64_t, TopoCache> topo_;

    // settings
    bool showGrid_ = true;
    bool showEdges_ = true;
    bool showContacts_ = true;
    bool blockOnCollision_ = false;
    bool autoPlace_ = true;
    bool snap_ = false;
    float snapTranslate_ = 5.0f, snapRotate_ = 15.0f, snapScale_ = 0.1f;
    float featureAngle_ = 30.0f;
    float pickRadius_ = 12.0f;
    bool showHelp_ = false;
    bool layoutBuilt_ = false;
    bool quitRequested_ = false;
    float blockedFlash_ = 0.0f;

    std::vector<std::string> pendingFiles_;

    // robot
    std::unique_ptr<Robot> robot_;
    FileBrowser folderBrowser_{"Robot Klasörü Seç", true};
    std::string pendingRobot_;
    std::vector<std::string> robotLog_;
    std::vector<std::string> robotFolders_;
    bool robotFoldersScanned_ = false;
    int selectedJoint_ = -1;
    bool showJointAxes_ = true, showTcp_ = true, showAllJoints_ = false;
    bool robotDirty_ = false;
    bool robotSelfTest_ = false;
    bool focusRobotPanel_ = false;

    // motion paths
    MotionLibrary library_;
    bool libraryLoaded_ = false;
    int toolIndex_ = 0, motionIndex_ = -1;
    MotionDef motion_;
    CompiledMotion compiled_;
    std::string formulaErr_[CompiledMotion::Count];
    bool motionValid_ = false, motionEdited_ = false;
    char formulaBuf_[CompiledMotion::Count][512] = {};
    char motionNameBuf_[128] = {};
    char newParamBuf_[32] = {};
    std::vector<ElementRef> seamEdges_;
    bool seamReverse_ = false;
    Seam seam_;
    std::string seamError_;
    PlanSettings planSettings_;
    PlanSettings scanSettings_ = [] {
        PlanSettings s;
        s.step = 2.0f;
        s.speed = 40.0f;
        s.approachSpeed = 60.0f;
        return s;
    }();
    int motionMode_ = 0; // 0 = edge path, 1 = surface scan
    int modeIndex_[2] = {-1, -1};
    std::vector<ElementRef> scanFaces_;
    FaceMode scanFaceMode_ = FaceMode::Planar;
    std::vector<glm::mat4> scanFaceMats_;
    std::vector<PathObstacle> obstacles_; // workpieces for collision-free tool-off moves
    void updateObstacles();
    ScanStats scanStats_;
    std::vector<PathPoint> planPoints_;
    std::string planError_;
    int planReachable_ = 0;
    float planMaxPosErr_ = 0.0f, planMaxDirErr_ = 0.0f;
    bool planDirty_ = true;
    glm::mat4 planMount_{0.0f};
    IkSolver ik_;
    Trajectory traj_;
    bool playing_ = false, paused_ = false;
    float playTime_ = 0.0f, playSpeed_ = 1.0f;
    std::string runWarning_;
    std::vector<glm::vec3> trace_;
    bool stopOnCollision_ = true, showTrace_ = true, showToolTicks_ = true;
    bool focusMotionPanel_ = false;
    bool motionSelfTest_ = false;
    bool scanSelfTest_ = false;
    bool selectSelfTest_ = false;
    bool arduinoSelfTest_ = false;
    std::string arduinoCfgOverride_; // tests: keep the user's robot folder untouched
    long arduinoErrBase_ = 0;
    std::string selfTestDir_, captureRequest_;
    int selfTestFrame_ = 0;
    int selfTestFailures_ = 0;
    std::string status_;
    bool statusError_ = false;
    float statusTimer_ = 0.0f;
    float fps_ = 0.0f;
};

} // namespace pegasus
