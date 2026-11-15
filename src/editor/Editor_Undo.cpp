// Undo / redo (Ctrl+Z / Ctrl+Y) for the workspace.
//
// Snapshot based: after every finished interaction (mouse released, no widget being edited, no
// gizmo in use) the scene is compared with the last committed state; if anything changed, the
// previous state goes onto the undo stack. A whole drag, gizmo move or slider movement is
// therefore one step, and every kind of edit (move, rotate, add, delete, duplicate, properties,
// joint sliders, the joint editor, robot placement) is covered without instrumenting each one.
// Not recorded: camera, selection, the Run animation. Loading or removing a robot clears the history.

#include "editor/Editor.h"

#include <algorithm>
#include <cstdio>

namespace pegasus {

namespace {
constexpr size_t kMaxUndo = 100;

bool sameTransform(const Transform& a, const Transform& b) {
    return a.position == b.position && a.rotationDeg == b.rotationDeg && a.scale == b.scale;
}
bool sameObject(const SceneObject& a, const SceneObject& b) {
    return a.id == b.id && a.name == b.name && a.mesh == b.mesh && sameTransform(a.transform, b.transform) && a.color == b.color &&
           a.visible == b.visible && a.collidable == b.collidable && a.robotPart == b.robotPart;
}
bool sameJoint(const RobotJoint& a, const RobotJoint& b) {
    return a.name == b.name && a.part == b.part && a.type == b.type && a.origin == b.origin && a.axis == b.axis && a.minDeg == b.minDeg &&
           a.maxDeg == b.maxDeg && a.angleDeg == b.angleDeg && a.mimic == b.mimic && a.multiplier == b.multiplier &&
           a.offsetDeg == b.offsetDeg;
}
} // namespace

Editor::UndoState Editor::captureState() const {
    UndoState s;
    s.objects.reserve(scene_.objects().size());
    for (const auto& o : scene_.objects()) s.objects.push_back(*o);
    if (robot_) {
        s.hasRobot = true;
        s.joints = robot_->joints;
        s.mount = robot_->mount;
        s.tcpPart = robot_->tcpPart;
        s.tcpPoint = robot_->tcpPoint;
    }
    return s;
}

bool Editor::sameState(const UndoState& a, const UndoState& b) const {
    if (a.objects.size() != b.objects.size() || a.hasRobot != b.hasRobot) return false;
    for (size_t i = 0; i < a.objects.size(); ++i)
        if (!sameObject(a.objects[i], b.objects[i])) return false;
    if (!a.hasRobot) return true;
    if (a.mount != b.mount || a.tcpPart != b.tcpPart || a.tcpPoint != b.tcpPoint || a.joints.size() != b.joints.size()) return false;
    for (size_t i = 0; i < a.joints.size(); ++i)
        if (!sameJoint(a.joints[i], b.joints[i])) return false;
    return true;
}

// Short Turkish description of what changed from `a` to `b` (shown in the status bar).
std::string Editor::describeChange(const UndoState& a, const UndoState& b) const {
    int added = 0, removed = 0, moved = 0, other = 0;
    for (const auto& o : b.objects) {
        auto it = std::find_if(a.objects.begin(), a.objects.end(), [&](const SceneObject& x) { return x.id == o.id; });
        if (it == a.objects.end()) ++added;
        else if (!sameTransform(it->transform, o.transform)) ++moved;
        else if (!sameObject(*it, o)) ++other;
    }
    for (const auto& o : a.objects)
        if (std::none_of(b.objects.begin(), b.objects.end(), [&](const SceneObject& x) { return x.id == o.id; })) ++removed;
    bool angles = false, robotEdit = false;
    if (a.hasRobot && b.hasRobot && a.joints.size() == b.joints.size()) {
        robotEdit = a.mount != b.mount || a.tcpPoint != b.tcpPoint || a.tcpPart != b.tcpPart;
        for (size_t i = 0; i < a.joints.size(); ++i) {
            if (a.joints[i].angleDeg != b.joints[i].angleDeg) angles = true;
            RobotJoint x = a.joints[i];
            x.angleDeg = b.joints[i].angleDeg;
            if (!sameJoint(x, b.joints[i])) robotEdit = true;
        }
    }
    std::string d;
    auto add = [&](const std::string& s) { d += (d.empty() ? "" : ", ") + s; };
    if (added) add(std::to_string(added) + " nesne ekleme");
    if (removed) add(std::to_string(removed) + " nesne silme");
    if (moved) add(std::to_string(moved) + " nesne taşıma");
    if (other) add("nesne özellikleri");
    if (angles) add("eklem açıları");
    if (robotEdit) add("robot ayarları");
    return d.empty() ? "değişiklik" : d;
}

void Editor::restoreState(const UndoState& s) {
    scene_.restore(s.objects);
    if (robot_ && s.hasRobot && s.joints.size() == robot_->joints.size()) {
        robot_->joints = s.joints;
        robot_->mount = s.mount;
        robot_->tcpPart = s.tcpPart;
        robot_->tcpPoint = s.tcpPoint;
        syncRobot();
    }
    // references to objects that no longer exist
    auto alive = [&](const ElementRef& r) { return scene_.find(r.objectId) != nullptr; };
    selection_.erase(std::remove_if(selection_.begin(), selection_.end(), [&](const ElementRef& r) { return !alive(r); }), selection_.end());
    if (!scene_.find(activeObject_)) activeObject_ = 0;
    hover_ = {};
    const size_t seamBefore = seamEdges_.size(), scanBefore = scanFaces_.size();
    seamEdges_.erase(std::remove_if(seamEdges_.begin(), seamEdges_.end(), [&](const ElementRef& r) { return !alive(r); }), seamEdges_.end());
    scanFaces_.erase(std::remove_if(scanFaces_.begin(), scanFaces_.end(), [&](const ElementRef& r) { return !alive(r); }), scanFaces_.end());
    if (seamEdges_.size() != seamBefore) seamEdges_.clear(); // a seam needs both of its edges
    (void)scanBefore;
    planDirty_ = true;
    lastHoverVersion_ = 0;
    committed_ = captureState();
}

void Editor::clearHistory() {
    undoStack_.clear();
    redoStack_.clear();
    committed_ = captureState();
}

void Editor::updateUndo() {
    if (robot_.get() != undoRobot_) { // robot loaded / removed: the old states do not fit any more
        undoRobot_ = robot_.get();
        clearHistory();
        return;
    }
    if (playing_) { // the Run animation moves the joints: not an edit
        committed_ = captureState();
        return;
    }
    const ImGuiIO& io = ImGui::GetIO();
    const bool busy = ImGui::IsAnyItemActive() || ImGuizmo::IsUsingAny() || dragging_ || dragCandidate_ || brushActive_ ||
                      io.MouseDown[0] || io.MouseDown[1] || io.MouseDown[2];
    if (busy) return;
    UndoState cur = captureState();
    if (sameState(cur, committed_)) return;
    undoStack_.push_back(std::move(committed_));
    if (undoStack_.size() > kMaxUndo) undoStack_.erase(undoStack_.begin());
    redoStack_.clear();
    committed_ = std::move(cur);
}

void Editor::undo() {
    updateUndo(); // commit a pending edit first, so it is the one undone
    if (undoStack_.empty()) {
        notify("Geri alınacak bir şey yok.");
        return;
    }
    UndoState prev = std::move(undoStack_.back());
    undoStack_.pop_back();
    const std::string what = describeChange(prev, committed_);
    redoStack_.push_back(committed_);
    restoreState(prev);
    notify("Geri alındı: " + what + " (" + std::to_string(undoStack_.size()) + " adım daha)");
}

void Editor::redo() {
    if (redoStack_.empty()) {
        notify("Yinelenecek bir şey yok.");
        return;
    }
    UndoState next = std::move(redoStack_.back());
    redoStack_.pop_back();
    const std::string what = describeChange(committed_, next);
    undoStack_.push_back(committed_);
    restoreState(next);
    notify("Yinelendi: " + what + " (" + std::to_string(redoStack_.size()) + " adım daha)");
}

} // namespace pegasus
