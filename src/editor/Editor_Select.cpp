// Blender-like face selection helpers for the Face tool:
//   L / Shift+L        select / deselect linked surface under the cursor (delimited by an angle)
//   Ctrl+L             grow the selection to everything linked to it
//   Ctrl + / Ctrl -    grow / shrink the selection by one ring of neighbouring triangles
//   Alt+click          face loop (triangulated quads of STL meshes are paired back into quads)
//   C                  circle (brush) select: paint with LMB, Shift+LMB erases, wheel = radius
//   A / Ctrl+I         select all faces of the object / invert
// All of them work on single triangles, so a region selection is converted to triangles first.

#include "editor/Editor.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <unordered_set>

namespace pegasus {

namespace {
uint64_t refKey(uint32_t objectId, uint32_t tri) { return (uint64_t(objectId) << 32) | tri; }

uint32_t otherTri(const Mesh::Edge& e, uint32_t t) {
    if (e.faceCount != 2) return kInvalid;
    return e.tri[0] == t ? e.tri[1] : e.tri[0];
}
} // namespace

const Editor::TopoCache& Editor::topology(const Mesh& m) const {
    auto it = topo_.find(m.uid);
    if (it != topo_.end() && it->second.triCount == m.triangles.size()) return it->second;
    TopoCache c;
    c.triCount = m.triangles.size();
    // vertex -> triangles (CSR)
    c.vtOffsets.assign(m.positions.size() + 1, 0);
    for (const auto& t : m.triangles)
        for (int k = 0; k < 3; ++k) c.vtOffsets[t[k] + 1]++;
    for (size_t i = 1; i < c.vtOffsets.size(); ++i) c.vtOffsets[i] += c.vtOffsets[i - 1];
    c.vtTris.resize(c.vtOffsets.back());
    std::vector<uint32_t> fill(c.vtOffsets.begin(), c.vtOffsets.end() - 1);
    for (uint32_t t = 0; t < m.triangles.size(); ++t)
        for (int k = 0; k < 3; ++k) c.vtTris[fill[m.triangles[t][k]]++] = t;
    // pair triangles back into quads: both triangles must have the shared edge as their clearly
    // longest edge (the diagonal of the original quad) and be nearly coplanar
    auto longest = [&](uint32_t t) {
        float len[3];
        for (int k = 0; k < 3; ++k) {
            const Mesh::Edge& e = m.edges[m.triEdges[t][k]];
            len[k] = glm::length(m.positions[e.v[0]] - m.positions[e.v[1]]);
        }
        int best = 0;
        for (int k = 1; k < 3; ++k)
            if (len[k] > len[best]) best = k;
        const float second = std::max(len[(best + 1) % 3], len[(best + 2) % 3]);
        return len[best] > second * 1.01f ? m.triEdges[t][best] : kInvalid;
    };
    std::vector<uint32_t> lng(m.triangles.size());
    for (uint32_t t = 0; t < m.triangles.size(); ++t) lng[t] = longest(t);
    c.quadMate.assign(m.triangles.size(), kInvalid);
    const float cosMax = std::cos(glm::radians(35.0f));
    for (uint32_t t = 0; t < m.triangles.size(); ++t) {
        if (lng[t] == kInvalid) continue;
        const uint32_t u = otherTri(m.edges[lng[t]], t);
        if (u == kInvalid || lng[u] != lng[t]) continue;
        if (glm::dot(m.triNormals[t], m.triNormals[u]) < cosMax) continue;
        c.quadMate[t] = u;
    }
    return topo_[m.uid] = std::move(c);
}

void Editor::toTriangleSelection() {
    if (selKind_ != Tool::Face) {
        selection_.clear();
        selKind_ = Tool::Face;
        selFaceMode_ = FaceMode::Triangle;
        return;
    }
    if (selFaceMode_ == FaceMode::Triangle) return;
    std::vector<ElementRef> tris;
    for (const auto& r : selection_) {
        const SceneObject* o = scene_.find(r.objectId);
        if (!o) continue;
        std::vector<uint32_t> ts;
        appendFaceTriangles(*o->mesh, selFaceMode_, r.index, ts);
        for (uint32_t t : ts) tris.push_back({r.objectId, t});
    }
    selection_ = std::move(tris);
    selFaceMode_ = FaceMode::Triangle;
}

void Editor::addTriangles(uint32_t objectId, const std::vector<uint32_t>& tris, bool deselect) {
    toTriangleSelection();
    if (deselect) {
        std::unordered_set<uint64_t> rm;
        for (uint32_t t : tris) rm.insert(refKey(objectId, t));
        selection_.erase(std::remove_if(selection_.begin(), selection_.end(),
                                        [&](const ElementRef& r) { return rm.count(refKey(r.objectId, r.index)) > 0; }),
                         selection_.end());
        return;
    }
    std::unordered_set<uint64_t> have;
    have.reserve(selection_.size() * 2 + tris.size());
    for (const auto& r : selection_) have.insert(refKey(r.objectId, r.index));
    for (uint32_t t : tris)
        if (have.insert(refKey(objectId, t)).second) selection_.push_back({objectId, t});
    activeObject_ = objectId;
}

std::vector<uint32_t> Editor::linkedTriangles(const Mesh& m, const std::vector<uint32_t>& seeds) const {
    std::vector<char> seen(m.triangles.size(), 0);
    std::deque<uint32_t> q;
    for (uint32_t s : seeds)
        if (s < m.triangles.size() && !seen[s]) {
            seen[s] = 1;
            q.push_back(s);
        }
    const bool unlimited = linkAngle_ >= 179.5f;
    std::vector<uint32_t> out;
    while (!q.empty()) {
        const uint32_t t = q.front();
        q.pop_front();
        out.push_back(t);
        for (int k = 0; k < 3; ++k) {
            const Mesh::Edge& e = m.edges[m.triEdges[t][k]];
            if (!unlimited && e.angleDeg > linkAngle_) continue;
            for (int j = 0; j < 2; ++j) {
                const uint32_t n = e.tri[j];
                if (n == kInvalid || seen[n]) continue;
                seen[n] = 1;
                q.push_back(n);
            }
        }
    }
    return out;
}

bool Editor::selectLinkedUnderCursor(bool deselect) {
    PickContext ctx;
    ctx.scene = &scene_;
    ctx.camera = &camera_;
    ctx.viewportSize = glm::vec2(vpSize_.x, vpSize_.y);
    const ImGuiIO& io = ImGui::GetIO();
    ctx.mouse = glm::vec2(io.MousePos.x - vpPos_.x, io.MousePos.y - vpPos_.y);
    const SceneHit h = pickSurface(ctx);
    if (!h.hit()) return false;
    const SceneObject* o = scene_.find(h.objectId);
    const auto tris = linkedTriangles(*o->mesh, {h.tri});
    addTriangles(h.objectId, tris, deselect);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "Bağlı yüzey %s: %zu üçgen (sınır açısı %.0f°)", deselect ? "çıkarıldı" : "seçildi", tris.size(), linkAngle_);
    notify(buf);
    return true;
}

void Editor::selectLinkedFromSelection() {
    if (selKind_ != Tool::Face || selection_.empty()) return;
    toTriangleSelection();
    std::unordered_map<uint32_t, std::vector<uint32_t>> seeds;
    for (const auto& r : selection_) seeds[r.objectId].push_back(r.index);
    for (auto& [id, s] : seeds)
        if (const SceneObject* o = scene_.find(id)) addTriangles(id, linkedTriangles(*o->mesh, s), false);
}

void Editor::growShrinkSelection(bool grow) {
    if (selKind_ != Tool::Face || selection_.empty()) return;
    toTriangleSelection();
    std::unordered_map<uint32_t, std::vector<uint32_t>> byObj;
    for (const auto& r : selection_) byObj[r.objectId].push_back(r.index);
    std::vector<ElementRef> result;
    for (auto& [id, tris] : byObj) {
        const SceneObject* o = scene_.find(id);
        if (!o) continue;
        const Mesh& m = *o->mesh;
        const TopoCache& c = topology(m);
        std::vector<char> sel(m.triangles.size(), 0);
        for (uint32_t t : tris)
            if (t < sel.size()) sel[t] = 1;
        std::vector<char> next = sel;
        for (uint32_t t : tris) {
            if (t >= sel.size()) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t v = m.triangles[t][k];
                for (uint32_t i = c.vtOffsets[v]; i < c.vtOffsets[v + 1]; ++i) {
                    const uint32_t n = c.vtTris[i];
                    if (grow) next[n] = 1;
                    else if (!sel[n]) next[t] = 0; // touches an unselected triangle -> border
                }
            }
        }
        for (uint32_t t = 0; t < next.size(); ++t)
            if (next[t]) result.push_back({id, t});
    }
    selection_ = std::move(result);
}

void Editor::selectAllFaces(uint32_t objectId) {
    const SceneObject* o = scene_.find(objectId);
    if (!o) return;
    std::vector<uint32_t> all(o->mesh->triangles.size());
    for (uint32_t t = 0; t < all.size(); ++t) all[t] = t;
    if (selKind_ != Tool::Face) selection_.clear();
    addTriangles(objectId, all, false);
}

void Editor::invertFaceSelection() {
    if (selKind_ != Tool::Face) return;
    toTriangleSelection();
    std::vector<uint32_t> objs;
    std::unordered_set<uint64_t> have;
    for (const auto& r : selection_) {
        have.insert(refKey(r.objectId, r.index));
        if (std::find(objs.begin(), objs.end(), r.objectId) == objs.end()) objs.push_back(r.objectId);
    }
    if (objs.empty() && activeObject_) objs.push_back(activeObject_);
    std::vector<ElementRef> result;
    for (uint32_t id : objs)
        if (const SceneObject* o = scene_.find(id))
            for (uint32_t t = 0; t < o->mesh->triangles.size(); ++t)
                if (!have.count(refKey(id, t))) result.push_back({id, t});
    selection_ = std::move(result);
}

bool Editor::loopSelect(const SceneHit& hit, bool extend) {
    const SceneObject* o = scene_.find(hit.objectId);
    if (!o) return false;
    const Mesh& m = *o->mesh;
    const TopoCache& c = topology(m);
    const glm::vec3 pLocal = glm::vec3(glm::inverse(o->matrix()) * glm::vec4(hit.point, 1.0f));

    // a "unit" is a quad (triangle + its mate) or a lone triangle; its outer edges exclude the diagonal
    auto outerEdges = [&](uint32_t t, std::vector<uint32_t>& es) {
        es.clear();
        const uint32_t mate = c.quadMate[t];
        for (uint32_t tt : {t, mate}) {
            if (tt == kInvalid) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t e = m.triEdges[tt][k];
                if (mate != kInvalid && otherTri(m.edges[e], tt) == (tt == t ? mate : t)) continue; // diagonal
                es.push_back(e);
            }
        }
    };
    auto segDist = [&](uint32_t e) {
        const glm::vec3 a = m.positions[m.edges[e].v[0]], b = m.positions[m.edges[e].v[1]];
        const glm::vec3 ab = b - a;
        const float l2 = glm::dot(ab, ab);
        const float t = l2 > 0 ? std::clamp(glm::dot(pLocal - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
        return glm::length(pLocal - (a + ab * t));
    };
    auto opposite = [&](const std::vector<uint32_t>& es, uint32_t e) {
        if (es.size() != 4) return kInvalid;
        const Mesh::Edge& E = m.edges[e];
        for (uint32_t f : es) {
            const Mesh::Edge& F = m.edges[f];
            if (F.v[0] != E.v[0] && F.v[0] != E.v[1] && F.v[1] != E.v[0] && F.v[1] != E.v[1]) return f;
        }
        return kInvalid;
    };

    std::vector<uint32_t> es;
    outerEdges(hit.tri, es);
    uint32_t clicked = es.front();
    for (uint32_t e : es)
        if (segDist(e) < segDist(clicked)) clicked = e;

    std::vector<char> visited(m.triangles.size(), 0);
    std::vector<uint32_t> loop;
    auto addUnit = [&](uint32_t t) {
        for (uint32_t tt : {t, c.quadMate[t]})
            if (tt != kInvalid && !visited[tt]) {
                visited[tt] = 1;
                loop.push_back(tt);
            }
    };
    addUnit(hit.tri);
    auto across = [&](uint32_t unitTri, uint32_t edge) {
        const Mesh::Edge& E = m.edges[edge];
        if (E.faceCount != 2) return kInvalid;
        const uint32_t mate = c.quadMate[unitTri];
        for (int j = 0; j < 2; ++j)
            if (E.tri[j] != unitTri && E.tri[j] != mate) return E.tri[j];
        return kInvalid;
    };
    auto walk = [&](uint32_t startTri, uint32_t edge) {
        uint32_t cur = startTri;
        for (int guard = 0; guard < 100000 && edge != kInvalid; ++guard) {
            const uint32_t next = across(cur, edge);           // leave the current unit through `edge`
            if (next == kInvalid || visited[next]) break;      // open border or back at the start (closed loop)
            if (c.quadMate[next] == kInvalid) break;           // a triangle (pole, fan): the loop ends here
            addUnit(next);
            std::vector<uint32_t> ne;
            outerEdges(next, ne);
            edge = opposite(ne, edge);
            cur = next;
        }
    };
    walk(hit.tri, clicked);
    if (c.quadMate[hit.tri] != kInvalid) walk(hit.tri, opposite(es, clicked));

    if (!extend) {
        selection_.clear();
        selKind_ = Tool::Face;
        selFaceMode_ = FaceMode::Triangle;
    }
    addTriangles(hit.objectId, loop, false);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "Döngü seçimi: %zu üçgen (%zu dörtgen)", loop.size(), loop.size() / 2);
    notify(buf);
    return true;
}

void Editor::brushPaint(const glm::vec2& mouse) {
    const glm::vec2 vpSize(vpSize_.x, vpSize_.y);
    const Ray ray = camera_.rayFromViewport(mouse, vpSize);
    const SceneHit h = raycastScene(scene_, ray);
    if (!h.hit()) return;
    const SceneObject* o = scene_.find(h.objectId);
    if (!o || o->robotPart >= 0) return;
    const Mesh& m = *o->mesh;
    // brush radius in world units at the hit depth
    const float depth = std::max(1.0f, glm::dot(h.point - camera_.eye(), camera_.forward()));
    const float worldPerPx = 2.0f * depth * std::tan(glm::radians(camera_.fovYDeg) * 0.5f) / std::max(1.0f, vpSize.y);
    const glm::mat4 inv = glm::inverse(o->matrix());
    const float scale = std::cbrt(std::abs(glm::determinant(glm::mat3(o->matrix()))));
    const float r = brushRadius_ * worldPerPx / std::max(scale, 1e-6f);
    const glm::vec3 c = glm::vec3(inv * glm::vec4(h.point, 1.0f));
    const glm::vec3 eye = glm::vec3(inv * glm::vec4(camera_.eye(), 1.0f));
    const bool mirrored = glm::determinant(glm::mat3(o->matrix())) < 0.0f;
    auto inside = [&](uint32_t t) {
        const glm::vec3 a = m.triVertex(t, 0), b = m.triVertex(t, 1), d = m.triVertex(t, 2);
        const glm::vec3 cen = (a + b + d) / 3.0f;
        float facing = glm::dot(m.triNormals[t], eye - cen);
        if (mirrored) facing = -facing;
        if (facing <= 0.0f) return false; // only what the camera sees
        const float r2 = r * r;
        auto d2 = [&](const glm::vec3& p) { return glm::dot(p - c, p - c); };
        return d2(cen) <= r2 || d2(a) <= r2 || d2(b) <= r2 || d2(d) <= r2;
    };
    std::vector<uint32_t> tris;
    std::unordered_set<uint32_t> seen{h.tri};
    std::deque<uint32_t> q{h.tri};
    while (!q.empty()) {
        const uint32_t t = q.front();
        q.pop_front();
        tris.push_back(t);
        for (int k = 0; k < 3; ++k) {
            const Mesh::Edge& e = m.edges[m.triEdges[t][k]];
            for (int j = 0; j < 2; ++j) {
                const uint32_t n = e.tri[j];
                if (n == kInvalid || seen.count(n)) continue;
                seen.insert(n);
                if (inside(n)) q.push_back(n);
            }
        }
    }
    if (!brushErase_ && selKind_ == Tool::Face && selFaceMode_ != FaceMode::Triangle) toTriangleSelection();
    if (brushErase_) {
        if (selKind_ == Tool::Face) addTriangles(h.objectId, tris, true);
        return;
    }
    if (selKind_ != Tool::Face) toTriangleSelection();
    for (uint32_t t : tris)
        if (brushSet_.insert(refKey(h.objectId, t)).second) selection_.push_back({h.objectId, t});
    activeObject_ = h.objectId;
}

void Editor::beginBrush(bool erase) {
    brushErase_ = erase;
    brushActive_ = true;
    toTriangleSelection();
    brushSet_.clear();
    for (const auto& r : selection_) brushSet_.insert(refKey(r.objectId, r.index));
    brushLast_ = glm::vec2(-1.0f);
}

void Editor::updateBrush(const glm::vec2& mouse) {
    if (!brushActive_) return;
    if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        brushActive_ = false;
        return;
    }
    if (brushLast_.x < 0.0f) {
        brushPaint(mouse);
    } else {
        // fill the gap between frames so fast strokes leave no holes
        const float len = glm::length(mouse - brushLast_);
        const int n = std::max(1, int(std::ceil(len / std::max(2.0f, brushRadius_ * 0.5f))));
        for (int i = 1; i <= n; ++i) brushPaint(glm::mix(brushLast_, mouse, float(i) / float(n)));
    }
    brushLast_ = mouse;
}

void Editor::drawFaceSelectionTools() {
    ImGui::SeparatorText("Seçim yardımcıları");
    ImGui::SliderFloat("Sınır açısı", &linkAngle_, 1.0f, 180.0f, linkAngle_ >= 179.5f ? "sınırsız" : "%.0f°");
    ImGui::SetItemTooltip("Bağlı seçim bu açıdan keskin kenarlarda durur.\n180 = nesnenin bağlı tüm yüzeyi.");
    if (ImGui::Button("Bağlıyı seç (L)")) selectLinkedFromSelection();
    ImGui::SetItemTooltip("İmlecin altındaki bağlı yüzeyi seçmek için imleç nesnenin üzerindeyken L'ye basın\n(Shift+L çıkarır). Bu düğme / Ctrl+L mevcut seçimi bağlı yüzeye genişletir.");
    ImGui::SameLine();
    if (ImGui::Button("Büyüt (Ctrl +)")) growShrinkSelection(true);
    ImGui::SameLine();
    if (ImGui::Button("Küçült (Ctrl -)")) growShrinkSelection(false);
    if (ImGui::Button("Tümü (A)")) selectAllFaces(activeObject_);
    ImGui::SameLine();
    if (ImGui::Button("Tersini seç (Ctrl+I)")) invertFaceSelection();
    ImGui::SameLine();
    const bool on = brushMode_;
    if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    if (ImGui::Button("Fırça (C)")) brushMode_ = !brushMode_;
    if (on) ImGui::PopStyleColor();
    ImGui::SetItemTooltip("Fırçayla boyayarak seçim: sol tık sürükle = ekle, Shift + sürükle = çıkar,\nfare tekeri = fırça boyutu, C / Esc = çık.");
    if (brushMode_) ImGui::SliderFloat("Fırça yarıçapı", &brushRadius_, 4.0f, 200.0f, "%.0f px");
    ImGui::TextDisabled("Alt + tık: döngü seçimi (Shift ile ekle)");
}

} // namespace pegasus
