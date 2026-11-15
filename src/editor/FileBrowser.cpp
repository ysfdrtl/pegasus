#include "editor/FileBrowser.h"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace fs = std::filesystem;

namespace pegasus {

namespace {
bool hasStlExt(const std::string& name) {
    if (name.size() < 4) return false;
    std::string ext = name.substr(name.size() - 4);
    for (auto& c : ext) c = char(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".stl";
}
} // namespace

namespace {
// user's home folder: HOME on Linux / macOS, USERPROFILE on Windows
fs::path homeDir() {
    if (const char* h = std::getenv("HOME")) return fs::path(h);
    if (const char* h = std::getenv("USERPROFILE")) return fs::path(h);
    std::error_code ec;
    return fs::current_path(ec).root_path();
}
} // namespace

FileBrowser::FileBrowser(const char* title, bool directoryMode) : title_(title), directoryMode_(directoryMode) {
    std::error_code ec;
    dir_ = fs::current_path(ec);
    if (ec) {
        dir_ = homeDir();
    }
}

void FileBrowser::refresh() {
    entries_.clear();
    stlCount_ = 0;
    std::error_code ec;
    for (auto it = fs::directory_iterator(dir_, fs::directory_options::skip_permission_denied, ec);
         !ec && it != fs::directory_iterator(); it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.empty() || name[0] == '.') continue;
        std::error_code ec2;
        const bool isDir = it->is_directory(ec2);
        if (!isDir && hasStlExt(name)) stlCount_++;
        if (!isDir && (directoryMode_ || (!showAll_ && !hasStlExt(name)))) continue;
        entries_.push_back({name, isDir, isDir ? 0 : it->file_size(ec2)});
    }
    std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
        if (a.isDir != b.isDir) return a.isDir;
        return a.name < b.name;
    });
    std::snprintf(pathBuf_, sizeof(pathBuf_), "%s", dir_.string().c_str());
}

bool FileBrowser::draw(std::string& outPath) {
    bool picked = false;
    if (openRequested_) {
        openRequested_ = false;
        selected_.clear();
        refresh();
        ImGui::OpenPopup(title_.c_str());
    }
    ImGui::SetNextWindowSize(ImVec2(640, 460), ImGuiCond_Appearing);
    if (!ImGui::BeginPopupModal(title_.c_str(), nullptr)) return false;

    if (ImGui::Button("Yukarı")) {
        dir_ = dir_.parent_path();
        refresh();
    }
    ImGui::SameLine();
    if (ImGui::Button("Ev")) {
        dir_ = homeDir();
        refresh();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputText("##path", pathBuf_, sizeof(pathBuf_), ImGuiInputTextFlags_EnterReturnsTrue)) {
        std::error_code ec;
        fs::path p(pathBuf_);
        if (fs::is_directory(p, ec)) {
            dir_ = p;
            refresh();
        } else if (fs::is_regular_file(p, ec)) {
            outPath = p.string();
            picked = true;
        }
    }

    const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f;
    if (ImGui::BeginChild("##list", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
        for (const auto& e : entries_) {
            char label[1200];
            if (e.isDir) std::snprintf(label, sizeof(label), "[klasör]  %s", e.name.c_str());
            if (directoryMode_ && e.isDir) {
                if (ImGui::Selectable(label, selected_ == e.name, ImGuiSelectableFlags_AllowDoubleClick)) {
                    selected_ = e.name;
                    if (ImGui::IsMouseDoubleClicked(0)) {
                        dir_ /= e.name;
                        selected_.clear();
                        refresh();
                        break;
                    }
                }
                continue;
            }
            else std::snprintf(label, sizeof(label), "%s   (%.1f KB)", e.name.c_str(), double(e.size) / 1024.0);
            const bool isSel = !e.isDir && selected_ == e.name;
            if (ImGui::Selectable(label, isSel, ImGuiSelectableFlags_AllowDoubleClick)) {
                if (e.isDir) {
                    if (ImGui::IsMouseDoubleClicked(0)) {
                        dir_ /= e.name;
                        refresh();
                        break;
                    }
                } else {
                    selected_ = e.name;
                    if (ImGui::IsMouseDoubleClicked(0)) {
                        outPath = (dir_ / e.name).string();
                        picked = true;
                    }
                }
            }
        }
        if (directoryMode_) ImGui::TextDisabled("Bu klasörde %d STL dosyası var.", stlCount_);
        else if (entries_.empty()) ImGui::TextDisabled("(bu klasörde STL dosyası yok)");
    }
    ImGui::EndChild();

    if (directoryMode_) {
        if (ImGui::Button(selected_.empty() ? "Bu klasörü seç" : "Seçili klasörü seç")) {
            outPath = (selected_.empty() ? dir_ : dir_ / selected_).string();
            picked = true;
        }
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 70);
    } else {
        if (ImGui::Checkbox("Tüm dosyaları göster", &showAll_)) refresh();
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 150);
        ImGui::BeginDisabled(selected_.empty());
        if (ImGui::Button("Aç", ImVec2(70, 0))) {
            outPath = (dir_ / selected_).string();
            picked = true;
        }
        ImGui::EndDisabled();
    }
    ImGui::SameLine();
    if (ImGui::Button("İptal", ImVec2(70, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
    if (picked) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
    return picked;
}

} // namespace pegasus
