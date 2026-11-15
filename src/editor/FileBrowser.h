#pragma once
// Minimal ImGui file chooser for STL files (no external dialog dependency).

#include <filesystem>
#include <string>
#include <vector>

namespace pegasus {

class FileBrowser {
public:
    explicit FileBrowser(const char* title = "STL Dosyası Aç", bool directoryMode = false);
    void open() { openRequested_ = true; }
    // Draws the modal if open. Returns true (once) when the user picked a file.
    bool draw(std::string& outPath);

private:
    void refresh();

    struct Entry {
        std::string name;
        bool isDir;
        uintmax_t size;
    };
    std::string title_;
    bool directoryMode_ = false;
    int stlCount_ = 0;
    bool openRequested_ = false;
    bool showAll_ = false;
    std::filesystem::path dir_;
    std::vector<Entry> entries_;
    std::string selected_;
    char pathBuf_[2048] = {};
};

} // namespace pegasus
