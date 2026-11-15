#include "motion/MotionLibrary.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace pegasus {

namespace fs = std::filesystem;

namespace {

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) {
            out.push_back(trim(cur));
            cur.clear();
        } else cur += c;
    }
    out.push_back(trim(cur));
    return out;
}

bool readLines(const std::string& path, std::vector<std::pair<std::string, std::string>>& kv, std::string& err) {
    std::ifstream f(path);
    if (!f) {
        err = "Dosya açılamadı: " + path;
        return false;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;
        const size_t eq = t.find('=');
        if (eq == std::string::npos) continue;
        kv.emplace_back(trim(t.substr(0, eq)), trim(t.substr(eq + 1)));
    }
    return true;
}

double num(const std::string& s, double def) {
    if (s.empty()) return def;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    return end == s.c_str() ? def : v;
}

} // namespace

std::string MotionLibrary::findLibraryDir() {
    std::error_code ec;
    if (const char* env = std::getenv("PEGASUS_LIBRARY"); env && fs::is_directory(env, ec)) return env;
    if (fs::is_directory("library", ec)) return fs::absolute("library", ec).string();
#ifdef PEGASUS_LIBRARY_DIR
    if (fs::is_directory(PEGASUS_LIBRARY_DIR, ec)) return PEGASUS_LIBRARY_DIR;
#endif
    return "library";
}

bool MotionLibrary::parseMotion(const std::string& path, MotionDef& m, std::string& err) {
    std::vector<std::pair<std::string, std::string>> kv;
    if (!readLines(path, kv, err)) return false;
    m = MotionDef{};
    m.path = path;
    m.id = fs::path(path).stem().string();
    m.name = m.id;
    for (const auto& [k, v] : kv) {
        if (k == "name") m.name = v;
        else if (k == "description") m.description = v;
        else if (k == "x") m.fx = v;
        else if (k == "y") m.fy = v;
        else if (k == "z") m.fz = v;
        else if (k == "work_angle") m.fWork = v;
        else if (k == "travel_angle") m.fTravel = v;
        else if (k == "pattern") m.pattern = v;
        else if (k == "link") m.link = v;
        else if (k == "orient") m.orient = v;
        else if (k.rfind("param", 0) == 0) {
            // param NAME = default | min | max | unit | label
            MotionParam p;
            p.name = trim(k.substr(5));
            const auto f = split(v, '|');
            p.defaultValue = p.value = num(f.size() > 0 ? f[0] : "", 0.0);
            p.minValue = num(f.size() > 1 ? f[1] : "", -1000.0);
            p.maxValue = num(f.size() > 2 ? f[2] : "", 1000.0);
            p.unit = f.size() > 3 ? f[3] : "";
            p.label = f.size() > 4 ? f[4] : p.name;
            if (p.name.empty()) {
                err = path + ": parametre adı eksik";
                return false;
            }
            m.params.push_back(p);
        }
    }
    return true;
}

bool MotionLibrary::parseTool(const std::string& path, ToolDef& t, std::string& err) {
    std::vector<std::pair<std::string, std::string>> kv;
    if (!readLines(path, kv, err)) return false;
    t = ToolDef{};
    t.path = path;
    t.id = fs::path(path).stem().string();
    t.name = t.id;
    for (const auto& [k, v] : kv) {
        std::istringstream in(v);
        if (k == "name") t.name = v;
        else if (k == "description") t.description = v;
        else if (k == "tip_offset") in >> t.tipOffset.x >> t.tipOffset.y >> t.tipOffset.z;
        else if (k == "radius") in >> t.radius;
        else if (k == "color") in >> t.color.r >> t.color.g >> t.color.b;
    }
    return true;
}

bool MotionLibrary::load(const std::string& libraryDir) {
    dir = libraryDir;
    motions.clear();
    scans.clear();
    tools.clear();
    warnings.clear();
    std::error_code ec;
    auto scan = [&](const fs::path& sub, const char* ext, auto&& fn) {
        std::vector<fs::path> files;
        for (const auto& e : fs::directory_iterator(fs::path(dir) / sub, ec))
            if (e.is_regular_file() && e.path().extension() == ext) files.push_back(e.path());
        std::sort(files.begin(), files.end());
        for (const auto& f : files) fn(f.string());
    };
    scan("motions", ".pmotion", [&](const std::string& f) {
        MotionDef m;
        std::string err;
        if (parseMotion(f, m, err)) motions.push_back(std::move(m));
        else warnings.push_back(err);
    });
    scan("scans", ".pscan", [&](const std::string& f) {
        MotionDef m;
        std::string err;
        if (parseMotion(f, m, err)) scans.push_back(std::move(m));
        else warnings.push_back(err);
    });
    scan("tools", ".ptool", [&](const std::string& f) {
        ToolDef t;
        std::string err;
        if (parseTool(f, t, err)) tools.push_back(std::move(t));
        else warnings.push_back(err);
    });
    // keep the straight motion first: it is the default
    std::stable_sort(motions.begin(), motions.end(), [](const MotionDef& a, const MotionDef& b) {
        return (a.id == "duz") > (b.id == "duz");
    });
    if (tools.empty()) {
        ToolDef t;
        t.id = "kiskac";
        t.name = "Kıskaç";
        tools.push_back(t);
        warnings.push_back("Takım kütüphanesi bulunamadı: " + (fs::path(dir) / "tools").string());
    }
    if (motions.empty()) warnings.push_back("Hareket kütüphanesi boş: " + (fs::path(dir) / "motions").string());
    return !motions.empty();
}

bool MotionLibrary::saveMotion(const MotionDef& m, const std::string& path, std::string& err, bool scanFile) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::ofstream f(path);
    if (!f) {
        err = "Dosya yazılamadı: " + path;
        return false;
    }
    if (scanFile) {
        f << "# Pegasus yüzey tarama kütüphanesi\n";
        f << "# Zorunlu parametreler: offset (kenardan), spacing (çizgi aralığı), standoff (yüzeyden uzaklık), angle (tarama açısı)\n";
        f << "# Formüller her tarama çizgisine eklenir: x = yan, y = çizgi boyunca, z = normal yönünde ek uzaklık\n";
        f << "# Değişkenler: s, u, L (çizgi boyunca), t, v, k (çizgi no), n (çizgi sayısı)\n";
    } else {
        f << "# Pegasus hareket kütüphanesi\n";
        f << "# Yerel eksenler: x = yan (dikişe dik), y = ilerleme yönü, z = yüzey normali (dışarı)\n";
        f << "# Değişkenler: s (yol boyunca mm), u (0..1), L (yol uzunluğu mm), t (saniye), v (hız mm/s)\n";
    }
    f << "name = " << m.name << "\n";
    if (!m.description.empty()) f << "description = " << m.description << "\n";
    for (const auto& p : m.params)
        f << "param " << p.name << " = " << p.defaultValue << " | " << p.minValue << " | " << p.maxValue << " | " << p.unit << " | "
          << p.label << "\n";
    f << "x = " << m.fx << "\n";
    f << "y = " << m.fy << "\n";
    f << "z = " << m.fz << "\n";
    f << "work_angle = " << m.fWork << "\n";
    f << "travel_angle = " << m.fTravel << "\n";
    if (scanFile) {
        f << "pattern = " << m.pattern << "\n";
        f << "link = " << m.link << "\n";
        f << "orient = " << m.orient << "\n";
    }
    return bool(f);
}

std::string MotionLibrary::fileIdFromName(const std::string& name) {
    // transliterate Turkish characters, keep [a-z0-9_]
    static const std::pair<const char*, const char*> tr[] = {{"ç", "c"}, {"Ç", "c"}, {"ğ", "g"}, {"Ğ", "g"}, {"ı", "i"}, {"İ", "i"},
                                                              {"ö", "o"}, {"Ö", "o"}, {"ş", "s"}, {"Ş", "s"}, {"ü", "u"}, {"Ü", "u"}};
    std::string s = name;
    for (const auto& [a, b] : tr) {
        for (size_t p = s.find(a); p != std::string::npos; p = s.find(a, p)) s.replace(p, std::char_traits<char>::length(a), b);
    }
    std::string out;
    for (char c : s) {
        const unsigned char u = static_cast<unsigned char>(c);
        if (std::isalnum(u)) out += char(std::tolower(u));
        else if (!out.empty() && out.back() != '_') out += '_';
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out.empty() ? "hareket" : out;
}

bool CompiledMotion::compile(const MotionDef& def, std::string errors[Count]) {
    std::vector<std::string> vars = baseVariables();
    for (const auto& p : def.params) vars.push_back(p.name);
    const std::string* src[Count] = {&def.fx, &def.fy, &def.fz, &def.fWork, &def.fTravel};
    bool ok = true;
    for (int i = 0; i < Count; ++i) {
        errors[i].clear();
        const std::string s = src[i]->empty() ? "0" : *src[i];
        if (!expr_[i].compile(s, vars, errors[i])) ok = false;
    }
    return ok;
}

void CompiledMotion::evaluate(const MotionDef& def, double s, double u, double L, double speed, glm::vec3& offset,
                              float& workDeg, float& travelDeg, double k, double n) const {
    double vars[64];
    vars[0] = s;
    vars[1] = u;
    vars[2] = L;
    vars[3] = speed > 1e-9 ? s / speed : 0.0;
    vars[4] = speed;
    vars[5] = k;
    vars[6] = n;
    const size_t np = std::min<size_t>(def.params.size(), 57);
    for (size_t i = 0; i < np; ++i) vars[7 + i] = def.params[i].value;
    offset = glm::vec3(float(expr_[X].eval(vars)), float(expr_[Y].eval(vars)), float(expr_[Z].eval(vars)));
    workDeg = float(expr_[Work].eval(vars));
    travelDeg = float(expr_[Travel].eval(vars));
}

} // namespace pegasus
