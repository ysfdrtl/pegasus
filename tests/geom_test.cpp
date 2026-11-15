// Mesh topology, ray casting, point-in-mesh, collision and STL loading.
#include "check.h"

#include "geom/Collision.h"
#include "scene/Mesh.h"
#include "scene/Primitives.h"
#include "scene/StlLoader.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>

using namespace pegasus;

static glm::mat4 at(float x, float y, float z) { return glm::translate(glm::mat4(1.0f), glm::vec3(x, y, z)); }

int main() {
    section("1) kutu topolojisi (10 mm kup)");
    {
        MeshPtr b = makeBox(glm::vec3(10.0f));
        CHECK(b->positions.size() == 8 && b->triangles.size() == 12, "8 kose, 12 ucgen (kaynaklanmis)");
        CHECK(b->featureEdges.size() == 12, "12 keskin kenar");
        CHECK(b->planarRegions.count() == 6, "6 duzlemsel yuzey");
        CHECK(b->outwardOriented, "kapali ve normaller disa bakiyor");
        CHECK(std::abs(b->totalArea - 600.0f) < 1e-3f, "alan 600 mm2");
        CHECK(std::abs(b->bounds.min.z) < 1e-5f && std::abs(b->bounds.max.z - 10.0f) < 1e-5f, "pivot tabanin ortasinda (z = 0..10)");
    }

    section("2) silindir ve kure");
    {
        MeshPtr c = makeCylinder(25.0f, 90.0f, 48);
        CHECK(c->smoothRegions.count() == 3, "silindir: 3 yumusak yuzey (ust, alt, yan)");
        CHECK(c->outwardOriented, "silindir kapali");
        MeshPtr s = makeSphere(35.0f, 24, 48);
        CHECK(s->smoothRegions.count() == 1, "kure: tek yumusak yuzey");
        CHECK(s->outwardOriented, "kure kapali (kutuplarda dikis acigi yok)");
        // the same sphere from a triangle soup welds back to the same topology
        std::vector<glm::vec3> soup;
        for (uint32_t t = 0; t < s->triangles.size(); ++t)
            for (int k = 0; k < 3; ++k) soup.push_back(s->triVertex(t, k));
        MeshPtr w = Mesh::fromSoup(soup, "soup");
        CHECK(w->positions.size() == s->positions.size() && w->triangles.size() == s->triangles.size(), "ucgen corbasindan kaynaklama ayni topolojiyi verir");
    }

    section("3) isin ve nokta-icinde testleri");
    {
        MeshPtr b = makeBox(glm::vec3(10.0f));
        RayHit h;
        CHECK(b->raycast(Ray{glm::vec3(0, 0, 50), glm::vec3(0, 0, -1)}, h) && std::abs(h.t - 40.0f) < 1e-4f, "yukaridan isin ust yuze 40 mm'de carpar");
        CHECK(!b->raycast(Ray{glm::vec3(20, 0, 50), glm::vec3(0, 0, -1)}, h), "yandan gecen isin carpmaz");
        CHECK(b->containsPoint(glm::vec3(0, 0, 5)) && !b->containsPoint(glm::vec3(0, 0, 11)), "icteki / distaki nokta");
    }

    section("4) carpisma");
    {
        MeshPtr a = makeBox(glm::vec3(10.0f)), b = makeBox(glm::vec3(10.0f)), big = makeBox(glm::vec3(50.0f));
        bool trunc = false, cont = false;
        CHECK(CollisionSystem::testPair(*a, at(0, 0, 0), *b, at(5, 0, 0), nullptr, 0, &trunc, &cont), "ic ice gecen kutular carpisiyor");
        CHECK(!CollisionSystem::testPair(*a, at(0, 0, 0), *b, at(10.5f, 0, 0), nullptr, 0, &trunc, &cont), "0.5 mm aralikli kutular carpismiyor");
        CHECK(!CollisionSystem::testPair(*a, at(0, 0, 0), *b, at(0, 0, 10), nullptr, 0, &trunc, &cont), "ust uste konan kutular carpisma sayilmaz");
        CHECK(CollisionSystem::touching(*a, at(0, 0, 0), *b, at(0, 0, 10), 0.01f), "... ama temas ediyor");
        cont = false;
        CHECK(CollisionSystem::testPair(*big, at(0, 0, 0), *a, at(0, 0, 20), nullptr, 0, &trunc, &cont) && cont,
              "tamamen icerideki kutu: icerme olarak algilaniyor");
    }

    section("5) STL okuma (ASCII ve binary)");
    {
        namespace fs = std::filesystem;
        const fs::path dir = PEGASUS_TEST_TMP;
        fs::create_directories(dir);
        MeshPtr b = makeBox(glm::vec3(10.0f)), s = makeSphere(20.0f);
        {
            std::ofstream f(dir / "t_ascii.stl");
            f << "solid t\n";
            for (uint32_t t = 0; t < b->triangles.size(); ++t) {
                f << "facet normal 0 0 0\nouter loop\n";
                for (int k = 0; k < 3; ++k) {
                    const glm::vec3 p = b->triVertex(t, k);
                    f << "vertex " << p.x << " " << p.y << " " << p.z << "\n";
                }
                f << "endloop\nendfacet\n";
            }
            f << "endsolid t\n";
        }
        {
            std::ofstream f(dir / "t_bin.stl", std::ios::binary);
            char hdr[80] = {};
            f.write(hdr, 80);
            const uint32_t n = uint32_t(s->triangles.size());
            f.write(reinterpret_cast<const char*>(&n), 4);
            for (uint32_t t = 0; t < n; ++t) {
                const float zero[3] = {0, 0, 0};
                f.write(reinterpret_cast<const char*>(zero), 12);
                for (int k = 0; k < 3; ++k) {
                    const glm::vec3 p = s->triVertex(t, k);
                    f.write(reinterpret_cast<const char*>(&p), 12);
                }
                const uint16_t attr = 0;
                f.write(reinterpret_cast<const char*>(&attr), 2);
            }
        }
        std::string err;
        MeshPtr a = loadStl((dir / "t_ascii.stl").string(), err);
        CHECK(a && a->triangles.size() == 12 && a->positions.size() == 8, "ASCII STL: kup geri okundu");
        MeshPtr c = loadStl((dir / "t_bin.stl").string(), err);
        CHECK(c && c->triangles.size() == s->triangles.size() && c->positions.size() == s->positions.size(), "binary STL: kure geri okundu");
        MeshPtr none = loadStl((dir / "yok.stl").string(), err);
        CHECK(!none && !err.empty(), "olmayan dosya: anlasilir hata (" + err + ")");
    }

    section("6) Turkce karakterli yollar (UTF-8, Windows'ta da)");
    {
        namespace fs = std::filesystem;
        // the app passes paths around as UTF-8 std::string (like "robot kol stl parçaları")
        const std::string dir = std::string(PEGASUS_TEST_TMP) + "/klasör_çığüşö İ";
        const std::string file = dir + "/parça_ş.stl";
        fs::create_directories(dir);
        {
            std::ofstream f(file);
            f << "solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid t\n";
        }
        bool listed = false;
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(dir, ec)) listed |= e.path().filename().string() == "parça_ş.stl";
        std::string err;
        MeshPtr m = loadStl(file, err);
        CHECK(fs::exists(file) && listed, "Turkce karakterli klasor ve dosya olusturuldu, adi dogru listeleniyor");
        CHECK(m && m->triangles.size() == 1, "Turkce karakterli yoldan STL okundu " + err);
        CHECK(m && m->name == "parça_ş", "dosya adindan gelen parca adi bozulmadi");
    }
    return pegasus_test::finish("geom_test");
}
