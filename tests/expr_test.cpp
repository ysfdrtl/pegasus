// Formula compiler: operators, functions, wave shapes and error messages.
#include "check.h"

#include "motion/Expr.h"

#include <cmath>

using namespace pegasus;

static double ev(const std::string& src, double s = 0.0, double a = 0.0) {
    Expr e;
    std::string err;
    if (!e.compile(src, {"s", "a"}, err)) return NAN;
    const double v[2] = {s, a};
    return e.eval(v);
}
static bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

int main() {
    section("1) aritmetik ve oncelik");
    CHECK(near(ev("1 + 2 * 3"), 7), "1 + 2 * 3 = 7");
    CHECK(near(ev("(1 + 2) * 3"), 9), "(1 + 2) * 3 = 9");
    CHECK(near(ev("2 ^ 3 ^ 2"), 512), "us sagdan: 2 ^ 3 ^ 2 = 512");
    CHECK(near(ev("-2 ^ 2"), -4), "-2 ^ 2 = -4");
    CHECK(near(ev("7 % 3"), 1), "7 % 3 = 1");
    CHECK(near(ev("s * a", 3, 4), 12), "degiskenler: s * a = 12");

    section("2) karsilastirma ve mantik");
    CHECK(near(ev("s < 2", 1), 1) && near(ev("s < 2", 3), 0), "s < 2");
    CHECK(near(ev("1 && 0 || 1"), 1) && near(ev("!1"), 0), "&& || !");
    CHECK(near(ev("if(s < 0.5, 1, -1)", 0.2), 1) && near(ev("if(s < 0.5, 1, -1)", 0.7), -1), "if(...)");

    section("3) fonksiyonlar ve sabitler");
    CHECK(near(ev("sin(pi / 2)"), 1) && near(ev("cos(0)"), 1), "sin, cos, pi");
    CHECK(near(ev("tau"), 2 * M_PI) && near(ev("e"), std::exp(1.0)), "tau, e");
    CHECK(near(ev("clamp(5, 0, 2)"), 2) && near(ev("min(3, 1)"), 1) && near(ev("max(3, 1)"), 3), "clamp, min, max");
    CHECK(near(ev("atan2(1, 1)"), M_PI / 4) && near(ev("sqrt(16)"), 4) && near(ev("abs(-3)"), 3), "atan2, sqrt, abs");
    CHECK(near(ev("deg(pi)"), 180) && near(ev("rad(180)"), M_PI), "deg, rad");
    CHECK(near(ev("lerp(0, 10, 0.25)"), 2.5) && near(ev("fract(2.75)"), 0.75), "lerp, fract");

    section("4) dalga bicimleri (periyot 1)");
    CHECK(near(ev("tri(0)"), 0) && near(ev("tri(0.25)"), 1) && near(ev("tri(0.75)"), -1), "tri: 0, +1, -1");
    CHECK(near(ev("saw(0)"), 0) && ev("saw(0.49)") > 0.9, "saw");
    CHECK(near(ev("sq(0.25)"), 1) && near(ev("sq(0.75)"), -1), "sq: ilk yari +1, ikinci yari -1");
    CHECK(near(ev("pulse(0.1, 0.3)"), 1) && near(ev("pulse(0.5, 0.3)"), 0), "pulse(x, doluluk)");

    section("5) hata mesajlari (Turkce, konumlu)");
    auto err = [](const std::string& src) {
        Expr e;
        std::string m;
        const bool ok = e.compile(src, {"s"}, m);
        return ok ? std::string() : m;
    };
    CHECK(!err("(1").empty(), "kapanmayan parantez: " + err("(1"));
    CHECK(!err("").empty(), "bos formul: " + err(""));
    CHECK(!err("foo(1)").empty(), "bilinmeyen fonksiyon: " + err("foo(1)"));
    CHECK(!err("q + 1").empty(), "bilinmeyen degisken: " + err("q + 1"));
    CHECK(!err("sin(1, 2)").empty(), "yanlis arguman sayisi: " + err("sin(1, 2)"));
    CHECK(!err("1 +").empty(), "eksik ifade: " + err("1 +"));
    return pegasus_test::finish("expr_test");
}
