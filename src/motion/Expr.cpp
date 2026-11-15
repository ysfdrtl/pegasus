#include "motion/Expr.h"

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace pegasus {

namespace {

struct Func {
    const char* name;
    int arity;
};
const Func kFuncs[] = {
    {"sin", 1},   {"cos", 1},   {"tan", 1},   {"asin", 1},  {"acos", 1},  {"atan", 1},   {"atan2", 2},
    {"sqrt", 1},  {"abs", 1},   {"exp", 1},   {"log", 1},   {"log10", 1}, {"pow", 2},    {"min", 2},
    {"max", 2},   {"floor", 1}, {"ceil", 1},  {"round", 1}, {"fract", 1}, {"mod", 2},    {"clamp", 3},
    {"sign", 1},  {"step", 2},  {"smoothstep", 3}, {"lerp", 3}, {"if", 3}, {"deg", 1},   {"rad", 1},
    {"tri", 1},   {"saw", 1},   {"sq", 1},    {"pulse", 2},
};
constexpr int kFuncCount = int(sizeof(kFuncs) / sizeof(kFuncs[0]));

double fract(double x) { return x - std::floor(x); }

double callFunc(int id, const double* a) {
    switch (id) {
    case 0: return std::sin(a[0]);
    case 1: return std::cos(a[0]);
    case 2: return std::tan(a[0]);
    case 3: return std::asin(std::fmax(-1.0, std::fmin(1.0, a[0])));
    case 4: return std::acos(std::fmax(-1.0, std::fmin(1.0, a[0])));
    case 5: return std::atan(a[0]);
    case 6: return std::atan2(a[0], a[1]);
    case 7: return std::sqrt(std::fmax(0.0, a[0]));
    case 8: return std::abs(a[0]);
    case 9: return std::exp(a[0]);
    case 10: return std::log(a[0]);
    case 11: return std::log10(a[0]);
    case 12: return std::pow(a[0], a[1]);
    case 13: return std::fmin(a[0], a[1]);
    case 14: return std::fmax(a[0], a[1]);
    case 15: return std::floor(a[0]);
    case 16: return std::ceil(a[0]);
    case 17: return std::round(a[0]);
    case 18: return fract(a[0]);
    case 19: return a[1] != 0.0 ? a[0] - a[1] * std::floor(a[0] / a[1]) : 0.0;
    case 20: return std::fmax(a[1], std::fmin(a[2], a[0]));
    case 21: return a[0] > 0 ? 1.0 : (a[0] < 0 ? -1.0 : 0.0);
    case 22: return a[1] >= a[0] ? 1.0 : 0.0;
    case 23: {
        const double t = std::fmax(0.0, std::fmin(1.0, (a[2] - a[0]) / (a[1] - a[0])));
        return t * t * (3.0 - 2.0 * t);
    }
    case 24: return a[0] + (a[1] - a[0]) * a[2];
    case 25: return a[0] != 0.0 ? a[1] : a[2];
    case 26: return a[0] * 180.0 / M_PI;
    case 27: return a[0] * M_PI / 180.0;
    case 28: return 1.0 - 4.0 * std::abs(fract(a[0] + 0.25) - 0.5);
    case 29: return 2.0 * fract(a[0] + 0.5) - 1.0;
    case 30: return fract(a[0]) < 0.5 ? 1.0 : -1.0;
    case 31: return fract(a[0]) < a[1] ? 1.0 : 0.0;
    default: return 0.0;
    }
}

struct Parser {
    const std::string& s;
    const std::vector<std::string>& vars;
    std::vector<Expr::Ins>& code;
    size_t pos = 0;
    int depth = 0, maxDepth = 0;

    [[noreturn]] void fail(const std::string& msg) {
        throw std::runtime_error(msg + " (konum " + std::to_string(pos + 1) + ")");
    }
    void skip() {
        while (pos < s.size() && std::isspace(static_cast<unsigned char>(s[pos]))) ++pos;
    }
    bool accept(const char* tok) {
        skip();
        const size_t n = std::char_traits<char>::length(tok);
        if (s.compare(pos, n, tok) == 0) {
            pos += n;
            return true;
        }
        return false;
    }
    void emit(Expr::Op op, int index = 0, double value = 0.0, int stackDelta = 0) {
        code.push_back({op, index, value});
        depth += stackDelta;
        maxDepth = std::max(maxDepth, depth);
    }

    void parseExpr() { parseOr(); }
    void parseOr() {
        parseAnd();
        while (accept("||")) {
            parseAnd();
            emit(Expr::Or, 0, 0, -1);
        }
    }
    void parseAnd() {
        parseCmp();
        while (accept("&&")) {
            parseCmp();
            emit(Expr::And, 0, 0, -1);
        }
    }
    void parseCmp() {
        parseAdd();
        Expr::Op op;
        if (accept("<=")) op = Expr::Le;
        else if (accept(">=")) op = Expr::Ge;
        else if (accept("==")) op = Expr::Eq;
        else if (accept("!=")) op = Expr::Ne;
        else if (accept("<")) op = Expr::Lt;
        else if (accept(">")) op = Expr::Gt;
        else return;
        parseAdd();
        emit(op, 0, 0, -1);
    }
    void parseAdd() {
        parseMul();
        while (true) {
            if (accept("+")) {
                parseMul();
                emit(Expr::Add, 0, 0, -1);
            } else if (accept("-")) {
                parseMul();
                emit(Expr::Sub, 0, 0, -1);
            } else break;
        }
    }
    void parseMul() {
        parseUnary();
        while (true) {
            if (accept("*")) {
                parseUnary();
                emit(Expr::Mul, 0, 0, -1);
            } else if (accept("/")) {
                parseUnary();
                emit(Expr::Div, 0, 0, -1);
            } else if (accept("%")) {
                parseUnary();
                emit(Expr::Mod, 0, 0, -1);
            } else break;
        }
    }
    void parseUnary() {
        if (accept("-")) {
            parseUnary();
            emit(Expr::Neg);
        } else if (accept("+")) {
            parseUnary();
        } else if (accept("!")) {
            parseUnary();
            emit(Expr::Not);
        } else {
            parsePower();
        }
    }
    void parsePower() {
        parsePrimary();
        if (accept("^")) {
            parseUnary(); // right associative, binds tighter than unary minus on the left
            emit(Expr::Pow, 0, 0, -1);
        }
    }
    void parsePrimary() {
        skip();
        if (pos >= s.size()) fail("İfade beklenmeden bitti");
        const char c = s[pos];
        if (std::isdigit(static_cast<unsigned char>(c)) || c == '.') {
            char* end = nullptr;
            const double v = std::strtod(s.c_str() + pos, &end);
            if (end == s.c_str() + pos) fail("Geçersiz sayı");
            pos = size_t(end - s.c_str());
            emit(Expr::Const, 0, v, +1);
            return;
        }
        if (std::isalpha(static_cast<unsigned char>(c)) || c == '_') {
            size_t b = pos;
            while (pos < s.size() && (std::isalnum(static_cast<unsigned char>(s[pos])) || s[pos] == '_')) ++pos;
            const std::string id = s.substr(b, pos - b);
            if (accept("(")) {
                int f = -1;
                for (int i = 0; i < kFuncCount; ++i)
                    if (id == kFuncs[i].name) f = i;
                if (f < 0) fail("Bilinmeyen fonksiyon '" + id + "'");
                int args = 0;
                if (!accept(")")) {
                    do {
                        parseExpr();
                        ++args;
                    } while (accept(","));
                    if (!accept(")")) fail("')' bekleniyor");
                }
                if (args != kFuncs[f].arity)
                    fail("'" + id + "' " + std::to_string(kFuncs[f].arity) + " argüman alır, " + std::to_string(args) + " verildi");
                emit(Expr::Call, f, 0, 1 - args);
                return;
            }
            for (size_t i = 0; i < vars.size(); ++i)
                if (vars[i] == id) {
                    emit(Expr::Var, int(i), 0, +1);
                    return;
                }
            if (id == "pi") return emit(Expr::Const, 0, M_PI, +1);
            if (id == "tau") return emit(Expr::Const, 0, 2.0 * M_PI, +1);
            if (id == "e") return emit(Expr::Const, 0, M_E, +1);
            fail("Bilinmeyen değişken '" + id + "'");
        }
        if (accept("(")) {
            parseExpr();
            if (!accept(")")) fail("')' bekleniyor");
            return;
        }
        fail(std::string("Beklenmeyen karakter '") + c + "'");
    }
};

} // namespace

const std::vector<std::string>& Expr::functionNames() {
    static std::vector<std::string> names = [] {
        std::vector<std::string> n;
        for (const auto& f : kFuncs) n.push_back(f.name);
        return n;
    }();
    return names;
}

bool Expr::compile(const std::string& source, const std::vector<std::string>& variables, std::string& error) {
    source_ = source;
    code_.clear();
    valid_ = false;
    std::vector<Ins> code;
    Parser p{source, variables, code};
    try {
        p.skip();
        if (p.pos >= source.size()) throw std::runtime_error("Boş formül");
        p.parseExpr();
        p.skip();
        if (p.pos != source.size()) p.fail("Fazladan karakter");
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
    code_ = std::move(code);
    maxStack_ = std::max(p.maxDepth, 1);
    valid_ = true;
    error.clear();
    return true;
}

double Expr::eval(const double* vars) const {
    if (!valid_) return 0.0;
    double stackBuf[64];
    std::vector<double> heap;
    double* st = stackBuf;
    if (maxStack_ > 64) {
        heap.resize(size_t(maxStack_));
        st = heap.data();
    }
    int sp = 0;
    for (const Ins& in : code_) {
        switch (in.op) {
        case Const: st[sp++] = in.value; break;
        case Var: st[sp++] = vars[in.index]; break;
        case Neg: st[sp - 1] = -st[sp - 1]; break;
        case Not: st[sp - 1] = st[sp - 1] == 0.0 ? 1.0 : 0.0; break;
        case Call: {
            const int n = kFuncs[in.index].arity;
            sp -= n;
            st[sp] = callFunc(in.index, st + sp);
            ++sp;
            break;
        }
        default: {
            const double b = st[--sp], a = st[sp - 1];
            double r = 0.0;
            switch (in.op) {
            case Add: r = a + b; break;
            case Sub: r = a - b; break;
            case Mul: r = a * b; break;
            case Div: r = b != 0.0 ? a / b : 0.0; break;
            case Mod: r = b != 0.0 ? a - b * std::floor(a / b) : 0.0; break;
            case Pow: r = std::pow(a, b); break;
            case Lt: r = a < b; break;
            case Gt: r = a > b; break;
            case Le: r = a <= b; break;
            case Ge: r = a >= b; break;
            case Eq: r = std::abs(a - b) < 1e-12; break;
            case Ne: r = std::abs(a - b) >= 1e-12; break;
            case And: r = (a != 0.0 && b != 0.0); break;
            case Or: r = (a != 0.0 || b != 0.0); break;
            default: break;
            }
            st[sp - 1] = r;
        }
        }
    }
    const double v = sp > 0 ? st[sp - 1] : 0.0;
    return std::isfinite(v) ? v : 0.0;
}

} // namespace pegasus
