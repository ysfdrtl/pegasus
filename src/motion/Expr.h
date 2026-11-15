#pragma once
// Small math expression compiler/evaluator for motion formulas, e.g.
//   "A * sin(2*pi*s/W)"   "h + D*(0.5 + 0.5*cos(2*pi*s/P))"   "if(u < 0.5, 1, -1)"
//
// Operators: + - * / % ^ (power), unary -, comparisons < > <= >= == != (1 or 0), && || !
// Functions: sin cos tan asin acos atan atan2 sqrt abs exp log log10 pow min max floor ceil
//            round fract mod clamp sign step smoothstep lerp if deg rad
//            tri(x)  triangle wave, period 1, range [-1, 1], tri(0) = 0
//            saw(x)  sawtooth wave, period 1, range [-1, 1], saw(0) = 0
//            sq(x)   square wave, period 1, +1 for the first half, -1 for the second
//            pulse(x, duty)  1 while fract(x) < duty, else 0
// Constants: pi, tau, e

#include <string>
#include <vector>

namespace pegasus {

class Expr {
public:
    // Compiles `source`; variables are resolved against `variables` (their index is used by eval).
    bool compile(const std::string& source, const std::vector<std::string>& variables, std::string& error);
    double eval(const double* vars) const;
    bool valid() const { return valid_; }
    const std::string& source() const { return source_; }

    static const std::vector<std::string>& functionNames();

    enum Op : unsigned char {
        Const, Var, Add, Sub, Mul, Div, Mod, Pow, Neg, Not, Lt, Gt, Le, Ge, Eq, Ne, And, Or, Call
    };
    struct Ins {
        Op op;
        int index; // variable index or function id
        double value;
    };

private:
    std::vector<Ins> code_;
    int maxStack_ = 0;
    bool valid_ = false;
    std::string source_;
};

} // namespace pegasus
