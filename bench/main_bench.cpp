#include <iostream>
#include <iomanip>
#include <random>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>

#include "Matrix3x3.hpp"
#include "Quat.hpp"

// -------------------- Colors ANSI ----------------------
static constexpr const char* GREEN = "\x1b[32m";
static constexpr const char* RED = "\x1b[31m";
static constexpr const char* YELL = "\x1b[33m";
static constexpr const char* CYAN = "\x1b[36m";
static constexpr const char* BOLD = "\x1b[1m";
static constexpr const char* RESET = "\x1b[0m";

// -------------------- Constants ------------------------
static constexpr double PI = 3.14159265358979323846;
static constexpr double TOL = 1e-6;
static constexpr double TOL_S = 1e-9;   // strict
static const Matrix3x3 M_ZERO{};      // Matriu de zeros (stub)
static const Quat Q_IDENT{};      // Quat identitat (stub)

// -------------------- Helpers numèrics -----------------
static bool Nearly(double a, double b, double eps = TOL) { return std::fabs(a - b) <= eps; }
static bool VecEq(const Vec3& a, const Vec3& b, double eps = TOL) { return Nearly(a.x, b.x, eps) && Nearly(a.y, b.y, eps) && Nearly(a.z, b.z, eps); }
static bool MatEq(const Matrix3x3& A, const Matrix3x3& B, double eps = TOL) { for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) if (!Nearly(A.At(i, j), B.At(i, j), eps)) return false; return true; }
static bool QuatEq(const Quat& a, const Quat& b, double eps = TOL) { return Nearly(a.s, b.s, eps) && Nearly(a.x, b.x, eps) && Nearly(a.y, b.y, eps) && Nearly(a.z, b.z, eps); }
// Comprova si dos quaternions representen la mateixa rotació (q == -q)
static bool QuatEqv(const Quat& a, const Quat& b, double eps = TOL) { return QuatEq(a, b, eps) || QuatEq(a, { -b.s, -b.x, -b.y, -b.z }, eps); }


static Vec3 RandUnit(std::mt19937& g) { std::uniform_real_distribution<double> U(-1.0, 1.0); Vec3 v{ U(g),U(g),U(g) }; double n = v.Norm(); if (n == 0) return { 1,0,0 }; return { v.x / n,v.y / n,v.z / n }; }
static Vec3 RandVec(std::mt19937& g) { std::uniform_real_distribution<double> U(-1.0, 1.0); return { U(g),U(g),U(g) }; }

// -------------------- Marc de proves -------------------
struct CaseResult {
    bool ok; std::string name; std::string detail;
};

struct Suite {
    std::string title; std::vector<CaseResult> cases; int passed = 0; int total = 0;
    explicit Suite(std::string t) : title(std::move(t)) {}
    void add(bool ok, const std::string& name, const std::string& detail = "") {
        cases.push_back({ ok,name,detail }); total++; if (ok) passed++;
    }
    bool print() const {
        std::cout << CYAN << "\n== " << title << " ==" << RESET << "\n";
        for (auto& c : cases) {
            std::cout << (c.ok ? GREEN : RED) << (c.ok ? "OK" : "KO") << RESET
                << "  " << c.name;
            if (!c.detail.empty()) std::cout << "  " << (c.ok ? CYAN : YELL) << c.detail << RESET;
            std::cout << "\n";
        }
        std::cout << BOLD << ((passed == total) ? GREEN : (passed ? YELL : RED))
            << "-- " << passed << "/" << total << " subtests" << RESET << "\n";
        return passed == total;
    }
};

// --------------- Helpers de Matrius (per Ex0) -----------------
static Matrix3x3 MakeRz(double a) {
    Matrix3x3 Rz = Matrix3x3::Identity();
    Rz.At(0, 0) = std::cos(a); Rz.At(0, 1) = -std::sin(a);
    Rz.At(1, 0) = std::sin(a); Rz.At(1, 1) = std::cos(a);
    return Rz;
}
static Matrix3x3 MakeRy(double a) {
    Matrix3x3 Ry = Matrix3x3::Identity();
    Ry.At(0, 0) = std::cos(a); Ry.At(0, 2) = std::sin(a);
    Ry.At(2, 0) = -std::sin(a); Ry.At(2, 2) = std::cos(a);
    return Ry;
}
static Matrix3x3 MakeScale(double sx, double sy, double sz) {
    Matrix3x3 S{};
    S.At(0, 0) = sx; S.At(1, 1) = sy; S.At(2, 2) = sz;
    return S;
}

// --------------- [Ex0] Fonaments Matrix3x3 -----------------
// Aquestes funcions són necessàries per a la resta de proves.
// Assegura't que passen abans de continuar.
// -------------------------------------------------------------

static void EX0_Test_Multiply_Vec(Suite& S) {
    Matrix3x3 Rz90 = MakeRz(PI / 2);
    Vec3 v{ 1, 0, 0 };
    Vec3 w = Rz90.Multiply(v);
    Vec3 w_expected{ 0, 1, 0 };
    S.add(VecEq(w, w_expected), "Matriu * Vector (Rz(90) * [1,0,0])", "Resultat esperat [0,1,0]");

    Matrix3x3 Scl = MakeScale(1, 2, 3);
    Vec3 v2{ 1, 1, 1 };
    Vec3 w2 = Scl.Multiply(v2);
    Vec3 w2_expected{ 1, 2, 3 };
    S.add(VecEq(w2, w2_expected), "Matriu * Vector (Escala * [1,1,1])", "Resultat esperat [1,2,3]");
}

static void EX0_Test_Multiply_Mat(Suite& S) {
    Matrix3x3 Rz90 = MakeRz(PI / 2);
    Matrix3x3 Ry90 = MakeRy(PI / 2);
    Matrix3x3 Rzy = Rz90.Multiply(Ry90);

    // Rzy = [ 0 -1 0 ]
    //       [ 0 0 1 ]
    //       [ -1 0 0 ]
    Matrix3x3 Rzy_expected{};
    Rzy_expected.At(0, 1) = -1; Rzy_expected.At(1, 2) = 1; Rzy_expected.At(2, 0) = -1;

    S.add(MatEq(Rzy, Rzy_expected) && !MatEq(Rzy, M_ZERO), "Matriu * Matriu (Rz90 * Ry90)", "Comprova composicio");
}

static void EX0_Test_Transpose(Suite& S) {
    Matrix3x3 Rz90 = MakeRz(PI / 2);
    Matrix3x3 Rz90_T = Rz90.Transposed();
    Matrix3x3 Rz_neg90 = MakeRz(-PI / 2);

    S.add(MatEq(Rz90_T, Rz_neg90) && !MatEq(Rz90_T, M_ZERO), "Transposed (Rz(90)^T == Rz(-90))");

    Matrix3x3 A{}; A.At(0, 1) = 1; A.At(0, 2) = 2; A.At(1, 2) = 3;
    Matrix3x3 AT_expected{}; AT_expected.At(1, 0) = 1; AT_expected.At(2, 0) = 2; AT_expected.At(2, 1) = 3;
    S.add(MatEq(A.Transposed(), AT_expected), "Transposed (Matriu triangular)");
}

static void EX0_Test_Det_And_Trace(Suite& S) {
    S.add(Nearly(Matrix3x3::Identity().Det(), 1.0), "Determinant(Identitat) == 1");
    S.add(Nearly(MakeRz(PI / 2).Det(), 1.0), "Determinant(Rotacio) == 1");
    S.add(Nearly(MakeScale(1, 2, 3).Det(), 6.0), "Determinant(Escala(1,2,3)) == 6");

    S.add(Nearly(Matrix3x3::Identity().Trace(), 3.0), "Trace(Identitat) == 3");
    S.add(Nearly(MakeScale(1, 2, 3).Trace(), 6.0), "Trace(Escala(1,2,3)) == 6");
}


// --------------- [Ex1] Matrius: Axis-Angle -----------------
// Prova les funcions principals de l'Exercici 1.
// Requereix Ex0.
// -----------------------------------------------------------

static void EX1_Test_RotationAxisAngle(Suite& S) {
    Vec3 uZ{ 0,0,1 };
    Matrix3x3 Rz90 = Matrix3x3::RotationAxisAngle(uZ, PI / 2);
    Matrix3x3 Rz90_expected = MakeRz(PI / 2);
    S.add(MatEq(Rz90, Rz90_expected) && !MatEq(Rz90, M_ZERO), "RotationAxisAngle(Z, 90deg)", "Compara amb matriu coneguda Rz");

    Vec3 uY{ 0,1,0 };
    Matrix3x3 Ry90 = Matrix3x3::RotationAxisAngle(uY, PI / 2);
    Matrix3x3 Ry90_expected = MakeRy(PI / 2);
    S.add(MatEq(Ry90, Ry90_expected) && !MatEq(Ry90, M_ZERO), "RotationAxisAngle(Y, 90deg)", "Compara amb matriu coneguda Ry");

    Vec3 u{ 0.3,-0.7,0.64 }; u = u.Normalize(); double a = 1.2;
    Matrix3x3 R1 = Matrix3x3::RotationAxisAngle(u, a);
    Matrix3x3 R2 = Matrix3x3::RotationAxisAngle(Vec3{ -u.x,-u.y,-u.z }, -a);
    S.add(MatEq(R1, R2, 1e-6) && !MatEq(R1, M_ZERO), "Simetria R(u,phi) == R(-u,-phi)");
}

static void EX1_Test_IsRotation(Suite& S) {
    Matrix3x3 I = Matrix3x3::Identity();
    S.add(I.IsRotation(), "IsRotation(Identitat) == true");

    Vec3 u{ 0,0,1 }; Matrix3x3 Rz = Matrix3x3::RotationAxisAngle(u, PI / 2);
    bool is_stub_rz = MatEq(Rz, M_ZERO, 1e-9);
    S.add(Rz.IsRotation() && !is_stub_rz, "IsRotation(Rz90) == true");

    Matrix3x3 Scl = MakeScale(1.01, 1, 1); S.add(!Scl.IsRotation(), "IsRotation(Escala) == false", "R^T R != I");
    Matrix3x3 Ref = MakeScale(1, 1, -1);  S.add(!Ref.IsRotation(), "IsRotation(Reflexio) == false", "det(R) != 1");
}

static void EX1_Test_Rotate(Suite& S) {
    std::mt19937 rng(7); bool all = true; int N = 80;
    for (int k = 0; k < N; ++k) {
        Vec3 u = RandUnit(rng); double a = std::uniform_real_distribution<double>(-PI, PI)(rng);
        Matrix3x3 R = Matrix3x3::RotationAxisAngle(u, a);
        bool is_stub = MatEq(R, M_ZERO, 1e-9);
        if (!R.IsRotation() || (is_stub && a != 0.0)) { all = false; break; }

        // vector paral·lel a u es manté (quasi) igual
        Vec3 vpar = u; Vec3 Rp = R.Rotate(vpar);
        bool dir_ok = VecEq(Rp, vpar, 1e-5) || (Nearly(std::fabs(a), PI, 1e-5) && VecEq(Rp, { -vpar.x,-vpar.y,-vpar.z }, 1e-5));

        // un vector perpendicular conserva la norma
        Vec3 v = RandVec(rng); double dot = Vec3::Dot(v, u); Vec3 vperp{ v.x - dot * u.x, v.y - dot * u.y, v.z - dot * u.z };
        Vec3 Rv = R.Rotate(vperp);
        bool norm_ok = Nearly(vperp.Norm(), Rv.Norm(), 1e-6);
        if (!(dir_ok && norm_ok)) { all = false; break; }
    }
    S.add(all, "Rotate: Propietats (eix i norma)", "N=" + std::to_string(N) + " iteracions");

    // 1. Yaw (Z) 90 graus: Hauria de moure l'eix X a l'eix Y
    Matrix3x3 R_Yaw = Matrix3x3::RotationAxisAngle({0,0,1}, PI / 2);
    Vec3 vX{ 1, 0, 0 };
    Vec3 w_Yaw = R_Yaw.Rotate(vX);
    Vec3 w_Yaw_expected{ 0, 1, 0 };
    S.add(VecEq(w_Yaw, w_Yaw_expected) && !MatEq(R_Yaw, M_ZERO), "RotationAxisAngle + Rotate (Yaw 90)", "Eix X -> Eix Y");

    // 2. Pitch (Y) 90 graus: Hauria de moure l'eix Z a l'eix X
    Matrix3x3 R_Pitch = Matrix3x3::RotationAxisAngle({0,1,0}, PI / 2);
    Vec3 vZ{ 0, 0, 1 };
    Vec3 w_Pitch = R_Pitch.Rotate(vZ);
    Vec3 w_Pitch_expected{ 1, 0, 0 };
    S.add(VecEq(w_Pitch, w_Pitch_expected) && !MatEq(R_Pitch, M_ZERO), "RotationAxisAngle + Rotate (Pitch 90)", "Eix Z -> Eix X");

    // 3. Roll (X) 90 graus: Hauria de moure l'eix Y a l'eix Z
    Matrix3x3 R_Roll = Matrix3x3::RotationAxisAngle({ 1,0,0 }, PI / 2);
    Vec3 vY{ 0, 1, 0 };
    Vec3 w_Roll = R_Roll.Rotate(vY);
    Vec3 w_Roll_expected{ 0, 0, 1 };
    S.add(VecEq(w_Roll, w_Roll_expected) && !MatEq(R_Roll, M_ZERO), "RotationAxisAngle + Rotate (Roll 90)", "Eix Y -> Eix Z");
}

static void EX1_Test_ToAxisAngle(Suite& S) {
    bool threw = false; try { Matrix3x3 Sx = MakeScale(1.1, 1, 1); Vec3 u; double a; Sx.ToAxisAngle(u, a); }
    catch (...) { threw = true; }
    S.add(threw, "ToAxisAngle: Llença excepcio si no-rotació");

    Matrix3x3 I = Matrix3x3::Identity(); Vec3 u_i; double a_i; I.ToAxisAngle(u_i, a_i);
    S.add(Nearly(a_i, 0.0, 1e-6), "ToAxisAngle(Identitat) -> angle == 0");

    Vec3 ur{ 0,1,0 }; Matrix3x3 Rp = Matrix3x3::RotationAxisAngle(ur, PI);
    bool is_stub_pi = MatEq(Rp, M_ZERO, 1e-9);
    Vec3 ue; double ae; Rp.ToAxisAngle(ue, ae); Matrix3x3 Rrec = Matrix3x3::RotationAxisAngle(ue, ae);
    S.add(MatEq(Rp, Rrec, 1e-6) && Nearly(std::fabs(ae), PI, 1e-5) && !is_stub_pi, "ToAxisAngle: Cas angle PI", "Reconstrueix correctament");

    std::mt19937 g(9); bool all = true; int N = 60;
    for (int k = 0; k < N; ++k) {
        Vec3 uu = RandUnit(g); double a = std::uniform_real_distribution<double>(-PI, PI)(g);
        Matrix3x3 R = Matrix3x3::RotationAxisAngle(uu, a);
        if (MatEq(R, M_ZERO, 1e-9) && a != 0.0) { all = false; break; }
        Vec3 u2; double a2; R.ToAxisAngle(u2, a2);
        Matrix3x3 R2 = Matrix3x3::RotationAxisAngle(u2, a2);
        if (!MatEq(R, R2, 1e-6)) { all = false; break; }
    }
    S.add(all, "Cicle AxisAngle -> Matriu -> AxisAngle", "N=" + std::to_string(N) + " iteracions aleatòries");
}

// --------------- [Ex2] Matrius: Angles d'Euler -----------------
// Prova les funcions de l'Exercici 2.
// Requereix Ex1.
// -------------------------------------------------------------

static void EX2_Test_Euler_Rotate(Suite& S) {
    // Prova que FromEulerZYX crea les matrius correctes fent rotacions simples.
    // Nota: Aquests tests fallen si Multiply(Vec3) o FromEulerZYX són incorrectes.

    // 1. Yaw (Z) 90 graus: Hauria de moure l'eix X a l'eix Y
    Matrix3x3 R_Yaw = Matrix3x3::FromEulerZYX(PI / 2, 0, 0);
    Vec3 vX{ 1, 0, 0 };
    Vec3 w_Yaw = R_Yaw.Rotate(vX);
    Vec3 w_Yaw_expected{ 0, 1, 0 };
    S.add(VecEq(w_Yaw, w_Yaw_expected) && !MatEq(R_Yaw, M_ZERO), "FromEulerZYX + Rotate (Yaw 90)", "Eix X -> Eix Y");

    // 2. Pitch (Y) 90 graus: Hauria de moure l'eix Z a l'eix X
    Matrix3x3 R_Pitch = Matrix3x3::FromEulerZYX(0, PI / 2, 0);
    Vec3 vZ{ 0, 0, 1 };
    Vec3 w_Pitch = R_Pitch.Rotate(vZ);
    Vec3 w_Pitch_expected{ 1, 0, 0 };
    S.add(VecEq(w_Pitch, w_Pitch_expected) && !MatEq(R_Pitch, M_ZERO), "FromEulerZYX + Rotate (Pitch 90)", "Eix Z -> Eix X");

    // 3. Roll (X) 90 graus: Hauria de moure l'eix Y a l'eix Z
    Matrix3x3 R_Roll = Matrix3x3::FromEulerZYX(0, 0, PI / 2);
    Vec3 vY{ 0, 1, 0 };
    Vec3 w_Roll = R_Roll.Rotate(vY);
    Vec3 w_Roll_expected{ 0, 0, 1 };
    S.add(VecEq(w_Roll, w_Roll_expected) && !MatEq(R_Roll, M_ZERO), "FromEulerZYX + Rotate (Roll 90)", "Eix Y -> Eix Z");
}

static void EX2_Test_Euler_Roundtrip(Suite& S) {
    std::mt19937 g(13); bool all = true; int N = 120;
    for (int k = 0; k < N; ++k) {
        double yaw = std::uniform_real_distribution<double>(-PI, PI)(g);
        double pitch = std::uniform_real_distribution<double>(-1.4, 1.4)(g); // Evitem gimbal exacte
        double roll = std::uniform_real_distribution<double>(-PI, PI)(g);
        Matrix3x3 R = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
        if (!R.IsRotation() || MatEq(R, M_ZERO, 1e-9)) { all = false; break; }
        double Y, P, Rr; R.ToEulerZYX(Y, P, Rr);
        Matrix3x3 R2 = Matrix3x3::FromEulerZYX(Y, P, Rr);
        if (!MatEq(R, R2, 1e-6)) { all = false; break; }
    }
    S.add(all, "Cicle Euler -> Matriu -> Euler -> Matriu", "N=" + std::to_string(N) + " iteracions (sense gimbal)");
}

static void EX2_Test_Euler_Gimbal(Suite& S) {
    bool all = true; std::vector<double> P = { +PI / 2,-PI / 2 };
    for (double pitch : P) {
        double yaw = 1.0, roll = -0.7;
        Matrix3x3 R = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
        double Y, Pit, Rr; R.ToEulerZYX(Y, Pit, Rr);
        Matrix3x3 R2 = Matrix3x3::FromEulerZYX(Y, Pit, Rr);
        bool is_stub = MatEq(R, M_ZERO, 1e-9);
        bool ok = MatEq(R, R2, 1e-6) && !is_stub;
        std::string name = pitch > 0 ? "Cas Gimbal Lock (+90 deg)" : "Cas Gimbal Lock (-90 deg)";
        S.add(ok, name, ok ? "Reconstruccio correcta" : "Falla reconstruccio");
        all &= ok;
    }
}

// --------------- [Ex3] Quaternions -----------------------
// Prova les funcions de l'Exercici 3.
// Requereix Ex0 i Ex1.
// ---------------------------------------------------------

static void EX3_Test_Quat_FromAxisAngle(Suite& S) {
    Vec3 uZ{ 0,0,1 }; double a = PI / 2;
    Quat q = Quat::FromAxisAngle(uZ, a);
    Quat q_expected{ std::cos(a / 2), 0, 0, std::sin(a / 2) }; // s, x, y, z
    S.add(QuatEq(q, q_expected) && !QuatEq(q, Q_IDENT), "FromAxisAngle(Z, 90deg)", "Comprova valors coneguts");

    Quat q_norm = q.Normalized();
    S.add(Nearly(q_norm.s * q_norm.s + q_norm.x * q_norm.x + q_norm.y * q_norm.y + q_norm.z * q_norm.z, 1.0, 1e-6),
        "Normalized()", "Comprova que la norma^2 == 1");
}

static void EX3_Test_Quat_ToAxisAngle(Suite& S) {
    std::mt19937 g(7); bool all = true; int N = 100;
    for (int k = 0; k < N; ++k) {
        Vec3 u = RandUnit(g); double a = std::uniform_real_distribution<double>(-PI, PI)(g);
        Quat q = Quat::FromAxisAngle(u, a).Normalized();

        // Converteix amb el mètode nou
        Vec3 u2; double a2;
        q.ToAxisAngle(u2, a2);

        // Converteix via matriu com a oracle
        Matrix3x3 R = q.ToMatrix3x3();
        Vec3 ux; double ax; R.ToAxisAngle(ux, ax);

        // Compara rotacions: accepta (u,phi) i (-u,-phi)
        bool same = (VecEq(u2, ux, 1e-5) && Nearly(a2, ax, 1e-5)) ||
            (VecEq(Vec3{ -u2.x,-u2.y,-u2.z }, ux, 1e-5) && Nearly(-a2, ax, 1e-5));
        // Evita stubs
        bool stub = QuatEq(q, Q_IDENT) && std::fabs(a) > 1e-12;
        if (!same || stub) { all = false; break; }
    }
    S.add(all, "Quat::ToAxisAngle coherent amb R.ToAxisAngle", "Random u,a");
}

static void EX3_Test_Quat_Product(Suite& S) {
    std::mt19937 g(21); bool all = true; int N = 80;
    for (int k = 0; k < N; ++k) {
        Vec3 u1 = RandUnit(g), u2 = RandUnit(g); double a1 = 0.7, a2 = -1.1;
        Quat q1 = Quat::FromAxisAngle(u1, a1).Normalized();
        Quat q2 = Quat::FromAxisAngle(u2, a2).Normalized();
        if (QuatEq(q1, Q_IDENT) || QuatEq(q2, Q_IDENT)) { all = false; break; } // Stubs

        Quat q12 = q1.Multiply(q2).Normalized();
        Matrix3x3 R1 = Matrix3x3::RotationAxisAngle(u1, a1);
        Matrix3x3 R2 = Matrix3x3::RotationAxisAngle(u2, a2);
        Matrix3x3 R12 = R1.Multiply(R2);
        Matrix3x3 Rq = q12.ToMatrix3x3();

        bool is_stub_R = MatEq(R1, M_ZERO) || MatEq(R2, M_ZERO) || MatEq(Rq, M_ZERO);
        if (!MatEq(R12, Rq, 1e-6) || is_stub_R) { all = false; break; }
    }
    S.add(all, "Producte Quat (q1*q2) vs Producte Matriu (R1*R2)", "N=" + std::to_string(N) + " iteracions");
}

static void EX3_Test_Quat_Rotate(Suite& S) {
    std::mt19937 g(17); bool all = true; int N = 120;
    for (int k = 0; k < N; ++k) {
        Vec3 u = RandUnit(g); double a = std::uniform_real_distribution<double>(-PI, PI)(g);
        Matrix3x3 R = Matrix3x3::RotationAxisAngle(u, a);
        Quat q = Quat::FromAxisAngle(u, a).Normalized();
        Vec3 v = RandVec(g);
        Vec3 r1_mat = R.Rotate(v);
        Vec3 r2_quat = q.Rotate(v);

        bool is_stub_R = MatEq(R, M_ZERO, 1e-9) && a != 0.0;
        bool is_stub_Q = QuatEq(q, Q_IDENT) && a != 0.0;
        if (!VecEq(r1_mat, r2_quat, 1e-6) || is_stub_R || is_stub_Q) { all = false; break; }
    }
    S.add(all, "Rotacio de Vector (q v q*) vs (R v)", "N=" + std::to_string(N) + " iteracions");
}

static void EX3_Test_Quat_Roundtrips(Suite& S) {
    std::mt19937 g(33); bool all = true; int N = 90;
    for (int k = 0; k < N; ++k) {
        Vec3 u = RandUnit(g); double a = std::uniform_real_distribution<double>(-PI, PI)(g);
        Quat q = Quat::FromAxisAngle(u, a).Normalized();
        Matrix3x3 R = q.ToMatrix3x3();
        Quat q2 = Quat::FromMatrix3x3(R).Normalized();

        bool is_stub_R = MatEq(R, M_ZERO, 1e-9) && a != 0.0;
        if (is_stub_R) { all = false; break; }

        // q i -q representen la mateixa rotació
        if (!QuatEqv(q, q2, 1e-6)) { all = false; break; }
    }
    S.add(all, "Cicle Quat -> Matriu -> Quat", "N=" + std::to_string(N) + " iteracions (compte amb q == -q)");
}

// -------------------- Main -----------------------------
int main() {
    std::cout << BOLD << CYAN << "Test Bench Lab 2" << RESET << "\n";
    int total_suites = 0, suites_ok = 0, total_cases = 0, cases_ok = 0;

    auto RUN_SUITE = [&](Suite& s) {
        total_suites++; total_cases += (int)s.cases.size(); cases_ok += s.passed; bool ok = s.print(); if (ok) suites_ok++; };

    // ---------------- Exercici 0 ----------------
    {
        Suite S("[Ex0] Fonaments Matrix3x3");
        EX0_Test_Multiply_Vec(S);
        EX0_Test_Multiply_Mat(S);
        EX0_Test_Transpose(S);
        EX0_Test_Det_And_Trace(S);
        RUN_SUITE(S);
    }

    // ---------------- Exercici 1 ----------------
    {
        Suite S("[Ex1] Matrius: Axis-Angle");
        EX1_Test_IsRotation(S);
        EX1_Test_RotationAxisAngle(S);
        EX1_Test_Rotate(S);
        EX1_Test_ToAxisAngle(S);
        RUN_SUITE(S);
    }

    // ---------------- Exercici 2 ----------------
    {
        Suite S("[Ex2] Matrius: Angles d'Euler");
        EX2_Test_Euler_Rotate(S);
        EX2_Test_Euler_Roundtrip(S);
        EX2_Test_Euler_Gimbal(S);
        RUN_SUITE(S);
    }

    // ---------------- Exercici 3 ----------------
    {
        Suite S("[Ex3] Quaternions");
        EX3_Test_Quat_Product(S);
        EX3_Test_Quat_Rotate(S);
        EX3_Test_Quat_FromAxisAngle(S);
		EX3_Test_Quat_ToAxisAngle(S);
        EX3_Test_Quat_Roundtrips(S);
        RUN_SUITE(S);
    }

    // ---------------- Summary global ---------------
    std::cout << CYAN << "\n================ SUMMARY ================" << RESET << "\n";
    std::cout << BOLD << ((suites_ok == total_suites) ? GREEN : (suites_ok ? YELL : RED))
        << suites_ok << "/" << total_suites << " suites" << RESET << "  |  "
        << ((cases_ok == total_cases) ? GREEN : (cases_ok ? YELL : RED))
        << cases_ok << "/" << total_cases << " subtests" << RESET << "\n";

    bool all_ok = (suites_ok == total_suites);
    if (!all_ok) std::cout << RED << "Hi ha proves que han FALLAT. Revisa implementacions." << RESET << "\n";
    else        std::cout << GREEN << "Tot OK!" << RESET << "\n";

    return all_ok ? 0 : 1;
}