#include <SDL3/SDL.h>
#include <GL/glew.h>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <random>
#include <cmath>
#include <stdexcept>

// Dear ImGui
#include "imgui.h"
#include "imgui_impl_sdl3.h"
#include "imgui_impl_opengl3.h"

// Project headers (Lab 2)
#include "Matrix3x3.hpp"
#include "Quat.hpp"
#include "OpsCounter.hpp"

static void Check(bool ok, const char* msg) {
    if (!ok) { std::fprintf(stderr, "%s: %s", msg, SDL_GetError()); std::fflush(stderr); std::exit(1); }
}

// ---------------- Helpers numèrics ----------------
static constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
static constexpr double RAD2DEG = 180.0 / 3.14159265358979323846;
static constexpr double TOL = 1e-6;

static bool NearlyEq(double a, double b, double eps = TOL) { return std::fabs(a - b) <= eps; }
static bool VecNearlyEq(const Vec3& a, const Vec3& b, double eps = TOL) {
    return NearlyEq(a.x, b.x, eps) && NearlyEq(a.y, b.y, eps) && NearlyEq(a.z, b.z, eps);
}
static bool MatNearlyEq(const Matrix3x3& A, const Matrix3x3& B, double eps = TOL) {
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) if (!NearlyEq(A.At(i, j), B.At(i, j), eps)) return false; return true;
}

// ---------------- UI State ----------------
static bool show_pan_inputs = true;
static bool show_pan_ops = true;
static bool show_pan_out = true;

// Operacions disponibles (Lab 2 -> enunciat)
// 1) u,phi -> R
// 2) u,phi -> q
// 3) R -> (u,phi)
// 4) R -> q
// 5) R -> (yaw,pitch,roll)
// 6) (yaw,pitch,roll) -> R
// 7) q1*q2
// 8) q -> (u,phi)
// 9) q -> R
// 10) R*v
// 11) q*v
static int op_selected = 0;

// Entrades UI
static Vec3 axis = { 1,0,0 };
static double angle_deg = 45.0; // UI en graus -> intern en rad
static Vec3 vec_v = { 1,2,3 };
static double yaw_deg = 30.0, pitch_deg = 10.0, roll_deg = -20.0; // ZYX, UI en graus
static Matrix3x3 R_user = Matrix3x3::Identity();
static Quat q1 = { 1,0,0,0 };
static Quat q2 = { 1,0,0,0 };

// Resultats UI
static Matrix3x3 R_out = Matrix3x3::Identity();
static Vec3 v_out = { 0,0,0 };
static Quat q_out = { 1,0,0,0 };
static Vec3 axis_out = { 0,0,1 };
static double angle_out_deg = 0.0;
static double yaw_out_deg = 0, pitch_out_deg = 0, roll_out_deg = 0;

// Comptador d'operacions (Lab2: només sumes/multiplicacions)
static OpsCounter last_ops{};

// Estat feedback
enum class Status { None, Ok, Error };
static Status last_status = Status::None;
static char last_msg[256] = "";

//static void SetOk(const char* msg) { last_status = Status::Ok; std::snprintf(last_msg, sizeof(last_msg), "%s", msg); }
static void SetErr(const char* msg) { last_status = Status::Error; std::snprintf(last_msg, sizeof(last_msg), "%s", msg); }

// ---------------- Dibuixadors simples ----------------
static void DrawVec3Edit(const char* label, Vec3& v) {
    ImGui::SeparatorText(label);
    ImGui::PushID(label);            // <<< evita conflictes d'IDs entre instàncies
    ImGui::PushItemWidth(90);
    ImGui::InputDouble("x", &v.x, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("y", &v.y, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("z", &v.z, 0, 0, "%.6f");
    ImGui::PopItemWidth();
    ImGui::PopID();
}

static void DrawMat3Edit(const char* label, Matrix3x3& M, bool editable = true) {
    ImGui::SeparatorText(label);
    if (ImGui::BeginTable(label, 3, ImGuiTableFlags_Borders)) {
        for (int i = 0; i < 3; ++i) {
            ImGui::TableNextRow();
            for (int j = 0; j < 3; ++j) {
                ImGui::TableSetColumnIndex(j);
                double tmp = M.At(i, j);
                if (editable) {
                    ImGui::SetNextItemWidth(90);
                    std::string cell_id = std::string(label) + "[" + std::to_string(i) + "," + std::to_string(j) + "]";
                    if (ImGui::InputDouble(cell_id.c_str(), &tmp, 0, 0, "%.6f")) {
                        M.At(i, j) = tmp;
                    }
                }
                else {
                    ImGui::Text("%.6f", tmp);
                }
            }
        }
        ImGui::EndTable();
    }
}

static void DrawQuatEdit(const char* label, Quat& q) {
    ImGui::SeparatorText(label);
    ImGui::PushID(label);            // <<< evita conflictes d'IDs entre q1 i q2
    ImGui::PushItemWidth(90);
    ImGui::InputDouble("s", &q.s, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("x", &q.x, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("y", &q.y, 0, 0, "%.6f"); ImGui::SameLine();
    ImGui::InputDouble("z", &q.z, 0, 0, "%.6f");
    ImGui::PopItemWidth();
    ImGui::PopID();
}

int main()
{
    // Logs SDL
    SDL_SetLogPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_VERBOSE);

    // SDL init
    Check(SDL_Init(SDL_INIT_VIDEO), "SDL_Init failed");

    // GL attributes & window
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_Window* window = SDL_CreateWindow(
        "Laboratori 2: Rotacions (UI)",
        1280, 720,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
    );
    Check(window != nullptr, "SDL_CreateWindow failed");

    SDL_GLContext gl_ctx = SDL_GL_CreateContext(window);
    Check(gl_ctx != nullptr, "SDL_GL_CreateContext failed");
    SDL_GL_MakeCurrent(window, gl_ctx);
    SDL_GL_SetSwapInterval(1); // vsync

    // GLEW
    glewExperimental = GL_TRUE;
    GLenum glew_err = glewInit();
    if (glew_err != GLEW_OK) { std::fprintf(stderr, "glewInit failed"); return 1; }

    // Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForOpenGL(window, gl_ctx);
    const char* glsl_version = "#version 330";
    ImGui_ImplOpenGL3_Init(glsl_version);

    bool running = true;
    while (running)
    {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);
            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED && e.window.windowID == SDL_GetWindowID(window)) running = false;
        }
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        // ------------- PANELL: ENTRADES -------------
        if (show_pan_inputs && ImGui::Begin("Entrades", &show_pan_inputs)) {
            ImGui::TextWrapped("Introdueix dades per a les operacions del Lab 2. Els angles a la UI son en graus; internament en radians.");
            ImGui::Separator();

            DrawVec3Edit("Eix (u)", axis);
            ImGui::InputDouble("Angle (graus)", &angle_deg, 0, 0, "%.6f");
            if (ImGui::Button("Normalitza eix")) { try { axis = axis.Normalize(); } catch (...) { /*ignore*/ } }
            ImGui::SameLine();
            if (ImGui::Button("Angle = 0")) angle_deg = 0.0;
            ImGui::SameLine();
            if (ImGui::Button("Angle = 180")) angle_deg = 180.0;

            ImGui::Separator();
            ImGui::InputDouble("Yaw (graus, Z)", &yaw_deg, 0, 0, "%.6f");
            ImGui::InputDouble("Pitch (graus, Y)", &pitch_deg, 0, 0, "%.6f");
            ImGui::InputDouble("Roll (graus, X)", &roll_deg, 0, 0, "%.6f");
            if (ImGui::Button("Gimbal lock +90 deg (pitch)")) pitch_deg = 90.0;
            ImGui::SameLine();
            if (ImGui::Button("Gimbal lock -90 deg (pitch)")) pitch_deg = -90.0;

            ImGui::Separator();
            DrawMat3Edit("Matriu R (entrada)", R_user);
            DrawQuatEdit("Quaternion q1", q1);
            DrawQuatEdit("Quaternion q2", q2);
            DrawVec3Edit("Vector v", vec_v);
        }
        if (show_pan_inputs) ImGui::End();

        // ------------- PANELL: OPERACIO -------------
        if (show_pan_ops && ImGui::Begin("Operacio", &show_pan_ops)) {
            const char* ops[] = {
                "1) u,phi -> R", "2) u,phi -> q", "3) R -> (u,phi)", "4) R -> q",
                "5) R -> Euler ZYX", "6) Euler ZYX -> R",
                "7) q1*q2", "8) q -> (u,phi)", "9) q -> R",
                "10) R*v", "11) q*v"
            };
            ImGui::Combo("Operacio", &op_selected, ops, IM_ARRAYSIZE(ops));

            if (ImGui::Button("Run")) {
                last_status = Status::None; last_msg[0] = '\0'; // neteja correctament
                // Reset comptador
                last_ops.Reset();
                try {
                    double phi = angle_deg * DEG2RAD;
                    double yaw = yaw_deg * DEG2RAD, pitch = pitch_deg * DEG2RAD, roll = roll_deg * DEG2RAD;

                    switch (op_selected) {
                    case 0: { // u,phi -> R
                        Vec3 u = axis.Normalize();
                        R_out = Matrix3x3::RotationAxisAngle(u, phi);
                        if (!R_out.IsRotation()) throw std::runtime_error("La matriu retornada no es de rotacio");
                        //SetOk("OK: R = R(u,phi)");
                    } break;
                    case 1: { // u,phi -> q
                        Vec3 u = axis.Normalize();
                        q_out = Quat::FromAxisAngle(u, phi).Normalized();
                        //SetOk("OK: q = q(u,phi)");
                    } break;
                    case 2: { // R -> (u,phi)
                        if (!R_user.IsRotation()) throw std::runtime_error("R d'entrada no es de rotacio");
                        Matrix3x3 Ru = R_user; Vec3 u; double a;
                        Ru.ToAxisAngle(u, a);
                        axis_out = u; angle_out_deg = a * RAD2DEG;
                        //SetOk("OK: (u,phi) extrets de R");
                    } break;
                    case 3: { // R -> q
                        if (!R_user.IsRotation()) throw std::runtime_error("R d'entrada no es de rotacio");
                        q_out = Quat::FromMatrix3x3(R_user).Normalized();
                        //SetOk("OK: q(R)");
                    } break;
                    case 4: { // R -> Euler
                        if (!R_user.IsRotation()) throw std::runtime_error("R d'entrada no es de rotacio");
                        double Y, P, Rr; R_user.ToEulerZYX(Y, P, Rr);
                        yaw_out_deg = Y * RAD2DEG; pitch_out_deg = P * RAD2DEG; roll_out_deg = Rr * RAD2DEG;
                        //SetOk("OK: Euler(R)");
                    } break;
                    case 5: { // Euler -> R
                        R_out = Matrix3x3::FromEulerZYX(yaw, pitch, roll);
                        if (!R_out.IsRotation()) throw std::runtime_error("R(euler) no es de rotacio");
                        //SetOk("OK: R(euler)");
                    } break;
                    case 6: { // q1*q2
                        q_out = q1.Multiply(q2, &last_ops).Normalized();
                        //SetOk("OK: q1*q2");
                    } break;
                    case 7: { // q -> (u,phi)
                        Quat qn = q1.Normalized();
                        Vec3 u; double a; qn.ToAxisAngle(u, a);
                        axis_out = u; angle_out_deg = a * RAD2DEG;
                        //SetOk("OK: (u,phi) de q");
                    } break;
                    case 8: { // q -> R
                        q_out = q1.Normalized();
                        R_out = q_out.ToMatrix3x3();
                        if (!R_out.IsRotation()) throw std::runtime_error("R(q) no es de rotacio");
                        //SetOk("OK: R(q)");
                    } break;
                    case 9: { // R*v
                        if (!R_user.IsRotation()) throw std::runtime_error("R d'entrada no es de rotacio");
                        v_out = R_user.Rotate(vec_v, &last_ops); // instrumentat
                        //SetOk("OK: R*v");
                    } break;
                    case 10: { // q*v
                        v_out = q1.Normalized().Rotate(vec_v, &last_ops); // instrumentat
                        //SetOk("OK: q*v");
                    } break;
                    }
                }
                catch (const std::exception& ex) {
                    SetErr(ex.what());
                }
            }

            ImGui::Separator();
            if (last_status == Status::Ok) {
                ImGui::TextColored(ImVec4(0.2f, 0.8f, 0.2f, 1.0f), "OK: %s", last_msg);
            }
            else if (last_status == Status::Error) {
                ImGui::TextColored(ImVec4(0.95f, 0.25f, 0.2f, 1.0f), "Error: %s", last_msg);
            }
            else {
                ImGui::TextDisabled("Esperant execucio...");
            }
        }
        if (show_pan_ops) ImGui::End();

        // ------------- PANELL: RESULTATS -------------
        if (show_pan_out && ImGui::Begin("Resultats", &show_pan_out))
        {
            ImGui::SeparatorText("Comptador d'operacions (Ex.3)");
            ImGui::Text("Reals (instrumentats): mul=%llu  add=%llu", (unsigned long long)last_ops.mul, (unsigned long long)last_ops.add);

            if (op_selected == 0 || op_selected == 5 || op_selected == 8) {
                DrawMat3Edit("R sortida", R_out, false);
                ImGui::Text("IsRotation: %s", R_out.IsRotation() ? "true" : "false");
            }
            if (op_selected == 2 || op_selected == 7) {
                ImGui::SeparatorText("(u,phi) sortida");
                ImGui::Text("u = (%.6f, %.6f, %.6f)", axis_out.x, axis_out.y, axis_out.z);
                ImGui::Text("phi = %.6f graus", angle_out_deg);
            }
            if (op_selected == 1 || op_selected == 3 || op_selected == 6 || op_selected == 8) {
                ImGui::SeparatorText("q sortida");
                ImGui::Text("(s,x,y,z) = (%.6f, %.6f, %.6f, %.6f)", q_out.s, q_out.x, q_out.y, q_out.z);
            }
            if (op_selected == 4) {
                ImGui::SeparatorText("Euler ZYX sortida");
                ImGui::Text("yaw=%.6f deg, pitch=%.6f deg, roll=%.6f deg", yaw_out_deg, pitch_out_deg, roll_out_deg);
            }
            if (op_selected == 9 || op_selected == 10) {
                ImGui::SeparatorText("Vector rotat");
                ImGui::Text("v' = (%.6f, %.6f, %.6f)", v_out.x, v_out.y, v_out.z);
            }
        }

        if (show_pan_out) ImGui::End();

        // ------------- Render -------------
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }

    // Shutdown ImGui + SDL
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DestroyContext(gl_ctx);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
