#include "App.h"

#include "editor/Editor.h"
#include "render/Renderer.h"
#include "render/VulkanContext.h"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>
#include <ImGuizmo.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>

namespace pegasus {

namespace {

void glfwErrorCallback(int code, const char* desc) { std::fprintf(stderr, "[glfw] %d: %s\n", code, desc); }

void checkVk(VkResult r) {
    if (r != VK_SUCCESS) std::fprintf(stderr, "[imgui-vulkan] VkResult = %d\n", int(r));
}

void dropCallback(GLFWwindow* window, int count, const char** paths) {
    auto* editor = static_cast<Editor*>(glfwGetWindowUserPointer(window));
    for (int i = 0; i < count; ++i) editor->queueFile(paths[i]);
}

void setupStyle(float scale) {
    ImGuiStyle& style = ImGui::GetStyle();
    ImGui::StyleColorsDark();
    style.WindowRounding = 4.0f;
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.TabRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FramePadding = ImVec2(6, 4);
    style.ItemSpacing = ImVec2(8, 6);
    ImVec4* c = style.Colors;
    c[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.12f, 0.13f, 1.00f);
    c[ImGuiCol_TitleBg] = ImVec4(0.09f, 0.10f, 0.11f, 1.00f);
    c[ImGuiCol_TitleBgActive] = ImVec4(0.13f, 0.15f, 0.18f, 1.00f);
    c[ImGuiCol_Header] = ImVec4(0.85f, 0.45f, 0.10f, 0.35f);
    c[ImGuiCol_HeaderHovered] = ImVec4(0.85f, 0.45f, 0.10f, 0.55f);
    c[ImGuiCol_HeaderActive] = ImVec4(0.85f, 0.45f, 0.10f, 0.75f);
    c[ImGuiCol_Button] = ImVec4(0.22f, 0.25f, 0.29f, 1.00f);
    c[ImGuiCol_ButtonHovered] = ImVec4(0.30f, 0.34f, 0.40f, 1.00f);
    c[ImGuiCol_ButtonActive] = ImVec4(0.85f, 0.45f, 0.10f, 0.90f);
    c[ImGuiCol_CheckMark] = ImVec4(1.00f, 0.60f, 0.20f, 1.00f);
    c[ImGuiCol_SliderGrab] = ImVec4(1.00f, 0.60f, 0.20f, 0.80f);
    c[ImGuiCol_SliderGrabActive] = ImVec4(1.00f, 0.65f, 0.25f, 1.00f);
    c[ImGuiCol_FrameBg] = ImVec4(0.17f, 0.19f, 0.22f, 1.00f);
    c[ImGuiCol_Tab] = ImVec4(0.15f, 0.17f, 0.19f, 1.00f);
    c[ImGuiCol_TabSelected] = ImVec4(0.25f, 0.28f, 0.33f, 1.00f);
    c[ImGuiCol_DockingEmptyBg] = ImVec4(0, 0, 0, 0);
    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;
}

void loadFont() {
    ImGuiIO& io = ImGui::GetIO();
    // A TTF font with Turkish glyphs (ş, ğ, ı, İ ...). Falls back to ImGui's built-in font.
    const char* candidates[] = {
        "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu-sans-fonts/DejaVuSans.ttf",
        "C:\\Windows\\Fonts\\segoeui.ttf",
    };
    for (const char* path : candidates) {
        std::error_code ec;
        if (std::filesystem::exists(path, ec) && io.Fonts->AddFontFromFileTTF(path, 16.0f)) return;
    }
    io.Fonts->AddFontDefault();
}

} // namespace

int runApp(int argc, char** argv) {
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit()) {
        std::fprintf(stderr, "GLFW baslatilamadi.\n");
        return 1;
    }
    if (!glfwVulkanSupported()) {
        std::fprintf(stderr, "Bu sistemde Vulkan desteklenmiyor.\n");
        glfwTerminate();
        return 1;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    glfwWindowHint(GLFW_MAXIMIZED, GLFW_TRUE);
    GLFWwindow* window = glfwCreateWindow(1600, 950, "Pegasus - Robot Kol Çalışma Alanı", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }

    const char* validationEnv = std::getenv("PEGASUS_VALIDATION");
    const bool validation = validationEnv && validationEnv[0] == '1';

    VulkanContext ctx;
    Renderer renderer;
    if (!ctx.init(window, validation) || !renderer.init(&ctx)) {
        renderer.shutdown();
        ctx.shutdown();
        glfwDestroyWindow(window);
        glfwTerminate();
        return 1;
    }

    // ---- ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr; // layout is rebuilt on every start
    float xs = 1.0f, ys = 1.0f;
    glfwGetWindowContentScale(window, &xs, &ys);
    int ww, wh, fw, fh;
    glfwGetWindowSize(window, &ww, &wh);
    glfwGetFramebufferSize(window, &fw, &fh);
    const float fbScale = ww > 0 ? float(fw) / float(ww) : 1.0f;
    const float uiScale = std::max(1.0f, xs / std::max(fbScale, 1.0f)); // X11 HiDPI; Wayland scales via the framebuffer
    setupStyle(uiScale);
    loadFont();

    ImGui_ImplGlfw_InitForVulkan(window, true);
    ImGui_ImplVulkan_InitInfo ii{};
    ii.ApiVersion = VK_API_VERSION_1_2;
    ii.Instance = ctx.instance;
    ii.PhysicalDevice = ctx.physicalDevice;
    ii.Device = ctx.device;
    ii.QueueFamily = ctx.queueFamily;
    ii.Queue = ctx.queue;
    ii.DescriptorPoolSize = 64;
    ii.MinImageCount = ctx.minImageCount;
    ii.ImageCount = std::max(ctx.imageCount, ctx.minImageCount);
    ii.PipelineInfoMain.RenderPass = ctx.renderPass;
    ii.PipelineInfoMain.Subpass = 0;
    ii.PipelineInfoMain.MSAASamples = ctx.msaaSamples;
    ii.CheckVkResultFn = checkVk;
    ImGui_ImplVulkan_Init(&ii);

    // ---- Editor
    auto editor = std::make_unique<Editor>();
    glfwSetWindowUserPointer(window, editor.get());
    glfwSetDropCallback(window, dropCallback);
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--robot" && i + 1 < argc) {
            editor->queueRobot(argv[++i]);
        } else if (arg == "--selftest-arduino" && i + 2 < argc) {
            std::filesystem::create_directories(argv[i + 2]);
            editor->enableArduinoSelfTest(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (arg == "--selftest-select" && i + 2 < argc) {
            std::filesystem::create_directories(argv[i + 2]);
            editor->enableSelectSelfTest(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (arg == "--selftest-scan" && i + 2 < argc) {
            std::filesystem::create_directories(argv[i + 2]);
            editor->enableScanSelfTest(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (arg == "--selftest-motion" && i + 2 < argc) {
            std::filesystem::create_directories(argv[i + 2]);
            editor->enableMotionSelfTest(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (arg == "--selftest-robot" && i + 2 < argc) {
            std::filesystem::create_directories(argv[i + 2]);
            editor->enableRobotSelfTest(argv[i + 1], argv[i + 2]);
            i += 2;
        } else if (arg == "--selftest" && i + 1 < argc) {
            std::filesystem::create_directories(argv[i + 1]);
            editor->enableSelfTest(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::printf("Kullanim: pegasus [dosya.stl ...] [--robot <stl-klasoru>] [--selftest <cikti>] [--selftest-robot <stl-klasoru> <cikti>]\n");
            return 0;
        } else {
            editor->queueFile(arg);
        }
    }

    auto last = std::chrono::steady_clock::now();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        if (w == 0 || h == 0 || glfwGetWindowAttrib(window, GLFW_ICONIFIED)) {
            glfwWaitEvents();
            continue;
        }
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - last).count();
        last = now;

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        editor->selfTestInject();
        ImGui::NewFrame();
        ImGuizmo::BeginFrame();

        editor->update(dt);
        if (editor->quitRequested()) glfwSetWindowShouldClose(window, GLFW_TRUE);

        ImGui::Render();
        FrameScene fs;
        editor->buildFrameScene(fs, glm::vec2(io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y));
        const std::string capture = editor->takeCaptureRequest();
        if (!capture.empty()) ctx.requestCapture(capture);
        try {
            renderer.render(fs, ImGui::GetDrawData());
        } catch (const std::exception& e) {
            std::fprintf(stderr, "Render hatasi: %s\n", e.what());
            break;
        }
    }

    vkDeviceWaitIdle(ctx.device);
    editor.reset(); // releases meshes before GPU resources are torn down
    ImGui_ImplVulkan_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    renderer.shutdown();
    ctx.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}

} // namespace pegasus
