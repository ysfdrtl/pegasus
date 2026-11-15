#pragma once
// Vulkan device / swapchain / frame management.

#include <vulkan/vulkan.h>

#include <functional>
#include <string>
#include <vector>

struct GLFWwindow;

namespace pegasus {

struct Buffer {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkDeviceSize size = 0;
    void* mapped = nullptr;
};

struct Image {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
};

class VulkanContext {
public:
    static constexpr uint32_t kFramesInFlight = 2;

    bool init(GLFWwindow* window, bool enableValidation);
    void shutdown();

    // Returns VK_NULL_HANDLE if the frame should be skipped (minimized / swapchain recreated).
    VkCommandBuffer beginFrame();
    // Begins the main render pass on the current frame's command buffer.
    void beginRenderPass(VkCommandBuffer cmd, const float clearColor[4]);
    void endFrame(); // ends render pass, submits and presents

    uint32_t frameIndex() const { return frameIndex_; }
    // Saves the next presented frame as PNG (if the swapchain supports transfer).
    void requestCapture(const std::string& path) { capturePath_ = path; }

    Buffer createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags props);
    Buffer createDeviceLocalBuffer(const void* data, VkDeviceSize size, VkBufferUsageFlags usage);
    void destroyBuffer(Buffer& b);
    void immediateSubmit(const std::function<void(VkCommandBuffer)>& fn);
    VkShaderModule loadShader(const std::string& path);

    VkInstance instance = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue queue = VK_NULL_HANDLE;
    uint32_t queueFamily = 0;
    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;
    VkExtent2D extent{};
    uint32_t minImageCount = 2;
    uint32_t imageCount = 0;
    VkPhysicalDeviceProperties gpuProps{};
    bool wideLines = false;

private:
    bool createInstance(bool validation);
    bool pickPhysicalDevice();
    bool createDevice();
    bool createSwapchain();
    void destroySwapchain();
    void recreateSwapchain();
    bool createRenderPass();
    Image createImage(VkFormat format, VkImageUsageFlags usage, VkImageAspectFlags aspect, VkSampleCountFlagBits samples);
    void destroyImage(Image& img);
    uint32_t findMemoryType(uint32_t bits, VkMemoryPropertyFlags props) const;

    GLFWwindow* window_ = nullptr;
    VkDebugUtilsMessengerEXT messenger_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
    VkPhysicalDeviceMemoryProperties memProps_{};
    VkSurfaceFormatKHR surfaceFormat_{};
    VkFormat depthFormat_ = VK_FORMAT_D32_SFLOAT;
    VkPresentModeKHR presentMode_ = VK_PRESENT_MODE_FIFO_KHR;
    VkCommandPool commandPool_ = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    std::vector<VkImage> swapImages_;
    std::vector<VkImageView> swapViews_;
    std::vector<VkFramebuffer> framebuffers_;
    std::vector<VkSemaphore> renderFinished_; // one per swapchain image
    Image colorMsaa_, depth_;

    struct Frame {
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        VkSemaphore imageAvailable = VK_NULL_HANDLE;
        VkFence fence = VK_NULL_HANDLE;
    } frames_[kFramesInFlight];
    uint32_t frameIndex_ = 0;
    uint32_t imageIndex_ = 0;
    bool canCapture_ = false;
    std::string capturePath_;
    Buffer captureBuffer_;

};

} // namespace pegasus
