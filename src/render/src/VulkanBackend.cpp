#ifdef HZ_ENABLE_VULKAN

#include "horizon/render/VulkanBackend.h"

#include <vulkan/vulkan.h>

#include <cstring>
#include <vector>

namespace hz::render {

VulkanBackend::VulkanBackend() {
    // -- Instance ------------------------------------------------------------
    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Horizon CAD";
    appInfo.apiVersion = VK_API_VERSION_1_1;

    VkInstanceCreateInfo instanceInfo{};
    instanceInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instanceInfo.pApplicationInfo = &appInfo;

    if (vkCreateInstance(&instanceInfo, nullptr, &m_instance) != VK_SUCCESS) {
        m_instance = nullptr;
        return;
    }

    // -- Physical device: prefer a discrete GPU -------------------------------
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);
    if (deviceCount == 0) return;
    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

    for (VkPhysicalDevice candidate : devices) {
        VkPhysicalDeviceProperties props{};
        vkGetPhysicalDeviceProperties(candidate, &props);
        if (m_physicalDevice == nullptr ||
            props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            m_physicalDevice = candidate;
            m_deviceName = props.deviceName;
            m_apiVersion = props.apiVersion;
            if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) break;
        }
    }
    if (m_physicalDevice == nullptr) return;

    // -- Graphics queue family -------------------------------------------------
    uint32_t familyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &familyCount, nullptr);
    std::vector<VkQueueFamilyProperties> families(familyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(m_physicalDevice, &familyCount, families.data());
    uint32_t graphicsFamily = UINT32_MAX;
    for (uint32_t i = 0; i < familyCount; ++i) {
        if ((families[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
            graphicsFamily = i;
            break;
        }
    }
    if (graphicsFamily == UINT32_MAX) return;

    // -- Logical device ---------------------------------------------------------
    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queueInfo{};
    queueInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    queueInfo.queueFamilyIndex = graphicsFamily;
    queueInfo.queueCount = 1;
    queueInfo.pQueuePriorities = &priority;

    VkDeviceCreateInfo deviceInfo{};
    deviceInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    deviceInfo.queueCreateInfoCount = 1;
    deviceInfo.pQueueCreateInfos = &queueInfo;

    if (vkCreateDevice(m_physicalDevice, &deviceInfo, nullptr, &m_device) != VK_SUCCESS) {
        m_device = nullptr;
        return;
    }

    // -- Memory type table: selection happens per allocation ---------------------
    VkPhysicalDeviceMemoryProperties memProps{};
    vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProps);
    m_memoryTypeFlags.reserve(memProps.memoryTypeCount);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        m_memoryTypeFlags.push_back(memProps.memoryTypes[i].propertyFlags);
    }
}

VulkanBackend::~VulkanBackend() {
    if (m_device != nullptr) {
        vkDeviceWaitIdle(m_device);
        for (auto& [id, rec] : m_buffers) {
            vkDestroyBuffer(m_device, rec.buffer, nullptr);
            vkFreeMemory(m_device, rec.memory, nullptr);
        }
        vkDestroyDevice(m_device, nullptr);
    }
    if (m_instance != nullptr) {
        vkDestroyInstance(m_instance, nullptr);
    }
}

std::string VulkanBackend::name() const {
    if (!isAvailable()) return "Vulkan (unavailable)";
    return "Vulkan " + std::to_string(VK_VERSION_MAJOR(m_apiVersion)) + "." +
           std::to_string(VK_VERSION_MINOR(m_apiVersion)) + " — " + m_deviceName;
}

BufferHandle VulkanBackend::createBuffer(BufferUsage usage, const void* data, size_t size) {
    if (!isAvailable() || size == 0) return {};

    VkBufferUsageFlags usageFlags = 0;
    switch (usage) {
        case BufferUsage::Vertex:
            usageFlags = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
            break;
        case BufferUsage::Index:
            usageFlags = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
            break;
        case BufferUsage::Uniform:
            usageFlags = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
            break;
    }

    VkBufferCreateInfo bufferInfo{};
    bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bufferInfo.size = size;
    bufferInfo.usage = usageFlags;
    bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    BufferRecord rec;
    rec.size = size;
    if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &rec.buffer) != VK_SUCCESS) return {};

    VkMemoryRequirements memReq{};
    vkGetBufferMemoryRequirements(m_device, rec.buffer, &memReq);

    // The bound memory type must be one the buffer allows
    // (VUID-vkBindBufferMemory-memory-01035): pick the first type that is in
    // memoryTypeBits AND host-visible + host-coherent.
    const VkMemoryPropertyFlags wanted =
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
    uint32_t typeIndex = UINT32_MAX;
    for (uint32_t i = 0; i < m_memoryTypeFlags.size(); ++i) {
        if ((memReq.memoryTypeBits & (1u << i)) != 0 && (m_memoryTypeFlags[i] & wanted) == wanted) {
            typeIndex = i;
            break;
        }
    }
    if (typeIndex == UINT32_MAX) {
        vkDestroyBuffer(m_device, rec.buffer, nullptr);
        return {};
    }

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize = memReq.size;
    allocInfo.memoryTypeIndex = typeIndex;
    if (vkAllocateMemory(m_device, &allocInfo, nullptr, &rec.memory) != VK_SUCCESS ||
        vkBindBufferMemory(m_device, rec.buffer, rec.memory, 0) != VK_SUCCESS) {
        vkDestroyBuffer(m_device, rec.buffer, nullptr);
        if (rec.memory != nullptr) vkFreeMemory(m_device, rec.memory, nullptr);
        return {};
    }

    const BufferHandle handle{m_nextHandle++};
    m_buffers.emplace(handle.id, rec);
    if (data != nullptr) updateBuffer(handle, data, size);
    return handle;
}

void VulkanBackend::updateBuffer(BufferHandle handle, const void* data, size_t size) {
    auto it = m_buffers.find(handle.id);
    if (it == m_buffers.end() || data == nullptr || size > it->second.size) return;
    void* mapped = nullptr;
    if (vkMapMemory(m_device, it->second.memory, 0, size, 0, &mapped) != VK_SUCCESS) return;
    std::memcpy(mapped, data, size);
    vkUnmapMemory(m_device, it->second.memory);
}

void VulkanBackend::destroyBuffer(BufferHandle handle) {
    auto it = m_buffers.find(handle.id);
    if (it == m_buffers.end()) return;
    vkDestroyBuffer(m_device, it->second.buffer, nullptr);
    vkFreeMemory(m_device, it->second.memory, nullptr);
    m_buffers.erase(it);
}

// -- Staged features: honest no-ops until the SPIR-V pipeline lands ----------

TextureHandle VulkanBackend::createTexture(int, int, const void*) {
    return {};
}
void VulkanBackend::destroyTexture(TextureHandle) {}
ShaderHandle VulkanBackend::createShader(const std::string&, const std::string&) {
    return {};
}
void VulkanBackend::destroyShader(ShaderHandle) {}
void VulkanBackend::beginPass(const RenderPassDesc&) {}
void VulkanBackend::draw(const DrawCall&) {}
void VulkanBackend::endPass() {}

}  // namespace hz::render

#endif  // HZ_ENABLE_VULKAN
