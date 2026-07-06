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
    m_queueFamily = graphicsFamily;
    vkGetDeviceQueue(m_device, graphicsFamily, 0, &m_queue);

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
        case BufferUsage::Storage:
            usageFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
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

bool VulkanBackend::runComputeSpirv(const uint32_t* spirv, size_t wordCount,
                                    const std::vector<BufferHandle>& storageBuffers,
                                    uint32_t groupsX, uint32_t groupsY, uint32_t groupsZ) {
    if (!isAvailable() || m_queue == nullptr || spirv == nullptr || wordCount == 0 ||
        storageBuffers.empty()) {
        return false;
    }

    std::vector<VkBuffer> buffers;
    buffers.reserve(storageBuffers.size());
    for (const BufferHandle& h : storageBuffers) {
        auto it = m_buffers.find(h.id);
        if (it == m_buffers.end()) return false;
        buffers.push_back(it->second.buffer);
    }

    // One-shot pipeline: everything created here is destroyed before return.
    // Dispatch caching is a follow-up once compute becomes per-frame work.
    bool ok = false;
    VkShaderModule shader = VK_NULL_HANDLE;
    VkDescriptorSetLayout setLayout = VK_NULL_HANDLE;
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline pipeline = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    VkFence fence = VK_NULL_HANDLE;

    do {
        VkShaderModuleCreateInfo shaderInfo{};
        shaderInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        shaderInfo.codeSize = wordCount * sizeof(uint32_t);
        shaderInfo.pCode = spirv;
        if (vkCreateShaderModule(m_device, &shaderInfo, nullptr, &shader) != VK_SUCCESS) break;

        std::vector<VkDescriptorSetLayoutBinding> bindings(storageBuffers.size());
        for (uint32_t i = 0; i < bindings.size(); ++i) {
            bindings[i].binding = i;
            bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
        layoutInfo.pBindings = bindings.data();
        if (vkCreateDescriptorSetLayout(m_device, &layoutInfo, nullptr, &setLayout) != VK_SUCCESS) {
            break;
        }

        VkPipelineLayoutCreateInfo plInfo{};
        plInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        plInfo.setLayoutCount = 1;
        plInfo.pSetLayouts = &setLayout;
        if (vkCreatePipelineLayout(m_device, &plInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
            break;
        }

        VkComputePipelineCreateInfo pipeInfo{};
        pipeInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
        pipeInfo.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        pipeInfo.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        pipeInfo.stage.module = shader;
        pipeInfo.stage.pName = "main";
        pipeInfo.layout = pipelineLayout;
        if (vkCreateComputePipelines(m_device, VK_NULL_HANDLE, 1, &pipeInfo, nullptr, &pipeline) !=
            VK_SUCCESS) {
            break;
        }

        VkDescriptorPoolSize poolSize{};
        poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        poolSize.descriptorCount = static_cast<uint32_t>(storageBuffers.size());
        VkDescriptorPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.maxSets = 1;
        poolInfo.poolSizeCount = 1;
        poolInfo.pPoolSizes = &poolSize;
        if (vkCreateDescriptorPool(m_device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
            break;
        }

        VkDescriptorSetAllocateInfo setAlloc{};
        setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        setAlloc.descriptorPool = descriptorPool;
        setAlloc.descriptorSetCount = 1;
        setAlloc.pSetLayouts = &setLayout;
        VkDescriptorSet descriptorSet = VK_NULL_HANDLE;
        if (vkAllocateDescriptorSets(m_device, &setAlloc, &descriptorSet) != VK_SUCCESS) break;

        std::vector<VkDescriptorBufferInfo> bufferInfos(buffers.size());
        std::vector<VkWriteDescriptorSet> writes(buffers.size());
        for (uint32_t i = 0; i < buffers.size(); ++i) {
            bufferInfos[i].buffer = buffers[i];
            bufferInfos[i].offset = 0;
            bufferInfos[i].range = VK_WHOLE_SIZE;
            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
            writes[i].dstSet = descriptorSet;
            writes[i].dstBinding = i;
            writes[i].descriptorCount = 1;
            writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            writes[i].pBufferInfo = &bufferInfos[i];
        }
        vkUpdateDescriptorSets(m_device, static_cast<uint32_t>(writes.size()), writes.data(), 0,
                               nullptr);

        VkCommandPoolCreateInfo cpInfo{};
        cpInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        cpInfo.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
        cpInfo.queueFamilyIndex = m_queueFamily;
        if (vkCreateCommandPool(m_device, &cpInfo, nullptr, &commandPool) != VK_SUCCESS) break;

        VkCommandBufferAllocateInfo cbInfo{};
        cbInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        cbInfo.commandPool = commandPool;
        cbInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbInfo.commandBufferCount = 1;
        VkCommandBuffer cmd = VK_NULL_HANDLE;
        if (vkAllocateCommandBuffers(m_device, &cbInfo, &cmd) != VK_SUCCESS) break;

        VkCommandBufferBeginInfo begin{};
        begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        if (vkBeginCommandBuffer(cmd, &begin) != VK_SUCCESS) break;
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1,
                                &descriptorSet, 0, nullptr);
        vkCmdDispatch(cmd, groupsX, groupsY, groupsZ);

        // Make shader writes visible to the host read-back.
        VkMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
        barrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_HOST_BIT,
                             0, 1, &barrier, 0, nullptr, 0, nullptr);
        if (vkEndCommandBuffer(cmd) != VK_SUCCESS) break;

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        if (vkCreateFence(m_device, &fenceInfo, nullptr, &fence) != VK_SUCCESS) break;

        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        if (vkQueueSubmit(m_queue, 1, &submit, fence) != VK_SUCCESS) break;
        if (vkWaitForFences(m_device, 1, &fence, VK_TRUE, UINT64_MAX) != VK_SUCCESS) break;

        ok = true;
    } while (false);

    if (fence != VK_NULL_HANDLE) vkDestroyFence(m_device, fence, nullptr);
    if (commandPool != VK_NULL_HANDLE) vkDestroyCommandPool(m_device, commandPool, nullptr);
    if (descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_device, descriptorPool, nullptr);
    }
    if (pipeline != VK_NULL_HANDLE) vkDestroyPipeline(m_device, pipeline, nullptr);
    if (pipelineLayout != VK_NULL_HANDLE) {
        vkDestroyPipelineLayout(m_device, pipelineLayout, nullptr);
    }
    if (setLayout != VK_NULL_HANDLE) vkDestroyDescriptorSetLayout(m_device, setLayout, nullptr);
    if (shader != VK_NULL_HANDLE) vkDestroyShaderModule(m_device, shader, nullptr);
    return ok;
}

bool VulkanBackend::readBuffer(BufferHandle handle, void* out, size_t size) {
    auto it = m_buffers.find(handle.id);
    if (it == m_buffers.end() || out == nullptr || size > it->second.size) return false;
    void* mapped = nullptr;
    if (vkMapMemory(m_device, it->second.memory, 0, size, 0, &mapped) != VK_SUCCESS) return false;
    std::memcpy(out, mapped, size);
    vkUnmapMemory(m_device, it->second.memory);
    return true;
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
