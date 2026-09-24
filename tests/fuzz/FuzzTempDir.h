#pragma once

// A directory of the fuzz target's own for readers that take a path: made
// once, under the system's temporary directory, and removed when the process
// ends.

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <system_error>

namespace hz::fuzz {

class TempDir {
public:
    explicit TempDir(const std::string& prefix) {
        std::random_device random;
        const uint64_t tag = (static_cast<uint64_t>(random()) << 32) ^ random();
        m_path = std::filesystem::temp_directory_path() / (prefix + "_" + std::to_string(tag));
        std::filesystem::create_directories(m_path);
    }
    ~TempDir() {
        std::error_code ignored;  // best effort: a leftover temp directory harms nothing
        std::filesystem::remove_all(m_path, ignored);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;

    const std::filesystem::path& path() const { return m_path; }

    /// Write @p size bytes at @p data to @p name inside the directory, and
    /// return its path.
    std::filesystem::path write(const std::string& name, const uint8_t* data, size_t size) const {
        const auto file = m_path / name;
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(size));
        return file;
    }

private:
    std::filesystem::path m_path;
};

}  // namespace hz::fuzz
