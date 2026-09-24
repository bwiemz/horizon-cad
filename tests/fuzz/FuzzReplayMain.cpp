// Replays a fuzz target over files, for builds without libFuzzer.
//
// Every argument is a file or a directory of files; each file is fed to
// LLVMFuzzerTestOneInput once. Registered with CTest over the seed corpus, so
// every seed — and every crasher committed to the corpus — runs in every
// build, including the sanitizer job.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size);

namespace {

int runFile(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot read %s\n", path.string().c_str());
        return 1;
    }
    const std::vector<char> bytes((std::istreambuf_iterator<char>(in)),
                                  std::istreambuf_iterator<char>());
    LLVMFuzzerTestOneInput(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    int failures = 0;
    int inputs = 0;
    for (int i = 1; i < argc; ++i) {
        const std::filesystem::path arg(argv[i]);
        if (std::filesystem::is_directory(arg)) {
            for (const auto& entry : std::filesystem::recursive_directory_iterator(arg)) {
                if (!entry.is_regular_file()) continue;
                failures += runFile(entry.path());
                ++inputs;
            }
        } else {
            failures += runFile(arg);
            ++inputs;
        }
    }
    std::printf("replayed %d input(s)\n", inputs);
    return failures == 0 && inputs > 0 ? 0 : 1;
}
