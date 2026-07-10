#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {
    bool IsBlankLine(const std::string &line) {
        return std::all_of(line.begin(), line.end(), [](unsigned char ch) {
            return std::isspace(ch) != 0;
        });
    }

    bool HasCodeExtension(const fs::path &path) {
        const auto ext = path.extension().string();
        return ext == ".cpp" || ext == ".hpp";
    }
} // namespace

int main(int argc, char *argv[]) {
    const fs::path workspace_root = fs::current_path();
    const fs::path src_root = workspace_root / "src";
    fs::path output_path = workspace_root / "Result-GenCodeDoc.cpp";

    if (argc >= 2) {
        output_path = fs::path(argv[1]);
        if (output_path.extension().empty()) {
            output_path += ".cpp";
        }
    }

    if (!fs::exists(src_root) || !fs::is_directory(src_root)) {
        std::cerr << "src directory not found: " << src_root.string() << "\n";
        return 1;
    }

    std::vector<fs::path> files;
    for (const auto &entry : fs::recursive_directory_iterator(src_root)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        if (HasCodeExtension(entry.path())) {
            files.push_back(entry.path());
        }
    }

    std::sort(files.begin(), files.end());

    std::ofstream out(output_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        std::cerr << "failed to open output file: " << output_path.string() << "\n";
        return 1;
    }

    for (const auto &file_path : files) {
        const fs::path rel = fs::relative(file_path, src_root);

        out << "// ========================================\n";
        out << "//  " << rel.generic_string() << "\n";
        out << "// ========================================\n";

        std::ifstream in(file_path, std::ios::in);
        if (!in.is_open()) {
            std::cerr << "failed to read file: " << file_path.string() << "\n";
            return 1;
        }

        std::string line;
        while (std::getline(in, line)) {
            if (IsBlankLine(line)) {
                continue;
            }
            out << line << "\n";
        }

        out << "\n";
    }

    return 0;
}
