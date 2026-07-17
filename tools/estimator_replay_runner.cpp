#include "estimation/ReplayRunner.hpp"

#include <fstream>
#include <iostream>

namespace {

void print_usage() {
    std::cerr << "Usage: estimator_replay_runner --input <path> --output <path> "
                 "[--mode active_only|active_with_identical_shadow]\n";
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    drone::estimation::ReplayMode mode = drone::estimation::ReplayMode::ACTIVE_ONLY;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            output_path = argv[++i];
        } else if (arg == "--mode" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "active_only") {
                mode = drone::estimation::ReplayMode::ACTIVE_ONLY;
            } else if (value == "active_with_identical_shadow") {
                mode = drone::estimation::ReplayMode::ACTIVE_WITH_IDENTICAL_SHADOW;
            } else {
                std::cerr << "Unsupported mode: " << value << '\n';
                return 2;
            }
        } else {
            print_usage();
            return 2;
        }
    }

    if (input_path.empty() || output_path.empty()) {
        print_usage();
        return 2;
    }

    drone::estimation::ReplayRunConfig config;
    config.mode = mode;
    const auto result = drone::estimation::run_replay_file(input_path, config);

    std::ofstream output(output_path, std::ios::trunc);
    if (!output.is_open()) {
        std::cerr << "Could not open output file: " << output_path.string() << '\n';
        return 3;
    }
    output << drone::estimation::replay_report_json(result.report);
    output.close();
    return result.exit_code;
}
