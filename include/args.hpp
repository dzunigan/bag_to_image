// Header only extension of gflags to handle command line arguments

// STL
#include <iostream>
#include <string>

// GFlags
#include <gflags/gflags.h>

#define FLAG_CASE(type, name, val, txt) \
    DEFINE_##type(name, val, txt);

FLAGS_CASES

#undef FLAG_CASE

#define ARG_VARIABLE(name) \
    ARGS_##name

#define DEFINE_ARG(name) \
    std::string ARG_VARIABLE(name)

#define ARG_CASE(name) \
    DEFINE_ARG(name);

ARGS_CASES

#undef ARG_CASE

namespace args {

using namespace gflags;

bool HelpRequired(int argc, char* argv[]) {

    const std::string help_flag("--help");
    for (int i = 1; i < argc; ++i)
        if (help_flag.compare(argv[i]) == 0) return true;
    return false;
}

void ShowHelp() {

    std::cout << "Usage: " << PROGRAM_NAME << " [options]";
#define ARG_CASE(arg) \
    std::cout << " <" << #arg << ">";

    ARGS_CASES

#undef ARG_CASE
    std::cout << std::endl;

    std::cout << std::endl;
    std::cout << "Options:" << std::endl;

#define FLAG_CASE(type, name, val, txt)                                                            \
    std::cout << "  --" << #name << ": " << txt << std::endl;                                      \
    std::cout << "     " << "(type: " << #type << ", default: " << #val << ")" << std::endl;

    FLAGS_CASES

#undef FLAG_CASE

    std::cout << "  --help: Displays this message" << std::endl;
    std::cout << std::endl;
}

constexpr int NumArgs() {
#define ARG_CASE(name) +1
    return ARGS_CASES+0;
#undef ARG_CASE
}

void ParseCommandLineArgs(int argc, char* argv[]) noexcept {

    int i = 0;

#define ARG_CASE(name) \
    if (++i < argc) ARG_VARIABLE(name) = std::string(argv[i]);

    ARGS_CASES

#undef ARG_CASE
}

}
