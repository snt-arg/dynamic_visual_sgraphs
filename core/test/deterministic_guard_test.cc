// P9-B unit test: with System.Deterministic set, starting any thread throws.
//   usage: deterministic_guard_test <settings.yaml with System.Deterministic: 1> <vocab> <sys_params.yaml>
// Needs no ASLR/env set-up: the viewer guard fires before those checks (System::ConfigureDeterministicMode).
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "Deterministic.h"
#include "System.h"

static int failures = 0;
#define CHECK(cond, msg)                                       \
    do                                                         \
    {                                                          \
        if (!(cond))                                           \
        {                                                      \
            std::cerr << "FAIL: " << msg << std::endl;         \
            ++failures;                                        \
        }                                                      \
        else                                                   \
            std::cout << "ok:   " << msg << std::endl;         \
    } while (0)

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "usage: " << argv[0] << " <settings.yaml> <vocab> <sys_params.yaml>" << std::endl;
        return 2;
    }

    // 1. the guard itself
    ORB_SLAM3::Deterministic::Set(false);
    bool threw = false;
    try { ORB_SLAM3::Deterministic::ThrowIfThreadStart("t"); } catch (const std::runtime_error &) { threw = true; }
    CHECK(!threw, "guard is a no-op when the flag is off");

    ORB_SLAM3::Deterministic::Set(true);
    threw = false;
    try { ORB_SLAM3::Deterministic::ThrowIfThreadStart("t"); } catch (const std::runtime_error &) { threw = true; }
    CHECK(threw, "guard throws when the flag is on");
    ORB_SLAM3::Deterministic::Set(false);

    // 2. the only configurable thread (viewer) with the flag set in the settings file
    threw = false;
    std::string what;
    try
    {
        ORB_SLAM3::System slam(argv[2], argv[1], argv[3], ORB_SLAM3::System::MONOCULAR, /*bUseViewer=*/true);
    }
    catch (const std::runtime_error &e)
    {
        threw = true;
        what = e.what();
    }
    CHECK(threw && what.find("Viewer") != std::string::npos, "System(bUseViewer=true) with System.Deterministic throws (" + what + ")");
    CHECK(ORB_SLAM3::Deterministic::Enabled(), "flag stays set after the failed construction");

    std::cout << (failures ? "FAILED" : "ALL PASSED") << std::endl;
    return failures ? 1 : 0;
}
