#pragma once

#include <string>

namespace pendulum {

/**
 * @brief Placeholder for the refactored open-loop simulation.
 *
 * This class will encapsulate the logic formerly in main.cpp.
 * For now it provides a minimal run() interface so that CMake can
 * configure the project.
 */
class OpenLoopSimulation {
public:
    void run();
    void saveResults(const std::string& filename) const;
};

} // namespace pendulum
