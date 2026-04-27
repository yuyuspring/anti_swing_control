#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace pendulum {

/**
 * @brief Simple CSV writer for simulation logs.
 */
class CsvLogger {
public:
    explicit CsvLogger(const std::string& filename);
    ~CsvLogger();

    /**
     * @brief Write a header row.
     */
    void writeHeader(const std::vector<std::string>& columns);

    /**
     * @brief Write a single data row from a vector of doubles.
     */
    void writeRow(const std::vector<double>& values);

    /**
     * @brief Flush the underlying stream.
     */
    void flush();

    bool isOpen() const;

private:
    std::ofstream file_;
};

} // namespace pendulum
