#include "utils/csv_logger.hpp"

#include <iomanip>
#include <iostream>

namespace pendulum {

CsvLogger::CsvLogger(const std::string& filename) : file_(filename) {
    if (!file_.is_open()) {
        std::cerr << "[CsvLogger] Failed to open: " << filename << std::endl;
    }
    file_ << std::fixed << std::setprecision(6);
}

CsvLogger::~CsvLogger() {
    if (file_.is_open()) {
        file_.close();
    }
}

void CsvLogger::writeHeader(const std::vector<std::string>& columns) {
    if (!file_.is_open()) return;
    for (size_t i = 0; i < columns.size(); ++i) {
        if (i > 0) file_ << ",";
        file_ << columns[i];
    }
    file_ << "\n";
}

void CsvLogger::writeRow(const std::vector<double>& values) {
    if (!file_.is_open()) return;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) file_ << ",";
        file_ << values[i];
    }
    file_ << "\n";
}

void CsvLogger::flush() {
    if (file_.is_open()) {
        file_.flush();
    }
}

bool CsvLogger::isOpen() const {
    return file_.is_open();
}

} // namespace pendulum
