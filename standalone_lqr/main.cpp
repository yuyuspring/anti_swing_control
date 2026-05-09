#include "standalone_lqr.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void printUsage(const char* prog) {
    std::cout <<
"Usage: " << prog << " [OPTIONS]\n"
"\n"
"Standalone LQR closed-loop simulator for 1D pitch slung-load.\n"
"Self-contained: no external headers or libraries required.\n"
"\n"
"Options:\n"
"  --csv <path>         Input CSV with v_ref sequence (1 or 2 columns)\n"
"  --output <path>      Output CSV path (default: stdout)\n"
"  --dt <sec>           Discrete step [s] (default: 0.02)\n"
"  --rope <m>           Rope length [m] (default: 10.0)\n"
"  --mode <name>        Controller mode: diagonal | coupled (default: coupled)\n"
"  --accel <m/s2>       Trapezoid acceleration rate (default: 2.0)\n"
"  --cruise <m/s>       Trapezoid cruise speed (default: 15.0)\n"
"  --brake <sec>        Trapezoid brake start time (default: 40.0)\n"
"  --t-final <sec>      Trapezoid total duration (default: 60.0)\n"
"  --theta0 <rad>       Initial pendulum angle [rad] (default: 0.0)\n"
"  --help               Show this help\n"
"\n"
"Input CSV formats (auto-detected):\n"
"  1 column : v_ref values only (time = 0, dt, 2*dt, ...)\n"
"  2 columns: time,v_ref (time must start at 0 and be uniform)\n"
"\n"
"Output CSV columns: time,p,v,a,theta,omega\n"
"\n"
"Examples:\n"
"  " << prog << "\n"
"  " << prog << " --csv vref.csv --output result.csv --dt 0.01\n"
"  " << prog << " --mode diagonal --rope 15.0 --t-final 120.0\n";
}

struct Args {
    std::string csvInput;
    std::string outputPath;
    double dt = 0.02;
    double ropeLength = 10.0;
    lqr::ControllerMode mode = lqr::ControllerMode::kCoupled;
    double accelRate = 2.0;
    double cruiseSpeed = 15.0;
    double brakeStartTime = 40.0;
    double tFinal = 60.0;
    double theta0 = 0.0;
};

Args parseArgs(int argc, char* argv[]) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--csv" && i + 1 < argc) {
            args.csvInput = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            args.outputPath = argv[++i];
        } else if (arg == "--dt" && i + 1 < argc) {
            args.dt = std::stod(argv[++i]);
        } else if (arg == "--rope" && i + 1 < argc) {
            args.ropeLength = std::stod(argv[++i]);
        } else if (arg == "--mode" && i + 1 < argc) {
            std::string m = argv[++i];
            if (m == "diagonal" || m == "Diagonal") {
                args.mode = lqr::ControllerMode::kDiagonal;
            } else if (m == "coupled" || m == "Coupled") {
                args.mode = lqr::ControllerMode::kCoupled;
            } else {
                std::cerr << "[Error] Unknown mode: " << m << "\n";
                std::exit(1);
            }
        } else if (arg == "--accel" && i + 1 < argc) {
            args.accelRate = std::stod(argv[++i]);
        } else if (arg == "--cruise" && i + 1 < argc) {
            args.cruiseSpeed = std::stod(argv[++i]);
        } else if (arg == "--brake" && i + 1 < argc) {
            args.brakeStartTime = std::stod(argv[++i]);
        } else if (arg == "--t-final" && i + 1 < argc) {
            args.tFinal = std::stod(argv[++i]);
        } else if (arg == "--theta0" && i + 1 < argc) {
            args.theta0 = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "[Error] Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            std::exit(1);
        }
    }
    return args;
}

std::vector<double> loadVRefFromCSV(const std::string& path, double& out_dt,
                                    bool& hasTimeColumn) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Error] Cannot open: " << path << "\n";
        std::exit(1);
    }

    std::string line;
    if (!std::getline(f, line)) {
        std::cerr << "[Error] Empty CSV: " << path << "\n";
        std::exit(1);
    }

    // Parse header to detect column count
    std::istringstream hdrStream(line);
    std::vector<std::string> headers;
    std::string cell;
    while (std::getline(hdrStream, cell, ',')) {
        // trim whitespace
        cell.erase(0, cell.find_first_not_of(" \t\r\n"));
        cell.erase(cell.find_last_not_of(" \t\r\n") + 1);
        headers.push_back(cell);
    }

    size_t numCols = headers.size();
    if (numCols < 1) {
        std::cerr << "[Error] CSV has no columns\n";
        std::exit(1);
    }

    // Find v_ref column index; fall back to vx_truth if v_ref not found
    hasTimeColumn = false;
    size_t vrefCol = 0;
    if (numCols >= 2) {
        std::string first = headers[0];
        std::transform(first.begin(), first.end(), first.begin(), ::tolower);
        if (first.find("time") != std::string::npos) {
            hasTimeColumn = true;
        }
    }
    // Search for v_ref_m_s or vx_truth_m_s column
    for (size_t i = 0; i < headers.size(); ++i) {
        std::string h = headers[i];
        std::transform(h.begin(), h.end(), h.begin(), ::tolower);
        if (h.find("v_ref") != std::string::npos) {
            vrefCol = i;
            break;
        }
        if (h.find("vx_truth") != std::string::npos && vrefCol == 0) {
            vrefCol = i;
        }
    }

    std::vector<double> times;
    std::vector<double> vrefs;

    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::vector<double> row;
        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
        if (row.size() <= vrefCol) continue;
        if (hasTimeColumn) times.push_back(row[0]);
        vrefs.push_back(row[vrefCol]);
    }

    if (vrefs.empty()) {
        std::cerr << "[Error] No data rows in CSV\n";
        std::exit(1);
    }

    if (hasTimeColumn && times.size() >= 2) {
        out_dt = times[1] - times[0];
        // Verify uniform
        for (size_t i = 2; i < times.size(); ++i) {
            double d = times[i] - times[i - 1];
            if (std::abs(d - out_dt) > 1e-9) {
                std::cerr << "[Warn] Non-uniform time step detected. Using average dt.\n";
                out_dt = (times.back() - times.front()) / (times.size() - 1);
                break;
            }
        }
    } else {
        // No time column: user must provide --dt
        if (out_dt <= 0) {
            std::cerr << "[Error] CSV has no time column. Please specify --dt.\n";
            std::exit(1);
        }
    }

    return vrefs;
}

void writeCSV(std::ostream& out,
              double dt,
              const std::vector<double>& p,
              const std::vector<double>& v,
              const std::vector<double>& a,
              const std::vector<double>& theta,
              const std::vector<double>& omega) {
    out << "time,p,v,a,theta,omega\n";
    out << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < p.size(); ++i) {
        out << (i * dt) << ","
            << p[i] << ","
            << v[i] << ","
            << a[i] << ","
            << theta[i] << ","
            << omega[i] << "\n";
    }
}

int main(int argc, char* argv[]) {
    Args args = parseArgs(argc, argv);

    std::vector<double> vRefSeq;
    double dt = args.dt;

    if (!args.csvInput.empty()) {
        bool hasTime = false;
        vRefSeq = loadVRefFromCSV(args.csvInput, dt, hasTime);
        std::cout << "[Input] Loaded " << vRefSeq.size()
                  << " samples from " << args.csvInput
                  << " (dt=" << dt << "s)\n";
    } else {
        vRefSeq = lqr::StandaloneLqrSimulator::generateTrapezoid(
            dt, args.tFinal, args.accelRate, args.cruiseSpeed, args.brakeStartTime);
        std::cout << "[Input] Trapezoid profile: accel=" << args.accelRate
                  << " cruise=" << args.cruiseSpeed
                  << " brake=" << args.brakeStartTime
                  << " T=" << args.tFinal
                  << " (N=" << vRefSeq.size() << ", dt=" << dt << "s)\n";
    }

    lqr::StandaloneLqrSimulator::Config simConfig;
    simConfig.dt = dt;
    simConfig.ropeLength = args.ropeLength;
    simConfig.mode = args.mode;
    simConfig.initialTheta = args.theta0;

    lqr::StandaloneLqrSimulator sim(simConfig);

    std::vector<double> p, v, a, theta, omega;
    sim.run(vRefSeq, p, v, a, theta, omega);

    if (!args.outputPath.empty()) {
        std::ofstream out(args.outputPath);
        if (!out.is_open()) {
            std::cerr << "[Error] Cannot write: " << args.outputPath << "\n";
            return 1;
        }
        writeCSV(out, dt, p, v, a, theta, omega);
        std::cout << "[Output] Saved to " << args.outputPath << "\n";
    } else {
        writeCSV(std::cout, dt, p, v, a, theta, omega);
    }

    return 0;
}
