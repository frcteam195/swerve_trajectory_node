#include <chrono>
#define START_TIMER auto start_time = std::chrono::high_resolution_clock::now();
#define TIME_ELAPSED_US (std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start_time).count() / 1000.0)
#define RESET_TIMER start_time = std::chrono::high_resolution_clock::now();