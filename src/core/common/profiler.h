#ifndef PROFILER_H
#define PROFILER_H

#include <chrono>
#include <functional>
#include <iostream> // For std::cout, replace with proper logger later
#include <string>
#include <utility> // For std::forward, std::is_void_v

namespace core {
namespace common {

template <typename Func, typename... Args>
auto timed_call(const std::string& name, Func func, Args&&... args) -> decltype(func(std::forward<Args>(args)...)) {
    auto start_time = std::chrono::high_resolution_clock::now();

    if constexpr (std::is_void_v<decltype(func(std::forward<Args>(args)...))>) {
        func(std::forward<Args>(args)...);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // Replace with proper logger later
        std::cout << name << " took " << duration.count() << " microseconds." << std::endl;
        // Explicitly return for void functions if required by a specific coding style,
        // though it's not strictly necessary here as the function signature is void.
    } else {
        auto result = func(std::forward<Args>(args)...);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        // Replace with proper logger later
        std::cout << name << " took " << duration.count() << " microseconds." << std::endl;
        return result;
    }
}

} // namespace common
} // namespace core

#endif // PROFILER_H
