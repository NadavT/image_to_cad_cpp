#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <iostream>
#include <string>

#define TOKEN_PASTE(x, y) x##y
#define CAT(x, y) TOKEN_PASTE(x, y)

#define TIMED_FUNCTION_WITH_PREFIX(function, name, prefix)                                                             \
    std::cout << prefix << name << std::endl;                                                                          \
    auto CAT(__start, __LINE__) = std::chrono::high_resolution_clock::now();                                           \
    function;                                                                                                          \
    auto CAT(__end, __LINE__) = std::chrono::high_resolution_clock::now();                                             \
    std::cout << prefix << "Finished " << name                                                                         \
              << " in: " << std::chrono::duration<double>(CAT(__end, __LINE__) - CAT(__start, __LINE__)).count()       \
              << " seconds" << std::endl;

#define TIMED_FUNCTION(function, name) TIMED_FUNCTION_WITH_PREFIX(function, name, "")

#define TIMED_INNER_FUNCTION(function, name) TIMED_FUNCTION_WITH_PREFIX(function, name, "\t")

#endif /* UTILS_H */
