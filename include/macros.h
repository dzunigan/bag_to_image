#ifndef MACROS_HPP_
#define MACROS_HPP_

// STL
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define AT __FILE__ ":" TOSTRING(__LINE__)
#define RUNTIME_ASSERT(x) { if (!(x)) throw std::runtime_error("Runtime assertion failed (at " AT "):\n" TOSTRING(x)); }

#define DO_PRAGMA(x) _Pragma (#x)
#define TODO(x) DO_PRAGMA(message ("TODO - " x))

#define USE(x) (void)(x);

#define COUT_LOG(x) { std::cout << std::string("[ ") + __PRETTY_FUNCTION__ + std::string(" ] ") + TOSTRING(x) + std::string(": ") << (x) << std::endl; }

#endif // MACROS_HPP_
