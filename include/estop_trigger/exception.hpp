#include <stdexcept>

namespace estop {
    class Exception : public std::runtime_error {
    public:
        Exception(const std::string& what) : runtime_error(what) {}
    };
} // namespace estop