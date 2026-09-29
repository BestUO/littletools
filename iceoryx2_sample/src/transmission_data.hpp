#ifndef LITTLETOOLS_ICEORYX2_SAMPLE_TRANSMISSION_DATA_HPP
#define LITTLETOOLS_ICEORYX2_SAMPLE_TRANSMISSION_DATA_HPP

#include <cstdint>
#include <ostream>

struct TransmissionData
{
    // Keep the C++ payload type name stable for iceoryx2 type matching.
    static constexpr const char* IOX2_TYPE_NAME = "TransmissionData";

    std::int32_t x;
    std::int32_t y;
    double value;
};

inline auto operator<<(std::ostream& stream,
    const TransmissionData& data) -> std::ostream&
{
    return stream << "TransmissionData { x: " << data.x << ", y: " << data.y
                  << ", value: " << data.value << " }";
}

#endif
