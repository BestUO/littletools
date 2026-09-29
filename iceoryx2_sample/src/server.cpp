#include "iox2/iceoryx2.hpp"
#include "transmission_data.hpp"

#include <cstdint>
#include <iostream>
#include <utility>

namespace
{
constexpr iox2::bb::Duration CYCLE_TIME = iox2::bb::Duration::from_millis(100);
constexpr const char* SERVICE_NAME      = "littletools/RequestResponse";
}  // namespace

auto main() -> int
{
    using namespace iox2;

    set_log_level_from_env_or(LogLevel::Info);
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    auto service
        = node.service_builder(ServiceName::create(SERVICE_NAME).value())
              .request_response<std::uint64_t, TransmissionData>()
              .open_or_create()
              .value();
    auto server = service.server_builder().create().value();

    std::cout << "server ready" << std::endl;
    std::int32_t counter = 0;

    while (node.wait(CYCLE_TIME).has_value())
    {
        while (true)
        {
            auto active_request = server.receive().value();
            if (!active_request.has_value())
            {
                break;
            }

            std::cout << "received request: " << active_request->payload()
                      << std::endl;

            const TransmissionData copied_response{
                5 + counter, 6 * counter, 7.77};
            active_request->send_copy(copied_response).value();
            std::cout << "sent response: " << copied_response << std::endl;

            // Odd-numbered requests receive a second, zero-copy response to
            // demonstrate iceoryx2's streaming response API.
            for (std::uint64_t i = 0; i < active_request->payload() % 2; ++i)
            {
                auto response = active_request->loan_uninit().value();
                auto initialized_response
                    = response.write_payload(TransmissionData{
                        counter * static_cast<std::int32_t>(i + 1),
                        counter + static_cast<std::int32_t>(i),
                        counter * 0.1234,
                    });
                std::cout << "sent response: " << initialized_response.payload()
                          << std::endl;
                send(std::move(initialized_response)).value();
            }
        }
        ++counter;
    }

    std::cout << "server stopped" << std::endl;
    return 0;
}
