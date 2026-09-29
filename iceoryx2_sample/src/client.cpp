#include "iox2/iceoryx2.hpp"
#include "transmission_data.hpp"

#include <cstdint>
#include <iostream>
#include <utility>

namespace
{
constexpr iox2::bb::Duration CYCLE_TIME = iox2::bb::Duration::from_secs(1);
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
    auto client = service.client_builder().create().value();

    std::uint64_t request_counter  = 0;
    std::uint64_t response_counter = 0;

    std::cout << "send request " << request_counter << std::endl;
    auto pending_response = client.send_copy(request_counter).value();

    while (node.wait(CYCLE_TIME).has_value())
    {
        while (true)
        {
            auto response = pending_response.receive().value();
            if (!response.has_value())
            {
                break;
            }
            std::cout << "received response " << response_counter++ << ": "
                      << response->payload() << std::endl;
        }

        ++request_counter;
        auto request = client.loan_uninit().value();
        auto initialized_request
            = request.write_payload(std::uint64_t{request_counter});
        pending_response = send(std::move(initialized_request)).value();
        std::cout << "send request " << request_counter << std::endl;
    }

    std::cout << "client stopped" << std::endl;
    return 0;
}
