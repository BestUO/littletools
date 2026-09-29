#include "iox2/iceoryx2.hpp"
#include "transmission_data.hpp"

#include <cstdint>
#include <iostream>
#include <utility>

namespace
{
constexpr iox2::bb::Duration CYCLE_TIME = iox2::bb::Duration::from_secs(1);
constexpr const char* SERVICE_NAME      = "littletools/PublishSubscribe";
}  // namespace

auto main() -> int
{
    using namespace iox2;

    set_log_level_from_env_or(LogLevel::Info);
    auto node = NodeBuilder().create<ServiceType::Ipc>().value();
    auto service
        = node.service_builder(ServiceName::create(SERVICE_NAME).value())
              .publish_subscribe<TransmissionData>()
              .open_or_create()
              .value();
    auto publisher = service.publisher_builder().create().value();

    std::int32_t counter = 0;
    while (node.wait(CYCLE_TIME).has_value())
    {
        ++counter;
        auto sample             = publisher.loan_uninit().value();
        auto initialized_sample = sample.write_payload(
            TransmissionData{counter, counter * 3, counter * 812.12});
        send(std::move(initialized_sample)).value();
        std::cout << "sent sample " << counter << std::endl;
    }

    std::cout << "publisher stopped" << std::endl;
    return 0;
}
