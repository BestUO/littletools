#include "iox2/iceoryx2.hpp"
#include "transmission_data.hpp"

#include <iostream>

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
    auto subscriber = service.subscriber_builder().create().value();

    std::cout << "subscriber ready" << std::endl;
    while (node.wait(CYCLE_TIME).has_value())
    {
        auto sample = subscriber.receive().value();
        while (sample.has_value())
        {
            std::cout << "received: " << sample->payload() << std::endl;
            sample = subscriber.receive().value();
        }
    }

    std::cout << "subscriber stopped" << std::endl;
    return 0;
}
