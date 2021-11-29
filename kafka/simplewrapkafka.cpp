#include "simplewrapkafka.h"

SimpleWrapKafka::~SimpleWrapKafka()
{
    for(auto &thread : _consumerthreads)
        thread.join();
    std::cout << "end SimpleWrapKafka" << std::endl;
}

void SimpleWrapKafka::CreateTopic(std::string brokers, std::string topic)
{}

bool SimpleWrapKafka::CreateProducer(std::string brokers, std::string topic)
{
    std::string errstr;
    RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
    if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) 
        return false;
    if (conf->set("dr_cb", &_ex_dr_cb, errstr) != RdKafka::Conf::CONF_OK) 
        return false;

    auto producer = std::shared_ptr<RdKafka::Producer>(RdKafka::Producer::create(conf, errstr));
    if (!producer) 
        return false;
    std::cout << "Created producer " << producer->name() << std::endl;
    delete conf;

    std::unique_lock<std::mutex> lck(_mutex);
    _producers.emplace(topic,producer);
    return true;
}

void SimpleWrapKafka::Add2Kafka(std::string topic, std::string message)
{
    std::shared_ptr<RdKafka::Producer> producer = nullptr;
    {
        std::unique_lock<std::mutex> lck(_mutex);
        producer = _producers.find(topic)->second;
    }
    while(true)
    {
        RdKafka::ErrorCode err = producer->produce(
                    topic,
                    RdKafka::Topic::PARTITION_UA,
                    /* Make a copy of the value */
                    RdKafka::Producer::RK_MSG_COPY /* Copy payload */,
                    /* Value */
                    const_cast<char *>(message.c_str()), message.size(),
                    /* Key */
                    NULL, 0,
                    /* Timestamp (defaults to current time) */
                    0,
                    /* Message headers, if any */
                    NULL,
                    /* Per-message opaque value passed to
                        * delivery report */
                    NULL);

        if (err != RdKafka::ERR_NO_ERROR) 
        {
            std::cerr << "% Failed to produce to topic " << topic << ": " << RdKafka::err2str(err) << std::endl;
            if (err == RdKafka::ERR__QUEUE_FULL) 
            {
                producer->poll(1000/*block for max 1000ms*/);
                continue;
            }
        } 
        else 
            std::cerr << "% Enqueued message (" << message.size() << " bytes) " << "for topic " << topic << std::endl;

        /* A producer application should continually serve
        * the delivery report queue by calling poll()
        * at frequent intervals.
        * Either put the poll call in your main loop, or in a
        * dedicated thread, or call it after every produce() call.
        * Just make sure that poll() is still called
        * during periods where you are not producing any messages
        * to make sure previously produced messages have their
        * delivery report callback served (and any other callbacks
        * you register). */
        producer->poll(0);
        break;
    }

    while(producer->outq_len() > 0)
    {
        std::cerr << "Waiting for " << producer->outq_len() << std::endl;
        producer->poll(1000);
    }
}

void SimpleWrapKafka::StopConsumer()
{
    _stopconsumers = true;
}

std::vector<RdKafka::Message *> SimpleWrapKafka::consume_batch(std::shared_ptr<RdKafka::KafkaConsumer> consumerptr, size_t batch_size) 
{
    std::vector<RdKafka::Message *> msgs;
    msgs.reserve(batch_size);

    while (msgs.size() < batch_size) 
    {
        RdKafka::Message *msg = consumerptr->consume(1000);
        switch (msg->err()) 
        {
        case RdKafka::ERR__TIMED_OUT:
            delete msg;
            return msgs;
        case RdKafka::ERR_NO_ERROR:
            msgs.push_back(msg);
            break;
        default:
            std::cerr << "%% Consumer error: " << msg->errstr() << std::endl;
            delete msg;
            return msgs;
        }
    }
    return msgs;
}