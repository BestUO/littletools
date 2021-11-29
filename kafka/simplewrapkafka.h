#pragma once
#include <vector>
#include <rdkafka/rdkafkacpp.h>
#include <memory>
#include <map>
#include <mutex>
#include <iostream>
#include <thread>
#include <singleton.hpp>

class ExampleDeliveryReportCb : public RdKafka::DeliveryReportCb 
{
public:
    void dr_cb (RdKafka::Message &message)
    {
        std::string status_name;
        switch (message.status())
        {
        case RdKafka::Message::MSG_STATUS_NOT_PERSISTED:
            status_name = "NotPersisted";
            break;
        case RdKafka::Message::MSG_STATUS_POSSIBLY_PERSISTED:
            status_name = "PossiblyPersisted";
            break;
        case RdKafka::Message::MSG_STATUS_PERSISTED:
            status_name = "Persisted";
            break;
        default:
            status_name = "Unknown?";
            break;
        }
        std::cout << "Message delivery for (" << message.len() << " bytes): " << status_name << ": " << message.errstr() << std::endl;
        if (message.key())
            std::cout << "Key: " << *(message.key()) << ";" << std::endl;
    }
};

class SimpleWrapKafka : public Singleton<SimpleWrapKafka>
{
public:
    SimpleWrapKafka() = default;
    ~SimpleWrapKafka();

    void CreateTopic(std::string brokers, std::string topic);
    bool CreateProducer(std::string brokers, std::string topic);

    template <class Function>
    bool CreateConsumer(std::string brokers, std::string topic, std::string groupid, Function f)
    {
        std::string errstr;
        int batch_size = 16;
        std::vector<std::string> topics(1,topic);
        RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
        if (conf->set("enable.partition.eof", "false", errstr) != RdKafka::Conf::CONF_OK) 
            return false;
        if (conf->set("group.id", groupid, errstr) != RdKafka::Conf::CONF_OK)
            return false;
        if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) 
            return false;
        auto consumer = std::shared_ptr<RdKafka::KafkaConsumer>(RdKafka::KafkaConsumer::create(conf, errstr));
        if (!consumer) 
            return false;
        RdKafka::ErrorCode err = consumer->subscribe(topics);
        if (err)
            return false;
        delete conf;

        _consumerthreads.emplace_back(std::thread([this, consumer = std::move(consumer), batch_size, f=std::move(f)]()
        {
            ConsumerAddThread<>(consumer, batch_size, f);
        }));
        return true;
    }

    void Add2Kafka(std::string topic, std::string message);
    void StopConsumer();
    
private:
    bool _stopconsumers = false;
    std::map<std::string, std::shared_ptr<RdKafka::Producer>> _producers;
    std::mutex _mutex;
    ExampleDeliveryReportCb _ex_dr_cb;

    std::vector<std::thread> _consumerthreads;

    template <class Function>
    void ConsumerAddThread(std::shared_ptr<RdKafka::KafkaConsumer> consumerptr, int batch_size, Function f)
    {
        while (!_stopconsumers)
        {
            auto msgs = consume_batch(consumerptr, batch_size);
            std::cout << "Accumulated " << msgs.size() << " messages:" << std::endl;
            for (auto &msg : msgs) 
            {
                std::cout << " Message in " << msg->topic_name() << " [" << msg->partition() << "] at offset " << msg->offset() << std::endl;
                
                if(msg->err() == RdKafka::ERR_NO_ERROR)
                {
                    f((char*)msg->payload());
                }
                else
                    std::cout << "err" << msg->err() << std::endl;
    
                delete msg;
            }
        }

        consumerptr->close();
    }
    std::vector<RdKafka::Message *> consume_batch(std::shared_ptr<RdKafka::KafkaConsumer> consumerptr, size_t batch_size);
};