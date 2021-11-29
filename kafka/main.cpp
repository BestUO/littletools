#include <iostream>
#include "rdkafkacpp.h"
#include <chrono>
#include <thread>
#include "simplewrapkafka.h"

void produce(std::string brokers, std::string topic)
{
    std::string errstr;
    RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
    if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }
    ExampleDeliveryReportCb ex_dr_cb;
    if (conf->set("dr_cb", &ex_dr_cb, errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }

    RdKafka::Producer *producer = RdKafka::Producer::create(conf, errstr);
    if (!producer) 
    {
        std::cerr << "Failed to create producer: " << errstr << std::endl;
        exit(1);
    }
    std::cout << "% Created producer " << producer->name() << std::endl;
    delete conf;

    std::string message;
    retry:
    for(int i = 0;i<2;i++)
    {
        if(i==0)
            message = "192.168.10.51_styletransfer_wct_filepath";
        else
            message = "192.168.10.51_styletransfer_wct_filepath2";
        RdKafka::ErrorCode err =
        producer->produce(
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
                /* If the internal queue is full, wait for
                * messages to be delivered and then retry.
                * The internal queue represents both
                * messages to be sent and messages that have
                * been sent or failed, awaiting their
                * delivery report callback to be called.
                *
                * The internal queue is limited by the
                * configuration property
                * queue.buffering.max.messages */
                producer->poll(1000/*block for max 1000ms*/);
                goto retry;
            }
        } 
        else 
        {
        std::cerr << "% Enqueued message (" << message.size() << " bytes) " <<
            "for topic " << topic << std::endl;
        }

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
    }


    while(producer->outq_len() > 0)
    {
        std::cerr << "Waiting for " << producer->outq_len() << std::endl;
        producer->poll(1000);
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
    delete producer;
}

static std::vector<RdKafka::Message *> consume_batch(RdKafka::KafkaConsumer *consumer, size_t batch_size) 
{
    std::vector<RdKafka::Message *> msgs;
    msgs.reserve(batch_size);

    while (msgs.size() < batch_size) 
    {
        RdKafka::Message *msg = consumer->consume(1000);
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

void msg_consume(RdKafka::Message* message, void* opaque) {
  const RdKafka::Headers *headers;

  switch (message->err()) {
    case RdKafka::ERR__TIMED_OUT:
      break;

    case RdKafka::ERR_NO_ERROR:
      /* Real message */
      std::cout << "Read msg at offset " << message->offset() << std::endl;
      if (message->key()) {
        std::cout << "Key: " << *message->key() << std::endl;
      }
      headers = message->headers();
      if (headers) {
        std::vector<RdKafka::Headers::Header> hdrs = headers->get_all();
        for (size_t i = 0 ; i < hdrs.size() ; i++) {
          const RdKafka::Headers::Header hdr = hdrs[i];

          if (hdr.value() != NULL)
            printf(" Header: %s = \"%.*s\"\n",
                   hdr.key().c_str(),
                   (int)hdr.value_size(), (const char *)hdr.value());
          else
            printf(" Header:  %s = NULL\n", hdr.key().c_str());
        }
      }
      std::cout << (char*)message->payload() << std::endl;
    //   printf("%.*s\n",
    //     static_cast<int>(message->len()),
    //     static_cast<const char *>(message->payload()));
      break;

    case RdKafka::ERR__PARTITION_EOF:
      /* Last message */
      std::cout << "last message" << std::endl;
      break;

    case RdKafka::ERR__UNKNOWN_TOPIC:
    case RdKafka::ERR__UNKNOWN_PARTITION:
      std::cerr << "Consume failed: " << message->errstr() << std::endl;
      break;

    default:
      /* Errors */
      std::cerr << "Consume failed: " << message->errstr() << std::endl;
  }
}

void consumer(std::string brokers, std::string topic)
{
    std::string errstr;
    int batch_size = 16;
    std::vector<std::string> topics(1,topic);
    RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
    if (conf->set("enable.partition.eof", "true", errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }
    if (conf->set("group.id", "g1", errstr) != RdKafka::Conf::CONF_OK)
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }
    if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }
    RdKafka::KafkaConsumer *consumer = RdKafka::KafkaConsumer::create(conf, errstr);
    if (!consumer) 
    {
        std::cerr << "Failed to create consumer: " << errstr << std::endl;
        exit(1);
    }
    RdKafka::ErrorCode err = consumer->subscribe(topics);
    if (err)
    {
        std::cerr << "Failed to subscribe to " << topics.size() << " topics: "
                << RdKafka::err2str(err) << std::endl;
        exit(1);
    }
    delete conf;

    while (true) 
    {
        auto msgs = consume_batch(consumer, batch_size);
        std::cout << "Accumulated " << msgs.size() << " messages:" << std::endl;
        for (auto &msg : msgs) 
        {
            std::cout << " Message in " << msg->topic_name() << " [" << msg->partition() << "] at offset " << msg->offset() << std::endl;
            std::cout << msg->payload() << std::endl;
            msg_consume(msg, NULL);
            delete msg;
        }
    }

    consumer->close();
    delete consumer;
}

void createtopic(std::string brokers, std::string topic)
{
    std::string errstr;
    RdKafka::Conf *conf = RdKafka::Conf::create(RdKafka::Conf::CONF_GLOBAL);
    RdKafka::Conf *tconf = RdKafka::Conf::create(RdKafka::Conf::CONF_TOPIC);
    // tconf->set(name+strlen("topic."), "mytest", errstr);
    if (conf->set("bootstrap.servers", brokers, errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }
    ExampleDeliveryReportCb ex_dr_cb;
    if (conf->set("dr_cb", &ex_dr_cb, errstr) != RdKafka::Conf::CONF_OK) 
    {
        std::cerr << errstr << std::endl;
        exit(1);
    }

    RdKafka::Producer *producer = RdKafka::Producer::create(conf, errstr);
    if (!producer) 
    {
        std::cerr << "Failed to create producer: " << errstr << std::endl;
        exit(1);
    }
    std::cout << "% Created producer " << producer->name() << std::endl;

    auto topicptr = RdKafka::Topic::create(producer, topic, tconf, errstr);
    if (!topicptr) 
    {
        std::cerr << "Failed to create topic: " << errstr << std::endl;
        exit(1);
    }
    std::cout << "% Created topic " << topicptr->name() << std::endl;

    delete conf;
    delete producer;
}

template<typename T, typename... AP>
void set_http_handler(std::string name, T &&f, AP &&... ap) 
{
		auto lm = [name, f = std::move(f)](auto... ap) 
		{
			
		};
}

int main()
{
    set_http_handler<>("/upload_multipart", []() 
    {
        std::cout << "aa" << std::endl;
    });

    std::string errstr;
    std::string brokers("192.168.10.52:9092");
    std::string topic("mytest");
    std::string groupid("g1");

    // createtopic(brokers, topic);
    // produce(brokers, topic);
    // consumer(brokers, topic);

    SimpleWrapKafka::inst().CreateProducer(brokers,topic);
    SimpleWrapKafka::inst().Add2Kafka("mytest","wrap");

    SimpleWrapKafka::inst().CreateConsumer<>(brokers, topic, groupid, [](std::string name)
    {
        std::cout << name << std::endl;
    });

    std::this_thread::sleep_for(std::chrono::seconds(10));
    SimpleWrapKafka::inst().StopConsumer();
}