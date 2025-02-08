#include <iostream>
#include <unistd.h>
#include "doctest/doctest.h"
#include "nanobench.h"
#include "tools/concurrentqueue/concurrentqueue.h"
#include "tools/concurrentqueue/blockingconcurrentqueue.h"

TEST_CASE("concurrentqueue_base")
{
    moodycamel::ConcurrentQueue<int> q;
    q.enqueue(1);
    q.enqueue(2);
    q.enqueue(3);
    int item;
    q.try_dequeue(item);
    CHECK(item == 1);
    q.try_dequeue(item);
    CHECK(item == 2);
    q.try_dequeue(item);
    CHECK(item == 3);
}

TEST_CASE("concurrentqueue_bulk")
{
    moodycamel::ConcurrentQueue<int> q;
    q.enqueue(1);
    q.enqueue(2);
    q.enqueue(3);
    int item[3];
    q.try_dequeue_bulk(item, 3);
    CHECK(item[0] == 1);
    CHECK(item[1] == 2);
    CHECK(item[2] == 3);
}

TEST_CASE("concurrentqueue_token")
{
    moodycamel::ConcurrentQueue<int> q;
    moodycamel::ProducerToken token1(q);
    moodycamel::ProducerToken token2(q);
    moodycamel::ProducerToken token3(q);
    q.try_enqueue(token1, 1);
    q.try_enqueue(token2, 2);
    q.try_enqueue(token3, 3);
    int item;
    q.try_dequeue_from_producer(token1, item);
    CHECK(item == 1);
    q.try_dequeue_from_producer(token2, item);
    CHECK(item == 2);
    q.try_dequeue_from_producer(token3, item);
    CHECK(item == 3);
}

TEST_CASE("concurrentqueue_block")
{
    moodycamel::BlockingConcurrentQueue<int> q;
    q.enqueue(1);
    q.enqueue(2);
    q.enqueue(3);
    int item;
    q.wait_dequeue(item);
    CHECK(item == 1);
    q.wait_dequeue(item);
    CHECK(item == 2);
    q.wait_dequeue(item);
    CHECK(item == 3);
}