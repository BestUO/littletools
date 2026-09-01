#include <thread>
#include <vector>
#include <csignal>

struct LeakObject
{
    std::string s;
};

void TestLeak1(std::vector<LeakObject>& v)
{
    v.emplace_back(LeakObject{"aaaaaaaaaaaaaaaaaaaaa"});
}
void TestLeak2(std::vector<LeakObject>& v)
{
    v.emplace_back(LeakObject{"a"});
}
volatile bool g_shotdown = false;
int main()
{
    std::signal(SIGINT, [](int) {
        g_shotdown = true;
    });
    std::vector<LeakObject> v;
    while (!g_shotdown)
    {
        TestLeak1(v);
        TestLeak2(v);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
