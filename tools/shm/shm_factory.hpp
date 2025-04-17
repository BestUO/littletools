#include <cstddef>
#include <mutex>
#include <string>
#include <memory>
#include <fcntl.h>
#include <sys/mman.h>
#include <map>

template <typename T>
class SHMFactory
{
public:
    T* AllocateSHM(const std::string& name, size_t size)
    {
        bool is_create = true;
        auto fd = shm_open(name.c_str(), O_CREAT | O_RDWR | O_EXCL, 0666);
        if (fd >= 0)
        {
            if (ftruncate(fd, size) == -1)
                perror("ftruncate failed");
        }
        else if (errno == EEXIST)
        {
            fd        = shm_open(name.c_str(), O_RDWR, 0666);
            is_create = false;
        }

        auto ptr = (T*)mmap(
            NULL, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (is_create)
        {
            ptr = new (ptr) T();
        }
        else
        {
            ptr = reinterpret_cast<T*>(ptr);
        }

        std::lock_guard<std::mutex> lock(__mutex);
        __shm_map.push_back(std::make_pair(name, std::make_pair(fd, ptr)));
    }
    void FreeSHM(std::string&& name)
    {
        std::lock_guard<std::mutex> lock(__mutex);
        if (auto it = __shm_map.find(name); it != __shm_map.end())
        {
            close(it->second.first);
            munmap(it->second.second, sizeof(T));
            shm_unlink(it->first.c_str());
            __shm_map.erase(it);
        }
    }

    ~SHMFactory()
    {
        std::lock_guard<std::mutex> lock(__mutex);
        for (const auto& [name, pair] : __shm_map)
        {
            close(pair.first);
            munmap(pair.second, sizeof(T));
            shm_unlink(name.c_str());
        }
    }

private:
    std::map<std::string, std::pair<int, T*>> __shm_map;
    std::mutex __mutex;
};