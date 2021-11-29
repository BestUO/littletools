#ifndef SINGLETON_HPP
#define SINGLETON_HPP

class NonCopyable
{
protected:
    NonCopyable() = default;
    ~NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    NonCopyable& operator=(const NonCopyable& c) = delete;
};

template <class T>
class Singleton : public NonCopyable 
{
// private:
//     static T* inst_;

public:
    Singleton() {}
    virtual ~Singleton() {}

    // static T& inst()
    // {
    //     if (!inst_) inst_ = new T;
    //     return *inst_;
    // }

    // static void uninst()
    // {
    //     if (!inst_) return;
    //     delete inst_;
    //     inst_ = nullptr;
    // }

    static T& inst()
    {
      static T instance;
      return instance;
    }
};
// template<class T> T* Singleton<T>::inst_ = nullptr;
#endif