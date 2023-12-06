#pragma once

#include <string>

#ifdef _WIN32
    #ifdef _WIN32_WINNT
        #undef _WIN32_WINNT
    #endif
    #define _WIN32_WINNT 0x0600
#endif

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <iphlpapi.h>
    #include <windows.h>
#else
    #include <netinet/in.h>
    #include <sys/poll.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <ifaddrs.h>
    #include <sys/types.h>
    #include <net/if.h>
    #include <sys/un.h>
    #include <dlfcn.h>
#endif

#ifdef _WIN32
    #ifdef ERROR
        #undef ERROR
    #endif

    #ifdef SYNCHRONIZE
        #undef SYNCHRONIZE
    #endif

    #define UNIX_PATH_MAX 108
typedef struct sockaddr_un
{
    ADDRESS_FAMILY sun_family;
    char sun_path[UNIX_PATH_MAX];
} SOCKADDR_UN, *PSOCKADDR_UN;

    #define SIO_AF_UNIX_GETPEERPID _WSAIOR(IOC_VENDOR, 256)
    #define SIO_AF_UNIX_SETBINDPARENTPATH _WSAIOW(IOC_VENDOR, 257)
    #define SIO_AF_UNIX_SETCONNPARENTPATH _WSAIOW(IOC_VENDOR, 258)

    // win32 only for compile success, not used
    #define RTLD_NOW 0x0001
    #define RTLD_GLOBAL 0x0002
#endif

#ifdef _WIN32
    #define POLL WSAPoll
#else
    #define POLL poll
#endif

inline void* DLOPEN(const char* libraryName, int flags)
{
#ifdef _WIN32
    return LoadLibrary(libraryName);
#else
    return dlopen(libraryName, flags);
#endif
}

inline void* DLSYM(void* libraryHandle, const char* functionName)
{
#ifdef _WIN32
    return (void*)GetProcAddress(
        static_cast<HMODULE>(libraryHandle), functionName);
#else
    return dlsym(libraryHandle, functionName);
#endif
}

inline void DLCLOSE(void* libraryHandle)
{
#ifdef _WIN32
    FreeLibrary(static_cast<HMODULE>(libraryHandle));
#else
    dlclose(libraryHandle);
#endif
}

inline std::string DLERROR()
{
#ifdef _WIN32
    DWORD errorMessageID = ::GetLastError();
    if (errorMessageID == 0)
    {
        return std::string();
    }
    LPSTR messageBuffer = nullptr;
    size_t size         = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER
            | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        errorMessageID,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPSTR)&messageBuffer,
        0,
        NULL);
    std::string message(messageBuffer, size);
    LocalFree(messageBuffer);
    return message;
#else
    const char* error = dlerror();
    return (error != nullptr) ? std::string(error) : std::string();
#endif
}
