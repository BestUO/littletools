#include "http/http_download.hpp"
#include "manager/download_manager.hpp"
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <URL> <output_file> [thread_num]"
                  << std::endl;
        return 1;
    }

    std::string url              = argv[1];
    std::string output_file_name = argv[2];
    uint8_t thread_num           = (argc > 3) ? std::stoi(argv[3]) : 1;

    DownloadManager<HttpDownload, 1024 * 1024> download_manager;
    auto result = download_manager.Download(url, output_file_name, thread_num);
    std::cout << "download " << output_file_name
              << " task finish, result: " << (result ? "success" : "fail")
              << std::endl;
    return 0;
}