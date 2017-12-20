//
// Created by michal on 20.12.17.
//

#pragma once

#include <esp_partition.h>

namespace flyhero
{

class Logger
{
private:
    Logger();
    Logger(Logger const &);
    Logger &operator=(Logger const &);

    const esp_partition_t *partition;
    bool can_write;
    size_t write_offset;

public:
    static Logger& Instance();

    bool Init();
    bool Erase();
    bool Log(const void *data, size_t size);
};

}