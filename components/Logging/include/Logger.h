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
    size_t write_offset, read_offset;
    bool log;

public:
    static Logger &Instance();

    bool Init();

    void Enable_Writes();

    bool Erase();

    bool Log_Next(const void *data, size_t size);

    void Reset_Read_Pointer();

    bool Read_Next(void *data, size_t size);
};

}