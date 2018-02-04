//
// Created by michal on 20.12.17.
//

#include <esp_partition.h>
#include "Logger.h"

namespace flyhero
{

Logger &Logger::Instance()
{
    static Logger instance;

    return instance;
}

Logger::Logger()
{
    this->partition = NULL;
}

bool Logger::Init()
{
    this->partition = esp_partition_find_first((esp_partition_type_t) 0x40, (esp_partition_subtype_t) 0x40, NULL);
    this->write_offset = 0;
    this->read_offset = 0;
    this->log = false;

    return esp_partition_verify(this->partition) != NULL;
}

void Logger::Enable_Writes()
{
    this->log = true;
}

bool Logger::Erase()
{
    return esp_partition_erase_range(this->partition, 0, this->partition->size) == ESP_OK;
}

bool Logger::Log_Next(const void *data, size_t size)
{
    if (!this->log || esp_partition_write(this->partition, this->write_offset, data, size) != ESP_OK)
        return false;

    this->write_offset += size;
    return true;
}

bool Logger::Read_Next(void *data, size_t size)
{
    if (esp_partition_read(this->partition, this->read_offset, data, size) != ESP_OK)
        return false;

    this->read_offset += size;
    return true;
}

}