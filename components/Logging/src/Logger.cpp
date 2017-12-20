//
// Created by michal on 20.12.17.
//

#include <esp_partition.h>
#include "Logger.h"

namespace flyhero
{

Logger& Logger::Instance()
{
    static Logger instance;

    return instance;
}

Logger::Logger()
{
    this->partition = NULL;
    this->can_write = true;
    this->write_offset = 0;
}

bool Logger::Init()
{
    this->partition = esp_partition_find_first((esp_partition_type_t)0x40, (esp_partition_subtype_t)0x40, NULL);

    return esp_partition_verify(this->partition) != NULL;
}

bool Logger::Erase()
{
    return esp_partition_erase_range(this->partition, 0, this->partition->size) == ESP_OK;
}

bool Logger::Log(const void *data, size_t size)
{
    if (!this->can_write)
        return false;

    if (esp_partition_write(this->partition, this->write_offset, data, size) != ESP_OK)
    {
        this->can_write = false;
        return false;
    }

    this->write_offset += size;
    return true;
}

}