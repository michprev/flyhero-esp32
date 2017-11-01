#include <iostream>

#include "MS5611.h"


using namespace flyhero;

extern "C" void app_main(void)
{
    MS5611 &ms5611 = MS5611::Instance();
    ms5611.Init();

    int32_t press, temp;

    while (true)
    {
        ms5611.Get_Data(temp, press);
        std::cout << "temp: " << temp / 100.0 << " C, press: " << press / 100.0 << " mbar\n";

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}
