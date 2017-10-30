#include "IMU_Detector.h"

using namespace flyhero;


extern "C" void app_main(void) {
	IMU::Sensor_Data accel, gyro;

	IMU *imu;
	ESP_ERROR_CHECK(IMU_Detector::Detect_IMU(&imu));

    imu->Init();

	while (true) {
        if (imu->Data_Ready()) {
            imu->Read_Data(accel, gyro);

            printf("%f %f %f %f %f %f\n", accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
        }
    }
}
