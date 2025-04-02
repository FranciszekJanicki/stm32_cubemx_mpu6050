#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "mpu6050.hpp"
#include "mpu6050_dmp.hpp"
#include "usart.h"

namespace {

    inline auto volatile gpio_exti_5 = false;

};

#ifdef __cplusplus
extern "C" {
#endif

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if (GPIO_Pin == GPIO_PIN_5) {
    std::puts("GPIO EXTI5 CALLBACK");
    gpio_exti_5 = true;
    // }
}

#ifdef __cplusplus
}
#endif

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_GPIO_Init();

    using namespace MPU6050;

    auto i2c_device = I2CDevice{&hi2c1, std::to_underlying(DevAddress::AD0_LOW)};
    i2c_device.bus_scan();

    auto mpu6050 = MPU6050::MPU6050{std::move(i2c_device),
                                    200U,
                                    GyroRange::GYRO_FS_2000,
                                    AccelRange::ACCEL_FS_2,
                                    DLPF::BW_42,
                                    DHPF::DHPF_RESET};

    auto mpu_dmp = MPU6050_DMP{std::move(mpu6050)};

    while (1) {
        // if (gpio_exti_5) {
        auto const& [r, p, y] = mpu_dmp.get_roll_pitch_yaw();
        std::printf("r: %f, p: %f, y: %f\n\r", r, p, y);
        gpio_exti_5 = false;
        //}
        HAL_Delay(50);
    }
}
