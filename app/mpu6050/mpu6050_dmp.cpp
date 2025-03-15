#include "mpu6050_dmp.hpp"
#include "mpu6050_config.hpp"
#include "mpu6050_dmp_config.hpp"

namespace MPU6050 {

    MPU6050_DMP::MPU6050_DMP(MPU6050&& mpu6050) noexcept : mpu6050_{std::forward<MPU6050>(mpu6050)}
    {}

    MPU6050_DMP::~MPU6050_DMP() noexcept
    {}

}; // namespace MPU6050