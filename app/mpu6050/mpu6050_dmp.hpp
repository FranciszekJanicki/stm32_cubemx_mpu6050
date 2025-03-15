#ifndef MPU6050_DMP_HPP
#define MPU6050_DMP_HPP

#include "mpu6050.hpp"
#include "mpu6050_config.hpp"
#include "mpu6050_dmp_config.hpp"
#include "mpu6050_dmp_img.hpp"

namespace MPU6050 {

    struct MPU6050_DMP {
    public:
        MPU6050_DMP() noexcept = default;
        MPU6050_DMP(MPU6050&& mpu6050) noexcept;

        MPU6050_DMP(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP(MPU6050_DMP&& other) noexcept = default;

        MPU6050_DMP& operator=(MPU6050_DMP const& other) noexcept = delete;
        MPU6050_DMP& operator=(MPU6050_DMP&& other) noexcept = default;

        ~MPU6050_DMP() noexcept;

    private:
        bool initialized_{false};

        MPU6050 mpu6050_{};
    };

}; // namespace MPU6050

#endif // MPU6050_DMP_HPP