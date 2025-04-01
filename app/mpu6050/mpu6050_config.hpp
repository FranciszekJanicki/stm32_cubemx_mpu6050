#ifndef MPU6050_CONFIG_HPP
#define MPU6050_CONFIG_HPP

#include "i2c_device.hpp"
#include "mpu6050_registers.hpp"
#include "quaternion3d.hpp"
#include "vector3d.hpp"
#include <cstdint>

namespace MPU6050 {

    enum struct RA : std::uint8_t {
        XG_OFFS_TC = 0x00,
        YG_OFFS_TC = 0x01,
        ZG_OFFS_TC = 0x02,
        X_FINE_GAIN = 0x03,
        Y_FINE_GAIN = 0x04,
        Z_FINE_GAIN = 0x05,
        XA_OFFS_H = 0x06,
        XA_OFFS_L_TC = 0x07,
        YA_OFFS_H = 0x08,
        YA_OFFS_L_TC = 0x09,
        ZA_OFFS_H = 0x0A,
        ZA_OFFS_L_TC = 0x0B,
        SELF_TEST_X = 0x0D,
        SELF_TEST_Y = 0x0E,
        SELF_TEST_Z = 0x0F,
        SELF_TEST_A = 0x10,
        XG_OFFS_USRH = 0x13,
        XG_OFFS_USRL = 0x14,
        YG_OFFS_USRH = 0x15,
        YG_OFFS_USRL = 0x16,
        ZG_OFFS_USRH = 0x17,
        ZG_OFFS_USRL = 0x18,
        SMPLRT_DIV = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        FF_THR = 0x1D,
        FF_DUR = 0x1E,
        MOT_THR = 0x1F,
        MOT_DUR = 0x20,
        ZRMOT_THR = 0x21,
        ZRMOT_DUR = 0x22,
        FIFO_EN = 0x23,
        I2C_MST_CTRL = 0x24,
        I2C_SLV0_ADDR = 0x25,
        I2C_SLV0_REG = 0x26,
        I2C_SLV0_CTRL = 0x27,
        I2C_SLV1_ADDR = 0x28,
        I2C_SLV1_REG = 0x29,
        I2C_SLV1_CTRL = 0x2A,
        I2C_SLV2_ADDR = 0x2B,
        I2C_SLV2_REG = 0x2C,
        I2C_SLV2_CTRL = 0x2D,
        I2C_SLV3_ADDR = 0x2E,
        I2C_SLV3_REG = 0x2F,
        I2C_SLV3_CTRL = 0x30,
        I2C_SLV4_ADDR = 0x31,
        I2C_SLV4_REG = 0x32,
        I2C_SLV4_DO = 0x33,
        I2C_SLV4_CTRL = 0x34,
        I2C_SLV4_DI = 0x35,
        I2C_MST_STATUS = 0x36,
        INT_PIN_CFG = 0x37,
        INT_ENABLE = 0x38,
        DMP_INT_STATUS = 0x39,
        INT_STATUS = 0x3A,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        EXT_SENS_DATA_00 = 0x49,
        EXT_SENS_DATA_01 = 0x4A,
        EXT_SENS_DATA_02 = 0x4B,
        EXT_SENS_DATA_03 = 0x4C,
        EXT_SENS_DATA_04 = 0x4D,
        EXT_SENS_DATA_05 = 0x4E,
        EXT_SENS_DATA_06 = 0x4F,
        EXT_SENS_DATA_07 = 0x50,
        EXT_SENS_DATA_08 = 0x51,
        EXT_SENS_DATA_09 = 0x52,
        EXT_SENS_DATA_10 = 0x53,
        EXT_SENS_DATA_11 = 0x54,
        EXT_SENS_DATA_12 = 0x55,
        EXT_SENS_DATA_13 = 0x56,
        EXT_SENS_DATA_14 = 0x57,
        EXT_SENS_DATA_15 = 0x58,
        EXT_SENS_DATA_16 = 0x59,
        EXT_SENS_DATA_17 = 0x5A,
        EXT_SENS_DATA_18 = 0x5B,
        EXT_SENS_DATA_19 = 0x5C,
        EXT_SENS_DATA_20 = 0x5D,
        EXT_SENS_DATA_21 = 0x5E,
        EXT_SENS_DATA_22 = 0x5F,
        EXT_SENS_DATA_23 = 0x60,
        MOT_DETECT_STATUS = 0x61,
        I2C_SLV0_DO = 0x63,
        I2C_SLV1_DO = 0x64,
        I2C_SLV2_DO = 0x65,
        I2C_SLV3_DO = 0x66,
        I2C_MST_DELAY_CTRL = 0x67,
        SIGNAL_PATH_RESET = 0x68,
        MOT_DETECT_CTRL = 0x69,
        USER_CTRL = 0x6A,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        BANK_SEL = 0x6D,
        MEM_START_ADDR = 0x6E,
        MEM_R_W = 0x6F,
        DMP_CFG_1 = 0x70,
        DMP_CFG_2 = 0x71,
        FIFO_COUNTH = 0x72,
        FIFO_COUNTL = 0x73,
        FIFO_R_W = 0x74,
        WHO_AM_I = 0x75,
    };

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    template <typename T>
    using Quat3D = Utility::Quaternion3D<T>;

    using I2CDevice = STM32_Utility::I2CDevice;

    enum struct DevAddress : std::uint16_t {
        AD0_LOW = 0x68,
        AD0_HIGH = 0x69,
    };

    enum struct GyroRange : std::uint8_t {
        GYRO_FS_250 = 0x00,
        GYRO_FS_500 = 0x01,
        GYRO_FS_1000 = 0x02,
        GYRO_FS_2000 = 0x03,
    };

    enum struct AccelRange : std::uint8_t {
        ACCEL_FS_2 = 0x00,
        ACCEL_FS_4 = 0x01,
        ACCEL_FS_8 = 0x02,
        ACCEL_FS_16 = 0x03,
    };

    enum struct ExtSync : std::uint8_t {
        DISABLED = 0x0,
        TEMP_OUT_L = 0x1,
        GYRO_XOUT_L = 0x2,
        GYRO_YOUT_L = 0x3,
        GYRO_ZOUT_L = 0x4,
        ACCEL_XOUT_L = 0x5,
        ACCEL_YOUT_L = 0x6,
        ACCEL_ZOUT_L = 0x7,
    };

    enum struct DLPF : std::uint8_t {
        BW_256 = 0x00,
        BW_188 = 0x01,
        BW_98 = 0x02,
        BW_42 = 0x03,
        BW_20 = 0x04,
        BW_10 = 0x05,
        BW_5 = 0x06,
    };

    enum struct DHPF : std::uint8_t {
        DHPF_RESET = 0x00,
        DHPF_5 = 0x01,
        DHPF_2P5 = 0x02,
        DHPF_1P25 = 0x03,
        DHPF_0P63 = 0x04,
        DHPF_HOLD = 0x07,
    };

    enum struct ClockDiv : std::uint8_t {
        DIV_500 = 0x9,
        DIV_471 = 0xA,
        DIV_444 = 0xB,
        DIV_421 = 0xC,
        DIV_400 = 0xD,
        DIV_381 = 0xE,
        DIV_364 = 0xF,
        DIV_348 = 0x0,
        DIV_333 = 0x1,
        DIV_320 = 0x2,
        DIV_308 = 0x3,
        DIV_296 = 0x4,
        DIV_286 = 0x5,
        DIV_276 = 0x6,
        DIV_267 = 0x7,
        DIV_258 = 0x8,
    };

    enum struct IntMode : std::uint8_t {
        ACTIVEHIGH = 0x00,
        ACTIVELOW = 0x01,
    };

    enum struct IntDrive : std::uint8_t {
        PUSHPULL = 0x00,
        OPENDRAIN = 0x01,
    };

    enum struct IntLatch : std::uint8_t {
        PULSE50US = 0x00,
        WAITCLEAR = 0x01,
    };

    enum struct IntClear : std::uint8_t {
        STATUSREAD = 0x00,
        ANYREAD = 0x01,
    };

    enum struct DetectDecrement : std::uint8_t {
        DECREMENT_RESET = 0x0,
        DECREMENT_1 = 0x1,
        DECREMENT_2 = 0x2,
        DECREMENT_4 = 0x3,
    };

    enum struct Delay : std::uint8_t {
        DELAY_3MS = 0b11,
        DELAY_2MS = 0b10,
        DELAY_1MS = 0b01,
        NO_DELAY = 0b00,
    };

    enum struct Clock : std::uint8_t {
        INTERNAL = 0x00,
        PLL_XGYRO = 0x01,
        PLL_YGYRO = 0x02,
        PLL_ZGYRO = 0x03,
        PLL_EXT32K = 0x04,
        PLL_EXT19M = 0x05,
        KEEP_RESET = 0x07,
    };

    enum struct WakeFreq : std::uint8_t {
        FREQ_1P25 = 0x0,
        FREQ_5 = 0x1,
        FREQ_20 = 0x2,
        FREQ_40 = 0x3,
    };

    enum struct SlaveNum : std::uint8_t {
        SLAVE1 = 1,
        SLAVE2 = 2,
        SLAVE3 = 3,
    };

    auto constexpr GYRO_OUTPUT_RATE_DLPF_EN_HZ = 1000U;
    auto constexpr GYRO_OUTPUT_RATE_DLPF_DIS_HZ = 8000U;
    auto constexpr ACCEL_OUTPUT_RATE_HZ = 1000U;

    inline Vec3D<float> accel_to_roll_pitch_yaw(Vec3D<float> const& accel_scaled) noexcept
    {
        return Vec3D<float>{
            std::atan2(accel_scaled.y, accel_scaled.z),
            -std::atan2(accel_scaled.x, std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z)),
            0.0F};
    }

    inline float accel_to_roll(Vec3D<float> const& accel_scaled) noexcept
    {
        return std::atan2(accel_scaled.y, accel_scaled.z);
    }

    inline float accel_to_pitch(Vec3D<float> const& accel_scaled) noexcept
    {
        return -std::atan2(accel_scaled.x,
                           std::sqrt(accel_scaled.y * accel_scaled.y + accel_scaled.z * accel_scaled.z));
    }

    inline float accel_to_yaw(Vec3D<float> const& accel_scaled) noexcept
    {
        return 0.0F;
    }

    inline float gyro_range_to_scale(GyroRange gyro_range) noexcept
    {
        switch (gyro_range) {
            case GyroRange::GYRO_FS_250:
                return 131.0F;
            case GyroRange::GYRO_FS_500:
                return 65.5F;
            case GyroRange::GYRO_FS_1000:
                return 32.8F;
            case GyroRange::GYRO_FS_2000:
                return 16.4F;
            default:
                return 0.0F;
        }
    }

    inline float gyro_config_to_scale(GYRO_CONFIG const gyro_config) noexcept
    {
        return gyro_range_to_scale(static_cast<GyroRange>(gyro_config.fs_sel));
    }

    inline float accel_range_to_scale(AccelRange accel_range) noexcept
    {
        switch (accel_range) {
            case AccelRange::ACCEL_FS_2:
                return 16384.0F;
            case AccelRange::ACCEL_FS_4:
                return 8192.0F;
            case AccelRange::ACCEL_FS_8:
                return 4096.0F;
            case AccelRange::ACCEL_FS_16:
                return 2048.0F;
            default:
                return 0.0F;
        }
    }

    inline float accel_config_to_scale(ACCEL_CONFIG const accel_config) noexcept
    {
        return accel_range_to_scale(static_cast<AccelRange>(accel_config.afs_sel));
    }

    inline std::uint16_t gyro_output_rate_to_smplrt_div(std::uint32_t const output_rate,
                                                        bool const dlpf_enabled = false) noexcept
    {
        return static_cast<std::uint16_t>((dlpf_enabled ? GYRO_OUTPUT_RATE_DLPF_EN_HZ : GYRO_OUTPUT_RATE_DLPF_DIS_HZ) /
                                          output_rate) -
               1U;
    }

}; // namespace MPU6050

#endif // MPU6050_CONFIG_HPP