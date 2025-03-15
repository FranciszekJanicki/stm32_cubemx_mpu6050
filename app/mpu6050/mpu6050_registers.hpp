#ifndef MPU6050_REGISTERS_HPP
#define MPU6050_REGISTERS_HPP

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace MPU6050 {

    struct XG_OFFS_TC {
        std::uint8_t aux_vddio : 1;
        std::uint8_t xg_offs_tc : 7;
    } PACKED;

    struct YG_OFFS_TC {
        std::uint8_t : 1;
        std::uint8_t yg_offs_tc : 7;
    } PACKED;

    struct ZG_OFFS_TC {
        std::uint8_t : 1;
        std::uint8_t zg_offs_tc : 7;
    } PACKED;

    struct X_FINE_GAIN {
        std::uint8_t x_fine_gain : 8;
    } PACKED;

    struct Y_FINE_GAIN {
        std::uint8_t y_fine_gain : 8;
    } PACKED;

    struct Z_FINE_GAIN {
        std::uint8_t z_fine_gain : 8;
    } PACKED;

    struct XA_OFFS {
        std::uint16_t xa_offs : 16;
    } PACKED;

    struct YA_OFFS {
        std::uint16_t ya_offs : 16;
    } PACKED;

    struct ZA_OFFS {
        std::uint16_t za_offs : 8;
    } PACKED;

    struct XG_OFFS_USR {
        std::uint16_t xg_offs_usr : 16;
    } PACKED;

    struct YG_OFFS_USR {
        std::uint16_t yg_offs_usr : 16;
    } PACKED;

    struct ZG_OFFS_USR {
        std::uint16_t zg_offs_usr : 16;
    } PACKED;

    struct SELF_TEST_X {
        std::uint8_t xa_test : 3;
        std::uint8_t xg_test : 5;
    };

    struct SELF_TEST_Y {
        std::uint8_t ya_test : 3;
        std::uint8_t yg_test : 5;
    };

    struct SELF_TEST_Z {
        std::uint8_t za_test : 3;
        std::uint8_t zg_test : 5;
    };

    struct SELF_TEST_A {
        std::uint8_t : 2;
        std::uint8_t xa_test : 2;
        std::uint8_t ya_test : 2;
        std::uint8_t za_test : 2;
    };

    struct SMPLRT_DIV {
        std::uint8_t smplrt_div : 8;
    } PACKED;

    struct CONFIG {
        std::uint8_t : 2;
        std::uint8_t ext_sync_set : 3;
        std::uint8_t dlpf_cfg : 3;
    } PACKED;

    struct GYRO_CONFIG {
        std::uint8_t xg_st : 1;
        std::uint8_t yg_st : 1;
        std::uint8_t zg_st : 1;
        std::uint8_t fs_sel : 2;
        std::uint8_t : 3;
    } PACKED;

    struct ACCEL_CONFIG {
        std::uint8_t xa_st : 1;
        std::uint8_t ya_st : 1;
        std::uint8_t za_st : 1;
        std::uint8_t afs_sel : 2;
        std::uint8_t accel_hpf : 3;
    } PACKED;

    struct FF_THR {
        std::uint8_t ff_thr : 8;
    } PACKED;

    struct FF_DUR {
        std::uint8_t ff_dur : 8;
    } PACKED;

    struct MOT_THR {
        std::uint8_t mot_thr : 8;
    } PACKED;

    struct MOT_DUR {
        std::uint8_t mot_dur : 8;
    } PACKED;

    struct ZRMOT_THR {
        std::uint8_t zrmot_thr : 8;
    } PACKED;

    struct ZRMOT_DUR {
        std::uint8_t zrmot_dur : 8;
    } PACKED;

    struct FIFO_EN {
        std::uint8_t temp_fifo_en : 1;
        std::uint8_t xg_fifo_en : 1;
        std::uint8_t yg_fifo_en : 1;
        std::uint8_t zg_fifo_en : 1;
        std::uint8_t accel_fifo_en : 1;
        std::uint8_t slv2_fifo_en : 1;
        std::uint8_t slv1_fifo_en : 1;
        std::uint8_t slv0_fifo_en : 1;
    } PACKED;

    struct I2C_MST_CTRL {
        std::uint8_t mult_mst_en : 1;
        std::uint8_t wait_for_es : 1;
        std::uint8_t slv3_fifo_en : 1;
        std::uint8_t i2c_mst_p_nsr : 1;
        std::uint8_t i2c_mst_clk : 4;
    } PACKED;

    struct I2C_SLV_ADDR {
        std::uint8_t i2c_slv_rw : 1;
        std::uint8_t i2c_slv_addr : 7;
    } PACKED;

    struct I2C_SLV_REG {
        std::uint8_t i2c_slv_reg : 8;
    } PACKED;

    struct I2C_SLV_CTRL {
        std::uint8_t i2c_slv_en : 1;
        std::uint8_t i2c_slv_byte_sw : 1;
        std::uint8_t i2c_slv_reg_dis : 1;
        std::uint8_t i2c_slv_grp : 1;
        std::uint8_t i2c_slv_len : 4;
    } PACKED;

    struct I2C_SLV4_ADDR {
        std::uint8_t i2c_slv4_rw : 1;
        std::uint8_t i2c_slv4_addr : 7;
    } PACKED;

    struct I2C_SLV4_REG {
        std::uint8_t i2c_slv4_reg : 8;
    } PACKED;

    struct I2C_SLV4_DO {
        std::uint8_t i2c_slv4_do : 8;
    } PACKED;

    struct I2C_SLV4_CTRL {
        std::uint8_t i2c_slv4_en : 1;
        std::uint8_t i2c_slv4_int_en : 1;
        std::uint8_t i2c_slv4_reg_dis : 1;
        std::uint8_t i2c_mst_dly : 5;
    } PACKED;

    struct I2C_SLV4_DI {
        std::uint8_t i2c_slv4_di : 8;
    } PACKED;

    struct I2C_MST_STATUS {
        std::uint8_t pass_through : 1;
        std::uint8_t i2c_slv4_done : 1;
        std::uint8_t i2c_lost_arb : 1;
        std::uint8_t i2c_slv4_nack : 1;
        std::uint8_t i2c_slv3_nack : 1;
        std::uint8_t i2c_slv2_nack : 1;
        std::uint8_t i2c_slv1_nack : 1;
        std::uint8_t i2c_slv0_nack : 1;
    } PACKED;

    struct INT_PIN_CFG {
        std::uint8_t int_level : 1;
        std::uint8_t int_open : 1;
        std::uint8_t latch_int_en : 1;
        std::uint8_t int_rd_clear : 1;
        std::uint8_t fsync_int_level : 1;
        std::uint8_t fsync_int_en : 1;
        std::uint8_t i2c_bypass_en : 1;
        std::uint8_t clock_output : 1;
    } PACKED;

    struct INT_ENABLE {
        std::uint8_t ff_en : 1;
        std::uint8_t mot_en : 1;
        std::uint8_t zmot_en : 1;
        std::uint8_t fifo_oflow_en : 1;
        std::uint8_t i2c_mst_en : 1;
        std::uint8_t pll_rdy_int_en : 1;
        std::uint8_t dmp_int_en : 1;
        std::uint8_t raw_rdy_int_en : 1;
    } PACKED;

    struct DMP_INT_STATUS {
        std::uint8_t : 2;
        std::uint8_t dmp_int_5 : 1;
        std::uint8_t dmp_int_4 : 1;
        std::uint8_t dmp_int_3 : 1;
        std::uint8_t dmp_int_2 : 1;
        std::uint8_t dmp_int_1 : 1;
        std::uint8_t dmp_int_0 : 1;
    } PACKED;

    struct TC {
        std::uint8_t pwr_mode : 1;
        std::uint8_t offset : 6;
        std::uint8_t otp_bnk_vld : 1;
    } PACKED;

    struct INT_STATUS {
        std::uint8_t ff_int : 1;
        std::uint8_t mot_int : 1;
        std::uint8_t zmot_int : 1;
        std::uint8_t fifo_oflow_int : 1;
        std::uint8_t i2c_mst_int : 1;
        std::uint8_t pll_rdy_int : 1;
        std::uint8_t dmp_int : 1;
        std::uint8_t raw_rdy_int : 1;
    } PACKED;

    struct ACCEL_XOUT {
        std::uint16_t accel_xout : 16;
    } PACKED;

    struct ACCEL_YOUT {
        std::uint16_t accel_yout : 16;
    } PACKED;

    struct ACCEL_ZOUT {
        std::uint16_t accel_zout : 16;
    } PACKED;

    struct ACCEL_OUT {
        ACCEL_XOUT accel_xout;
        ACCEL_YOUT accel_yout;
        ACCEL_ZOUT accel_zout;
    } PACKED;

    struct TEMP_OUT {
        std::uint16_t temp_out : 16;
    } PACKED;

    struct GYRO_XOUT {
        std::uint16_t gyro_xout : 16;
    } PACKED;

    struct GYRO_YOUT {
        std::uint16_t gyro_yout : 16;
    } PACKED;

    struct GYRO_ZOUT {
        std::uint16_t gyro_zout : 16;
    } PACKED;

    struct GYRO_OUT {
        GYRO_XOUT gyro_xout;
        GYRO_YOUT gyro_yout;
        GYRO_ZOUT gyro_zout;
    } PACKED;

    struct EXT_SENS_DATA {
        std::uint8_t ext_sens_data : 8;
    } PACKED;

    struct MOT_DETECT_STATUS {
        std::uint8_t mot_xneg : 1;
        std::uint8_t mot_xpos : 1;
        std::uint8_t mot_yneg : 1;
        std::uint8_t mot_ypos : 1;
        std::uint8_t mot_zneg : 1;
        std::uint8_t mot_zpos : 1;
        std::uint8_t : 1;
        std::uint8_t mot_zrmot : 1;
    } PACKED;

    struct I2C_SLV_DO {
        std::uint8_t i2c_slv_do : 8;
    } PACKED;

    struct I2C_MST_DELAY_CTRL {
        std::uint8_t delay_es_shadow : 1;
        std::uint8_t : 2;
        std::uint8_t i2c_slv4_dly_en : 1;
        std::uint8_t i2c_slv3_dly_en : 1;
        std::uint8_t i2c_slv2_dly_en : 1;
        std::uint8_t i2c_slv1_dly_en : 1;
        std::uint8_t i2c_slv0_dly_en : 1;
    } PACKED;

    struct SIGNAL_PATH_RESET {
        std::uint8_t : 5;
        std::uint8_t gyro_reset : 1;
        std::uint8_t accel_reset : 1;
        std::uint8_t temp_reset : 1;
    } PACKED;

    struct MOT_DETECT_CTRL {
        std::uint8_t : 2;
        std::uint8_t accel_on_delay : 2;
        std::uint8_t ff_count : 2;
        std::uint8_t mot_count : 2;
    } PACKED;

    struct USER_CTRL {
        std::uint8_t dmp_en : 1;
        std::uint8_t fifo_en : 1;
        std::uint8_t i2c_mst_en : 1;
        std::uint8_t i2c_if_dis : 1;
        std::uint8_t dmp_reset : 1;
        std::uint8_t fifo_reset : 1;
        std::uint8_t i2c_mst_reset : 1;
        std::uint8_t sig_cond_reset : 1;
    } PACKED;

    struct PWR_MGMT_1 {
        std::uint8_t device_reset : 1;
        std::uint8_t sleep : 1;
        std::uint8_t cycle : 1;
        std::uint8_t : 1;
        std::uint8_t temp_dis : 1;
        std::uint8_t clksel : 3;
    } PACKED;

    struct PWR_MGMT_2 {
        std::uint8_t lp_wake_ctrl : 1;
        std::uint8_t : 1;
        std::uint8_t stby_xa : 1;
        std::uint8_t stby_ya : 1;
        std::uint8_t stby_za : 1;
        std::uint8_t stby_xg : 1;
        std::uint8_t stby_yg : 1;
        std::uint8_t stby_yz : 1;
    } PACKED;

    struct BANK_SEL {
        std::uint8_t : 1;
        std::uint8_t prftch_en : 1;
        std::uint8_t cfg_user_bank : 1;
        std::uint8_t mem_sel : 5;
    } PACKED;

    struct MEM_START_ADDR {
        std::uint8_t start_addr : 8;
    } PACKED;

    struct MEM_R_W {
        std::uint8_t mem_r_w : 8;
    } PACKED;

    struct DMP_CFG_1 {
        std::uint8_t dmp_cfg_1 : 8;
    } PACKED;

    struct DMP_CFG_2 {
        std::uint8_t dmp_cfg_2 : 8;
    } PACKED;

    struct FIFO_COUNT {
        std::uint16_t fifo_count : 16;
    } PACKED;

    struct FIFO_R_W {
        std::uint8_t fifo_r_w : 8;
    } PACKED;

    struct WHO_AM_I {
        std::uint8_t : 1;
        std::uint8_t who_am_i : 6;
        std::uint8_t : 1;
    } PACKED;

}; // namespace MPU6050

#endif // MPU6050_BITFIELDS_REGISTERS_HPP