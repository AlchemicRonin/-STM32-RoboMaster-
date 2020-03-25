#ifndef MPU6500REG_H
#define MPU6500REG_H

#define MPU6050_ID                          (0x68)
#define MPU6500_ID                          (0x70)
#define MPU9250_ID                          (0x71)
#define MPU9255_ID                          (0x73)

#define MPU_I2C_ADDR                        (0xD0)
#define MPU_GYRO_I2C_ADDRESS                (0xD0)
#define MPU_ACCEL_I2C_ADDRESS               (0xD0)


#define MPU_SELF_TEST_X_GYRO                (0x00)
#define MPU_SELF_TEST_Y_GYRO                (0x01)
#define MPU_SELF_TEST_Z_GYRO                (0x02)

#define MPU_SELF_TEST_X_ACCEL               (0x0D)
#define MPU_SELF_TEST_Y_ACCEL               (0x0E)
#define MPU_SELF_TEST_Z_ACCEL               (0x0F)

#define MPU_XG_OFFSET_H                     (0x13)
#define MPU_XG_OFFSET_L                     (0x14)
#define MPU_YG_OFFSET_H                     (0x15)
#define MPU_YG_OFFSET_L                     (0x16)
#define MPU_ZG_OFFSET_H                     (0x17)
#define MPU_ZG_OFFSET_L                     (0x18)

// this register is only effetive when FCHOICE_B register bits are 2b00,and 0< DLPF_CFG <7,more please refer to the datasheet
// the value can be max 256,means 257 div,but usually be 0 (1 div)  in our robots.
//SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)
#define MPU_SMPLRT_DIV                      (0x19)

# define MPU_SMPLRT_DIV_1                   (0x00)
# define MPU_SMPLRT_DIV_2                   (0x01)
# define MPU_SMPLRT_DIV_3                   (0x02)
# define MPU_SMPLRT_DIV_4                   (0x03)
# define MPU_SMPLRT_DIV_5                   (0x04)
# define MPU_SMPLRT_DIV_6                   (0x05)
# define MPU_SMPLRT_DIV_7                   (0x06)
# define MPU_SMPLRT_DIV_8                   (0x07)
# define MPU_SMPLRT_DIV_9                   (0x08)
# define MPU_SMPLRT_DIV_10                  (0x0A)



#define MPU_CONFIG                          (0x1A)

# define MPU_FIFO_MODE_SHIFTS               (0x6)
# define MPU_FIFO_MODE_OFF_REPLACE_OLD_DATA (0x1 << MPU_FIFO_MODE_SHIFTS )

# define MPU_EXT_SYNC_SET_SHIFTS            (0x3)
# define MPU_EXT_SYNC_DISABLE               (0x0 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_TEMP_OUT              (0x1 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_GYRO_XOUT             (0x2 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_GYRO_YOUT             (0x3 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_GYRO_ZOUT             (0x4 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_ACCEL_XOUT            (0x5 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_ACCEL_YOUT            (0x6 << MPU_EXT_SYNC_SET_SHIFTS)
# define MPU_EXT_SYNC_ACCEL_ZOUT            (0x7 << MPU_EXT_SYNC_SET_SHIFTS)

# define MPU_DLPF_CFG_SHIFTS                (0x0)
# define MPU_DLPF_CFG_0_SET                 (0x0 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_1_SET                 (0x1 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_2_SET                 (0x2 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_3_SET                 (0x3 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_4_SET                 (0x4 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_5_SET                 (0x5 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_6_SET                 (0x6 << MPU_DLPF_CFG_SHIFTS)
# define MPU_DLPF_CFG_7_SET                 (0x7 << MPU_DLPF_CFG_SHIFTS)




#define MPU_GYRO_CONFIG                     (0x1B)

# define MPU_XG_SELF_TEST_SHIFTS            (0x7)
# define MPU_XG_SELF_TEST_SET               (0x1 << MPU_XG_SELF_TEST_SHIFTS)
# define MPU_YG_SELF_TEST_SHIFTS            (0x6)
# define MPU_YG_SELF_TEST_SET               (0x1 << MPU_YG_SELF_TEST_SHIFTS)
# define MPU_ZG_SELF_TEST_SHIFTS            (0x5)
# define MPU_ZG_SELF_TEST_SET               (0x1 << MPU_ZG_SELF_TEST_SHIFTS)

# define MPU_GYRO_FS_SEL_SHIFTS             (0x3)
# define MPU_GYRO_RANGLE_250                (0x0 << MPU_GYRO_FS_SEL_SHIFTS)
# define MPU_GYRO_RANGLE_500                (0x1 << MPU_GYRO_FS_SEL_SHIFTS)
# define MPU_GYRO_RANGLE_1000               (0x2 << MPU_GYRO_FS_SEL_SHIFTS)
# define MPU_GYRO_RANGLE_2000               (0x3 << MPU_GYRO_FS_SEL_SHIFTS)

# define MPU_FCHOCIE_B_SHIFTS               (0x0)
# define MPU_FCHOCIE_B_2B00                 (0x0 << MPU_FCHOCIE_B_SHIFTS)
# define MPU_FCHOCIE_B_2B01                 (0x1 << MPU_FCHOCIE_B_SHIFTS)
# define MPU_FCHOCIE_B_2B10                 (0x2 << MPU_FCHOCIE_B_SHIFTS)
# define MPU_FCHOCIE_B_2B11                 (0x3 << MPU_FCHOCIE_B_SHIFTS)


#define MPU_ACCEL_CONFIG                    (0x1C)

# define MPU_XA_SELF_TEST_SHIFTS            (0x7)
# define MPU_XA_SELF_TEST_SET               (0x1 << MPU_XA_SELF_TEST_SHIFTS)
# define MPU_YA_SELF_TEST_SHIFTS            (0x6)
# define MPU_YA_SELF_TEST_SET               (0x1 << MPU_YA_SELF_TEST_SHIFTS)
# define MPU_ZA_SELF_TEST_SHIFTS            (0x5)
# define MPU_ZA_SELF_TEST_SET               (0x1 << MPU_ZA_SELF_TEST_SHIFTS)

# define MPU_ACCEL_FS_SEL_SHIFTS            (0x3)
# define MPU_ACCEL_RANGLE_2G                (0x0 << MPU_ACCEL_FS_SEL_SHIFTS)
# define MPU_ACCEL_RANGLE_4G                (0x1 << MPU_ACCEL_FS_SEL_SHIFTS)
# define MPU_ACCEL_RANGLE_8G                (0x2 << MPU_ACCEL_FS_SEL_SHIFTS)
# define MPU_ACCEL_RANGLE_16G               (0x3 << MPU_ACCEL_FS_SEL_SHIFTS)


#define MPU_ACCEL_CONFIG_2                  (0x1D)

# define MPU_ACCEL_FCHOICE_B_SHIFTS         (0x3)
# define MPU_ACCEL_FCHOICE_B_0_SET          (0x0 << MPU_ACCEL_FCHOICE_B_SHIFTS)
# define MPU_ACCEL_FCHOICE_B_1_SET          (0x1 << MPU_ACCEL_FCHOICE_B_SHIFTS)

# define MPU_A_DLPL_CFG_SHIFTS              (0x0)
# define MPU_A_DLPL_CFG_0_SET               (0x0 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_1_SET               (0x1 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_2_SET               (0x2 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_3_SET               (0x3 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_4_SET               (0x4 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_5_SET               (0x5 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_6_SET               (0x6 << MPU_A_DLPL_CFG_SHIFTS)
# define MPU_A_DLPL_CFG_7_SET               (0x7 << MPU_A_DLPL_CFG_SHIFTS)


#define MPU_LP_ACCEL_ODR                    (0x1E)

# define MPU_LPOSC_CLKSEL_SHIFTS            (0x0)
# define MPU_LPOSC_CLKSEL_0_SET             (0x0 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_1_SET             (0x1 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_2_SET             (0x2 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_3_SET             (0x3 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_4_SET             (0x4 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_5_SET             (0x5 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_6_SET             (0x6 << MPU_LPOSC_CLKSEL_SHIFTS)
# define MPU_LPOSC_CLKSEL_7_SET             (0x7 << MPU_LPOSC_CLKSEL_SHIFTS)



#define MPU_WOM_THR                         (0x1F)



#define MPU_FIFO_EN                         (0X23)

# define MPU_FIFO_TEMP_OUT_SHIFTS           (0x7)
# define MPU_FIFO_TEMP_OUT_ENABLE           (0x1 << MPU_FIFO_TEMP_OUT_SHIFTS)

# define MPU_FIFO_GYRO_X_OUT_SHIFTS         (0x6)
# define MPU_FIFO_GYRO_X_OUT_ENABLE         (0x1 << MPU_FIFO_GYRO_X_OUT_SHIFTS)

# define MPU_FIFO_GYRO_Y_OUT_SHIFTS         (0x5)
# define MPU_FIFO_GYRO_Y_OUT_ENABLE         (0x1 << MPU_FIFO_GYRO_Y_OUT_SHIFTS)

# define MPU_FIFO_GYRO_Z_OUT_SHIFTS         (0x4)
# define MPU_FIFO_GYRO_Z_OUT_ENABLE         (0x1 << MPU_FIFO_GYRO_Z_OUT_SHIFTS)

# define MPU_FIFO_ACCEL_OUT_SHIFTS          (0x3)
# define MPU_FIFO_ACCEL_OUT_ENABLE          (0x1 << MPU_FIFO_ACCEL_OUT_SHIFTS)

# define MPU_FIFO_SLV_2_OUT_SHIFTS          (0x2)
# define MPU_FIFO_SLV_2_OUT_ENABLE          (0x1 << MPU_FIFO_SLV_2_OUT_SHIFTS)

# define MPU_FIFO_SLV_1_OUT_SHIFTS          (0x2)
# define MPU_FIFO_SLV_1_OUT_ENABLE          (0x1 << MPU_FIFO_SLV_1_OUT_SHIFTS)

# define MPU_FIFO_SLV_0_OUT_SHIFTS          (0x2)
# define MPU_FIFO_SLV_0_OUT_ENABLE          (0x1 << MPU_FIFO_SLV_0_OUT_SHIFTS)





#define MPU_I2C_MST_CTRL                    (0X24)

# define MPU_MULT_MST_EN_SHIFTS             (0x7)
# define MPU_MULT_MST_EN                    (0x1 << MPU_MULT_MST_EN_SHIFTS)

# define MPU_WAIT_FOR_ES_SHIFTS             (0x6)
# define MPU_WAIT_FOR_ES_EN                 (0x1 << MPU_WAIT_FOR_ES_SHIFTS)

# define MPU_SLV_3_FIFO_EN_SHIFTS           (0x5)
# define MPU_SLV_3_FIFO_EN                  (0x1 << MPU_SLV_3_FIFO_EN_SHIFTS)

# define MPU_I2C_MST_P_NSR_SHIFTS           (0x4)
# define MPU_I2C_MST_P_NSR                  (0x1 << MPU_I2C_MST_P_NSR_SHIFTS)

# define MPU_I2C_MST_CLK_SHIFTS             (0x0)
# define MPU_I2C_MST_CLK_348_KHZ            (0x0 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_333_KHZ            (0x1 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_320_KHZ            (0x2 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_308_KHZ            (0x3 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_296_KHZ            (0x4 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_286_KHZ            (0x5 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_276_KHZ            (0x6 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_267_KHZ            (0x7 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_258_KHZ            (0x8 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_500_KHZ            (0x9 << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_471_KHZ            (0xA << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_444_KHZ            (0xB << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_421_KHZ            (0xC << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_400_KHZ            (0xD << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_381_KHZ            (0xE << MPU_I2C_MST_CLK_SHIFTS)
# define MPU_I2C_MST_CLK_364_KHZ            (0xF << MPU_I2C_MST_CLK_SHIFTS)

#define MPU_I2CSLV0_ADDR                    (0X25)
#define MPU_I2CSLV0_REG                     (0X26)
#define MPU_I2CSLV0_CTRL                    (0X27)
#define MPU_I2CSLV1_ADDR                    (0X28)
#define MPU_I2CSLV1_REG                     (0X29)
#define MPU_I2CSLV1_CTRL                    (0X2A)
#define MPU_I2CSLV2_ADDR                    (0X2B)
#define MPU_I2CSLV2_REG                     (0X2C)
#define MPU_I2CSLV2_CTRL                    (0X2D)
#define MPU_I2CSLV3_ADDR                    (0X2E)
#define MPU_I2CSLV3_REG                     (0X2F)
#define MPU_I2CSLV3_CTRL                    (0X30)
#define MPU_I2CSLV4_ADDR                    (0X31)
#define MPU_I2CSLV4_REG                     (0X32)
#define MPU_I2CSLV4_DO                      (0X33)
#define MPU_I2CSLV4_CTRL                    (0X34)
#define MPU_I2CSLV4_DI                      (0X35)

#define MPU_I2CMST_STA                      (0X36)

# define MPU_PASS_THROUGH_SHIFTS            (0x7)
# define MPU_PASS_THROUGH                   (0x1 << MPU_PASS_THROUGH_SHIFTS)

# define MPU_I2C_SLV4_DONE_SHIFTS           (0x6)
# define MPU_I2C_SLV4_DONE                  (0x1 << MPU_I2C_SLV4_DONE_SHIFTS)

# define MPU_I2C_LOST_ARB_SHIFTS            (0x5)
# define MPU_I2C_LOST_ARB                   (0x1 << MPU_I2C_LOST_ARB_SHIFTS)

# define MPU_I2C_SLV4_NACK_SHIFTS           (0x4)
# define MPU_I2C_SLV4_NACK                  (0x1 << MPU_I2C_SLV4_NACK_SHIFTS)


# define MPU_I2C_SLV3_NACK_SHIFTS           (0x3)
# define MPU_I2C_SLV3_NACK                  (0x1 << MPU_I2C_SLV3_NACK_SHIFTS)

# define MPU_I2C_SLV2_NACK_SHIFTS           (0x2)
# define MPU_I2C_SLV2_NACK                  (0x1 << MPU_I2C_SLV2_NACK_SHIFTS)

# define MPU_I2C_SLV1_NACK_SHIFTS           (0x1)
# define MPU_I2C_SLV1_NACK                  (0x1 << MPU_I2C_SLV1_NACK_SHIFTS)

# define MPU_I2C_SLV0_NACK_SHIFTS           (0x0)
# define MPU_I2C_SLV0_NACK                  (0x1 << MPU_I2C_SLV0_NACK_SHIFTS)



#define MPU_INTBP_CFG                       (0X37)

# define MPU_INTBP_ACTL_SHIFTS              (0x7)
# define MPU_INTBP_ACTL                     (0x1 << MPU_INTBP_ACTL_SHIFTS)

# define MPU_INTBP_OPEN_SHIFTS              (0x6)
# define MPU_INTBP_OPEN                     (0x1 << MPU_INTBP_OPEN_SHIFTS)

# define MPU_LATCH_INT_EN_SHIFTS            (0x5)
# define MPU_LATCH_INT_EN                   (0x1 << MPU_LATCH_INT_EN_SHIFTS)

# define MPU_INT_ANYRD_2CLEAR_SHIFTS        (0x4)
# define MPU_INT_ANYRD_2CLEAR               (0x1 << MPU_INT_ANYRD_2CLEAR_SHIFTS)

# define MPU_ACTL_FSYNC_SHIFTS              (0x3)
# define MPU_ACTL_FSYNC                     (0x1 << MPU_ACTL_FSYNC_SHIFTS)

# define MPU_FSYNC_INT_MODE_EN_SHIFTS       (0x2)
# define MPU_FSYNC_INT_MODE_EN              (0x1 << MPU_FSYNC_INT_MODE_EN_SHIFTS)

# define MPU_BYPASS_EN_SHIFTS               (0x1)
# define MPU_BYPASS_EN                      (0x1 << MPU_BYPASS_EN_SHIFTS)




#define MPU_INT_ENABLE                      (0X38)

# define MPU_INT_WOM_EN_SHIFTS              (0x6)
# define MPU_INT_WOM_EN                     (0x1 << MPU_INT_WOM_EN_SHIFTS)

# define MPU_FIFO_OVERFLOW_EN_SHIFTS        (0x4)
# define MPU_FIFO_OVERFLOW_EN               (0x1 << MPU_FIFO_OVERFLOW_EN_SHIFTS)

# define MPU_FSYNC_INT_EN_SHIFTS            (0x3)
# define MPU_FSYNC_INT_EN                   (0x1 << MPU_FSYNC_INT_EN_SHIFTS)

# define MPU_RAW_RDY_EN_SHIFTS              (0x0)
# define MPU_RAW_RDY_EN                     (0x1 << MPU_RAW_RDY_EN_SHIFTS)


#define MPU_INT_STATUS                      (0X3A)

# define MPU_INT_WOM_INT                    (0x1 << MPU_INT_WOM_EN_SHIFTS)

# define MPU_FIFO_OVERFLOW_INT              (0x1 << MPU_FIFO_OVERFLOW_EN_SHIFTS)

# define MPU_FSYNC_INT_INT                  (0x1 << MPU_FSYNC_INT_EN_SHIFTS)

# define MPU_DMP_INT_SHIFTS                 (0x1)
# define MPU_DMP_INT                        (0x1 << MPU_DMP_INT_SHIFTS)

# define MPU_RAW_RDY_INT                    (0x1 << MPU_RAW_RDY_EN_SHIFTS)


#define MPU_ACCEL_XOUT_H                    (0x3B)
#define MPU_ACCEL_XOUT_L                    (0x3C)
#define MPU_ACCEL_YOUT_H                    (0x3D)
#define MPU_ACCEL_YOUT_L                    (0x3E)
#define MPU_ACCEL_ZOUT_H                    (0x3F)
#define MPU_ACCEL_ZOUT_L                    (0x40)
#define MPU_TEMP_OUT_H                      (0x41)
#define MPU_TEMP_OUT_L                      (0x42)
#define MPU_GYRO_XOUT_H                     (0x43)
#define MPU_GYRO_XOUT_L                     (0x44)
#define MPU_GYRO_YOUT_H                     (0x45)
#define MPU_GYRO_YOUT_L                     (0x46)
#define MPU_GYRO_ZOUT_H                     (0x47)
#define MPU_GYRO_ZOUT_L                     (0x48)

#define MPU_EXT_SENS_DATA_00                (0x49)
#define MPU_EXT_SENS_DATA_01                (0x4A)
#define MPU_EXT_SENS_DATA_02                (0x4B)
#define MPU_EXT_SENS_DATA_03                (0x4C)
#define MPU_EXT_SENS_DATA_04                (0x4D)
#define MPU_EXT_SENS_DATA_05                (0x4E)
#define MPU_EXT_SENS_DATA_06                (0x4F)
#define MPU_EXT_SENS_DATA_07                (0x50)
#define MPU_EXT_SENS_DATA_08                (0x51)
#define MPU_EXT_SENS_DATA_09                (0x52)
#define MPU_EXT_SENS_DATA_10                (0x53)
#define MPU_EXT_SENS_DATA_11                (0x54)
#define MPU_EXT_SENS_DATA_12                (0x55)
#define MPU_EXT_SENS_DATA_13                (0x56)
#define MPU_EXT_SENS_DATA_14                (0x57)
#define MPU_EXT_SENS_DATA_15                (0x58)
#define MPU_EXT_SENS_DATA_16                (0x59)
#define MPU_EXT_SENS_DATA_17                (0x5A)
#define MPU_EXT_SENS_DATA_18                (0x5B)
#define MPU_EXT_SENS_DATA_19                (0x5C)
#define MPU_EXT_SENS_DATA_20                (0x5D)
#define MPU_EXT_SENS_DATA_21                (0x5E)
#define MPU_EXT_SENS_DATA_22                (0x5F)
#define MPU_EXT_SENS_DATA_23                (0x60)

#define MPU_I2CSLV0_DO                      (0X63)
#define MPU_I2CSLV1_DO                      (0X64)
#define MPU_I2CSLV2_DO                      (0X65)
#define MPU_I2CSLV3_DO                      (0X66)

#define MPU_I2C_MST_DELAY_CTRL              (0X67)

# define MPU_I2C_DELAY_ES_SHADOW_SHIFTS     (0x7)
# define MPU_I2C_DELAY_ES_SHADOW            (0x1 << MPU_I2C_DELAY_ES_SHADOW_SHIFTS)

# define MPU_I2C_SLV4_DLY_EN_SHIFTS         (0x4)
# define MPU_I2C_SLV4_DLY_EN                (0x1 << MPU_I2C_SLV4_DLY_EN_SHIFTS)

# define MPU_I2C_SLV3_DLY_EN_SHIFTS         (0x3)
# define MPU_I2C_SLV3_DLY_EN                (0x1 << MPU_I2C_SLV3_DLY_EN_SHIFTS)

# define MPU_I2C_SLV2_DLY_EN_SHIFTS         (0x2)
# define MPU_I2C_SLV2_DLY_EN                (0x1 << MPU_I2C_SLV2_DLY_EN_SHIFTS)

# define MPU_I2C_SLV1_DLY_EN_SHIFTS         (0x1)
# define MPU_I2C_SLV1_DLY_EN                (0x1 << MPU_I2C_SLV1_DLY_EN_SHIFTS)

# define MPU_I2C_SLV0_DLY_EN_SHIFTS         (0x0)
# define MPU_I2C_SLV0_DLY_EN                (0x1 << MPU_I2C_SLV0_DLY_EN_SHIFTS)


#define MPU_SIGNAL_PATH_RESET               (0X68)

# define MPU_GYRO_RST_SHIFTS                (0x2)
# define MPU_GYRO_RST                       (0x1 << MPU_GYRO_RST_SHIFTS)

# define MPU_ACCEL_RST_SHIFTS               (0x2)
# define MPU_ACCEL_RST                      (0x1 << MPU_ACCEL_RST_SHIFTS)

# define MPU_TEMP_RST_SHIFTS                (0x2)
# define MPU_TEMP_RST                       (0x1 << MPU_TEMP_RST_SHIFTS)


#define MPU_MOT_DETECT_CTRL                 (0X69)

# define MPU_ACCEL_INTEL_EN_SHIFTS          (0x7)
# define MPU_ACCEL_INTEL_EN                 (0x1 << MPU_ACCEL_INTEL_EN_SHIFTS)

# define MPU_ACCEL_INTEL_MODE_SHIFTS        (0x6)
# define MPU_ACCEL_INTEL_MODE_COMPARE       (0x1 << MPU_ACCEL_INTEL_MODE_SHIFTS)




#define MPU_USER_CTRL                       (0X6A)

# define MPU_DMP_EN_SHIFTS                  (0x7)
# define MPU_DMP_EN                         (0x1 << MPU_DMP_EN_SHIFTS)

# define MPU_FIFO_MODE_EN_SHIFTS            (0x6)
# define MPU_FIFO_MODE_EN                   (0x1 << MPU_FIFO_MODE_EN_SHIFTS)

# define MPU_I2C_MST_EN_SHIFTS              (0x5)
# define MPU_I2C_MST_EN                     (0x1 << MPU_I2C_MST_EN_SHIFTS)

# define MPU_I2C_IF_DIS_SHIFTS              (0x4)
# define MPU_I2C_IF_DIS                     (0x1 << MPU_I2C_IF_DIS_SHIFTS)

# define MPU_DMP_RST_SHIFTS                 (0x3)
# define MPU_DMP_RST                        (0x1 << MPU_DMP_RST_SHIFTS)

# define MPU_FIFO_RST_SHIFTS                (0x2)
# define MPU_FIFO_RST                       (0x1 << MPU_FIFO_RST_SHIFTS)

# define MPU_I2C_MST_RST_SHIFTS             (0x1)
# define MPU_I2C_MST_RST                    (0x1 << MPU_I2C_MST_RST_SHIFTS)

# define MPU_SIG_COND_RST_SHIFTS            (0x0)
# define MPU_SIG_COND_RST                   (0x1 << MPU_SIG_COND_RST_SHIFTS)

#define MPU_PWR_MGMT_1                      (0X6B)

# define MPU_DEVICE_RESET_SHIFTS            (0x7)
# define MPU_DEVICE_RESET                   (0x1 << MPU_DEVICE_RESET_SHIFTS)

# define MPU_SLEEP_SHIFTS                   (0x6)
# define MPU_SLEEP                          (0x1 << MPU_SLEEP_SHIFTS)

# define MPU_CYCLE_SHIFTS                   (0x5)
# define MPU_CYCLE                          (0x1 << MPU_CYCLE_SHIFTS)

# define MPU_GYRO_STANDBY_SHIFTS            (0x4)
# define MPU_GYRO_STANDBY                   (0x1 << MPU_GYRO_STANDBY_SHIFTS)

# define MPU_TEMP_DISABLE_SHIFTS            (0x3)
# define MPU_TEMP_DISABLE                   (0x1 << MPU_TEMP_DISABLE_SHIFTS)

# define MPU_CLKSEL_SHIFTS                  (0x0)
# define MPU_CLKSEL_INTERNAL                (0x0 << MPU_CLKSEL_SHIFTS)
# define MPU_CLKSEL_BEST_PLL                (0x1 << MPU_CLKSEL_SHIFTS)
# define MPU_CLKSEL_STOP                    (0x7 << MPU_CLKSEL_SHIFTS)


#define MPU_PWR_MGMT_2                      (0X6C)

# define MPU_LP_WAKE_CTRL_SHIFTS            (0x6)
# define MPU_LP_WAKE_1_25_HZ                (0x0 << MPU_LP_WAKE_CTRL_SHIFTS)
# define MPU_LP_WAKE_5_HZ                   (0x1 << MPU_LP_WAKE_CTRL_SHIFTS)
# define MPU_LP_WAKE_20_HZ                  (0x2 << MPU_LP_WAKE_CTRL_SHIFTS)
# define MPU_LP_WAKE_40_HZ                  (0x3 << MPU_LP_WAKE_CTRL_SHIFTS)

# define MPU_DISABLE_XA_SHIFTS              (0x5)
# define MPU_DISABLE_XA                     (0x1 << MPU_DISABLE_XA_SHIFTS)

# define MPU_DISABLE_YA_SHIFTS              (0x4)
# define MPU_DISABLE_YA                     (0x1 << MPU_DISABLE_YA_SHIFTS)

# define MPU_DISABLE_ZA_SHIFTS              (0x3)
# define MPU_DISABLE_ZA                     (0x1 << MPU_DISABLE_ZA_SHIFTS)

# define MPU_DISABLE_XG_SHIFTS              (0x2)
# define MPU_DISABLE_XG                     (0x1 << MPU_DISABLE_XG_SHIFTS)

# define MPU_DISABLE_YG_SHIFTS              (0x1)
# define MPU_DISABLE_YG                     (0x1 << MPU_DISABLE_YG_SHIFTS)

# define MPU_DISABLE_ZG_SHIFTS              (0x0)
# define MPU_DISABLE_ZG                     (0x1 << MPU_DISABLE_ZG_SHIFTS)


#define MPU_FIFO_COUNTH                     (0X72)
#define MPU_FIFO_COUNTL                     (0X73)
#define MPU_FIFO_R_W                        (0X74)
#define MPU_WHO_AM_I                        (0X75)

#define MPU_XA_OFFSET_H                     (0x77)
#define MPU_XA_OFFSET_L                     (0x78)
#define MPU_YA_OFFSET_H                     (0x7A)
#define MPU_YA_OFFSET_L                     (0x7B)
#define MPU_ZA_OFFSET_H                     (0x7C)
#define MPU_ZA_OFFSET_L                     (0x7D)



#endif
