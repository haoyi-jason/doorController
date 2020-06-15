#include "bmi160.h"
#include "bmi160_defs.h"

struct bmi160_dev bmi160 = {
  .chip_id = 0xD1,
  .id = 0x1,
  .interface = BMI160_SPI_INTF,
  .accel_cfg = {BMI160_ACCEL_NORMAL_MODE,BMI160_ACCEL_ODR_100HZ,BMI160_ACCEL_RANGE_2G,BMI160_ACCEL_BW_NORMAL_AVG4},
  .gyro_cfg = {BMI160_GYRO_NORMAL_MODE,BMI160_GYRO_ODR_100HZ,BMI160_GYRO_RANGE_2000_DPS,BMI160_GYRO_BW_NORMAL_MODE},
  .read = bmi160_read,
  .write = bmi160_write,
  .delay_ms = bmi160_delay
};

bmi160_config_t bmi160_cfg = {
  .chip_id = 0xD1,
  .id = 0x1,
  .interface = BMI160_SPI_INTF,
  .accel_cfg = {BMI160_ACCEL_NORMAL_MODE,BMI160_ACCEL_ODR_1600HZ,BMI160_ACCEL_RANGE_2G,BMI160_ACCEL_BW_NORMAL_AVG4},
  .gyro_cfg = {BMI160_GYRO_NORMAL_MODE,BMI160_GYRO_ODR_1600HZ,BMI160_GYRO_RANGE_2000_DPS,BMI160_GYRO_BW_NORMAL_MODE},    
  .offsets = {0,0,0,0,0,0},
};