# BMI088

博世 BMI088 6 轴惯性测量单元（IMU）的驱动模块 / Driver module for Bosch BMI088 6-axis Inertial Measurement Unit (IMU)

## 硬件需求 / Required Hardware

spi\_bmi088/spi1/SPI1, bmi088\_accl\_cs, bmi088\_gyro\_cs, bmi088\_gyro\_int, pwm\_bmi088\_heat, ramfs, database

## 构造参数 / Constructor Arguments

- gyro_freq:          BMI088::GyroFreq::GYRO_2000HZ_BW532HZ
- accl_freq:          BMI088::AcclFreq::ACCL_1600HZ
- gyro_range:         BMI088::GyroRange::DEG_2000DPS
- accl_range:         BMI088::AcclRange::ACCL_24G
- rotation:           {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
- pid_param:          {k: 1.0, p: 0.0, i: 0.0, d: 0.0, i_limit: 0.0, out_limit: 0.0, cycle: false}
- gyro_topic_name:    "bmi088_gyro"
- accl_topic_name:    "bmi088_accl"
- target_temperature: 45
- task_stack_depth:   2048


## 依赖 / Depends

* 无（No dependencies）
