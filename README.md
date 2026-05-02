# BMI088

## 1. 模块作用
BMI088 IMU 驱动模块。负责传感器初始化、数据采样解析和温控输出。
Manifest 描述：博世 BMI088 6 轴惯性测量单元（IMU）的驱动模块 / Driver module for Bosch BMI088 6-axis Inertial Measurement Unit (IMU)

## 2. 时间戳约定

`bmi088_gyro` 与 `bmi088_accl` 使用同一次 gyro data-ready interrupt 采集到的时间戳发布。payload 中不再额外携带采样时间，消费者应读取 Topic envelope timestamp。

## 3. 主要函数说明
1. Init: 初始化 BMI088 寄存器与工作模式。
2. ThreadFunc: 等待 gyro INT，采样并用 gyro INT 时间戳发布陀螺仪/加速度数据。
3. RecvAccel / RecvGyro + ParseAccelData / ParseGyroData: 读取并解析原始数据。
4. ControlTemperature: 温控 PID 输出。
5. CommandFunc: 命令行查看模块状态。

## 4. 接入步骤
1. 添加模块并配置 SPI、CS、INT、加热 PWM 参数。
2. 配置输出 Topic 名称与温控参数。
3. 启动后先确认初始化成功，再检查数据更新。

标准命令流程：
    xrobot_add_mod BMI088 --instance-id bmi088
    xrobot_gen_main
    cube-cmake --build /home/leo/Documents/bsp-dev-c/build/debug --

## 5. 配置示例（YAML）
module: BMI088
entry_header: Modules/BMI088/BMI088.hpp
constructor_args:
  - gyro_freq: BMI088::GyroFreq::GYRO_2000HZ_BW532HZ
  - accl_freq: BMI088::AcclFreq::ACCL_1600HZ
  - gyro_range: BMI088::GyroRange::DEG_2000DPS
  - accl_range: BMI088::AcclRange::ACCL_24G
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - pid_param:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - gyro_topic_name: "bmi088_gyro"
  - accl_topic_name: "bmi088_accl"
  - target_temperature: 45
  - task_stack_depth: 2048
template_args:
[]

## 6. 依赖与硬件
Required Hardware:
- spi_bmi088/spi1/SPI1
- bmi088_accl_cs
- bmi088_gyro_cs
- bmi088_gyro_int
- pwm_bmi088_heat
- ramfs
- database

Depends:
[]

## 7. 代码入口
Modules/BMI088/BMI088.hpp
