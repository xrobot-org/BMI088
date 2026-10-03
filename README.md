# BMI088

博世 BMI088 6 轴 IMU（SPI）驱动模块 / Driver Module for the Bosch BMI088 6-axis IMU over SPI

## 1. 模块作用 / Purpose

构造时，BMI088 注册陀螺仪 INT 的下降沿中断，对加速度计和陀螺仪软复位并校验芯片 ID（加速度计 `0x1E`，陀螺仪 `0x0F`），按 `Param` 写入量程与输出频率，并把陀螺仪数据就绪中断映射到 INT3。初始化失败时输出错误日志并每 100 ms 重试，直到成功。

采样线程 `bmi088_thread`（REALTIME 优先级）等待陀螺仪中断，依次读取陀螺仪和加速度计（含温度），换算后乘以 `rotation`（传感器坐标系到应用坐标系）并发布。50 ms 内没有中断时输出日志 `BMI088 wait timeout.`。陀螺仪单位为 rad/s，发布前先减去零偏再旋转；加速度计单位为 g。原始读数全为 0 时输出不更新，保留上一次的值。

加热 PWM 以 30 kHz 运行；一个周期 50 ms 的 LibXR 定时器任务用 `pid_param` 计算占空比，使芯片温度趋近 `target_temperature`（°C）。默认 PID 的 `p`、`i`、`d` 为 0，占空比恒为 0。

陀螺仪零偏保存在 Database 的键 `bmi088_gyro_data` 中，上电时读取。

`OnMonitor()` 在数据出现 NaN 或 Inf 时输出警告，并在陀螺仪中断间隔偏离 `gyro_freq` 对应的理想周期超过 0.3 ms 时输出 `BMI088 Frequency Error`。

模块在 RamFS 中注册命令 `bmi088`：

- `bmi088`：打印用法。
- `bmi088 show <time_ms> <interval_ms>`：在 `time_ms` 内每隔 `interval_ms`（限制在 2 到 1000 ms）打印一次加速度、角速度和温度。
- `bmi088 list_offset`：打印当前陀螺仪零偏。
- `bmi088 cali`：陀螺仪零偏校准，期间设备保持静止。先等待 3 s，再采集 120 s 求平均零偏，然后采集 60 s 打印残差，最后把零偏写入 Database。

Upon construction, BMI088 registers the falling-edge interrupt of the gyroscope INT, soft-resets the accelerometer and the gyroscope and checks the chip IDs (accelerometer `0x1E`, gyroscope `0x0F`), writes the ranges and output frequencies from `Param`, and maps the gyroscope data-ready interrupt to INT3. When initialization fails, it logs an error and retries every 100 ms until it succeeds.

The sampling thread `bmi088_thread` (REALTIME priority) waits for the gyroscope interrupt, reads the gyroscope and then the accelerometer (including the temperature), converts the values, multiplies them by `rotation` (sensor frame to application frame) and publishes them. When no interrupt arrives within 50 ms, it logs `BMI088 wait timeout.`. The gyroscope unit is rad/s, with the zero offset subtracted before the rotation; the accelerometer unit is g. When the raw reading is all zeros, the output is not updated and keeps the previous value.

The heater PWM runs at 30 kHz; a LibXR timer task with a 50 ms period computes the duty cycle with `pid_param` so that the chip temperature approaches `target_temperature` (°C). With the default PID, `p`, `i` and `d` are 0 and the duty cycle stays 0.

The gyroscope zero offset is stored in the Database under the key `bmi088_gyro_data` and read at power-up.

`OnMonitor()` logs a warning when the data contains NaN or Inf, and logs `BMI088 Frequency Error` when the gyroscope interrupt interval deviates from the ideal period of `gyro_freq` by more than 0.3 ms.

The Module registers the command `bmi088` in RamFS:

- `bmi088`: print the usage.
- `bmi088 show <time_ms> <interval_ms>`: print the acceleration, angular velocity and temperature every `interval_ms` (limited to 2 to 1000 ms) for `time_ms`.
- `bmi088 list_offset`: print the current gyroscope zero offset.
- `bmi088 cali`: gyroscope zero-offset calibration, with the device held still. It waits 3 s, collects 120 s to average the zero offset, collects another 60 s to print the residual, and finally writes the zero offset to the Database.

## 2. 时间戳约定 / Timestamp Convention

`gyro_topic_name` 与 `accl_topic_name` 两个 Topic 使用同一次陀螺仪数据就绪中断采集到的时间戳（µs）发布。采样时间由 Topic 的 envelope timestamp 给出，消费者读取该时间戳。

The Topics `gyro_topic_name` and `accl_topic_name` are published with the timestamp (µs) captured at the same gyroscope data-ready interrupt. The sampling time is given by the Topic envelope timestamp, which consumers read.

## 3. 构造接口 / Constructor

```cpp
BMI088(LibXR::GPIO& accl_cs,
       LibXR::GPIO& gyro_cs,
       LibXR::GPIO& gyro_int,
       LibXR::SPI& spi,
       LibXR::PWM& heater_pwm,
       LibXR::Database& database,
       LibXR::RamFS& ramfs,
       const Param& param = {...});  // 节选 / excerpt
```

依赖：

- `accl_cs`：加速度计片选 GPIO（输出，低有效）。
- `gyro_cs`：陀螺仪片选 GPIO（输出，低有效）。
- `gyro_int`：陀螺仪 INT3 数据就绪中断 GPIO，模块将其配置为下降沿中断。
- `spi`：连接 BMI088 的 `LibXR::SPI`，片选由模块通过上面两个 GPIO 控制。
- `heater_pwm`：IMU 加热电阻的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `bmi088` 命令的 `LibXR::RamFS`。

配置参数（`Param`）：

- `gyro_freq`：陀螺仪输出频率与带宽，默认 `GYRO_2000HZ_BW532HZ`；可选 `GYRO_2000HZ_BW532HZ`、`GYRO_2000HZ_BW230HZ`、`GYRO_1000HZ_BW116HZ`、`GYRO_400HZ_BW46HZ`、`GYRO_200HZ_BW23HZ`、`GYRO_100HZ_BW12HZ`、`GYRO_200HZ_BW64HZ`、`GYRO_100HZ_BW32HZ`。
- `accl_freq`：加速度计输出频率，默认 `ACCL_1600HZ`；可选 1600、800、400、200、100、50、25、12.5 Hz（`ACCL_12_5HZ`）。
- `gyro_range`：陀螺仪量程，默认 `DEG_2000DPS`；可选 2000、1000、500、250、125 dps。
- `accl_range`：加速度计量程，默认 `ACCL_24G`；可选 3、6、12、24 g。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`，默认单位四元数。
- `pid_param`：温控 PID，`LibXR::PID<float>::Param`，字段为 `k, p, i, d, i_limit, out_limit, cycle`，默认 `k = 1`，其余为 0；输出直接作为 PWM 占空比（0.0 到 1.0）。
- `gyro_topic_name`、`accl_topic_name`：发布的 Topic 名称，默认 `"bmi088_gyro"`、`"bmi088_accl"`。
- `target_temperature`：目标温度，单位 °C，默认 45。
- `task_stack_depth`：采样线程栈深，默认 2048。

Dependencies:

- `accl_cs`: chip-select GPIO of the accelerometer (output, active low).
- `gyro_cs`: chip-select GPIO of the gyroscope (output, active low).
- `gyro_int`: GPIO of the gyroscope INT3 data-ready interrupt; the Module configures it as a falling-edge interrupt.
- `spi`: the `LibXR::SPI` connected to the BMI088; the chip selects are driven by the Module through the two GPIOs above.
- `heater_pwm`: the `LibXR::PWM` of the IMU heating resistor.
- `database`: the `LibXR::Database` that stores the gyroscope zero offset.
- `ramfs`: the `LibXR::RamFS` that receives the `bmi088` command.

Configuration parameters (`Param`):

- `gyro_freq`: gyroscope output frequency and bandwidth, default `GYRO_2000HZ_BW532HZ`; options are `GYRO_2000HZ_BW532HZ`, `GYRO_2000HZ_BW230HZ`, `GYRO_1000HZ_BW116HZ`, `GYRO_400HZ_BW46HZ`, `GYRO_200HZ_BW23HZ`, `GYRO_100HZ_BW12HZ`, `GYRO_200HZ_BW64HZ`, `GYRO_100HZ_BW32HZ`.
- `accl_freq`: accelerometer output frequency, default `ACCL_1600HZ`; options are 1600, 800, 400, 200, 100, 50, 25 and 12.5 Hz (`ACCL_12_5HZ`).
- `gyro_range`: gyroscope range, default `DEG_2000DPS`; options are 2000, 1000, 500, 250 and 125 dps.
- `accl_range`: accelerometer range, default `ACCL_24G`; options are 3, 6, 12 and 24 g.
- `rotation`: quaternion `{w, x, y, z}` from the sensor frame to the application frame, default identity.
- `pid_param`: temperature-control PID, `LibXR::PID<float>::Param` with fields `k, p, i, d, i_limit, out_limit, cycle`, default `k = 1` and all others 0; the output is used directly as the PWM duty cycle (0.0 to 1.0).
- `gyro_topic_name`, `accl_topic_name`: names of the published Topics, default `"bmi088_gyro"` and `"bmi088_accl"`.
- `target_temperature`: target temperature in °C, default 45.
- `task_stack_depth`: stack depth of the sampling thread, default 2048.

## 4. Topic

| Topic | 方向 | 类型 | 说明 |
| --- | --- | --- | --- |
| `gyro_topic_name`（默认 `bmi088_gyro`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 角速度，单位 rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `bmi088_accl`） | 发布 | `Eigen::Matrix<float, 3, 1>` | 加速度，单位 g，已旋转 |

| Topic | Direction | Type | Meaning |
| --- | --- | --- | --- |
| `gyro_topic_name` (default `bmi088_gyro`) | Publish | `Eigen::Matrix<float, 3, 1>` | Angular velocity in rad/s, zero offset removed and rotated |
| `accl_topic_name` (default `bmi088_accl`) | Publish | `Eigen::Matrix<float, 3, 1>` | Acceleration in g, rotated |

## 5. 配置示例 / Configuration Example

`xrobot instance add xrobot-org/BMI088` 写入的实例，依赖填写为 BSP 通过 `XR_REGISTER`（硬件注册）注册的名称：

An instance written by `xrobot instance add xrobot-org/BMI088`, with the dependencies set to names registered by the BSP with `XR_REGISTER` (Registration):

```yaml
modules:
  - module: xrobot-org/BMI088
    id: bmi088_0
    args:
      - accl_cs: bmi088_accl_cs
      - gyro_cs: bmi088_gyro_cs
      - gyro_int: bmi088_gyro_int
      - spi: spi1
      - heater_pwm: imu_heat_pwm
      - database: database
      - ramfs: ramfs
      - param:
          gyro_freq: BMI088::GyroFreq::GYRO_2000HZ_BW532HZ
          accl_freq: BMI088::AcclFreq::ACCL_1600HZ
          gyro_range: BMI088::GyroRange::DEG_2000DPS
          accl_range: BMI088::AcclRange::ACCL_24G
          rotation: '{1.0f, 0.0f, 0.0f, 0.0f}'
          pid_param:
            k: 1.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: false
          gyro_topic_name: "bmi088_gyro"
          accl_topic_name: "bmi088_accl"
          target_temperature: 45
          task_stack_depth: 2048
```

## 6. 依赖与硬件 / Dependencies and Hardware

依赖：LibXR。

硬件：一片通过 SPI 连接的 BMI088，加速度计与陀螺仪各有一个片选 GPIO，陀螺仪 INT3 连接到一个 GPIO，另有一路驱动加热电阻的 PWM；SPI、GPIO、PWM、Database 与 RamFS 由 BSP 通过 `XR_REGISTER` 注册。

Dependencies: LibXR.

Hardware: one BMI088 connected over SPI, with one chip-select GPIO each for the accelerometer and the gyroscope, the gyroscope INT3 wired to a GPIO, and one PWM output that drives the heating resistor; the SPI, GPIOs, PWM, Database and RamFS are registered by the BSP with `XR_REGISTER`.
