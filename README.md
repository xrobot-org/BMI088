# BMI088

博世 BMI088 6 轴 IMU 的 SPI 驱动模块：初始化传感器、按 gyro data-ready 中断采样、
发布陀螺仪与加速度计数据，并通过加热 PWM 做恒温控制。

## 行为

- 构造时注册 gyro INT 下降沿中断，对加速度计和陀螺仪软复位、校验芯片 ID
  （accl `0x1E`，gyro `0x0F`），按 `Param` 写入量程与输出频率，并把 gyro data-ready
  映射到 INT3。初始化失败时打印错误并每 100 ms 重试，直到成功。
- 采样线程 `bmi088_thread`（REALTIME 优先级）等待 gyro 中断，依次读取陀螺仪和加速度计
  （含温度），换算后乘以 `rotation`（传感器坐标系到应用坐标系）并发布。50 ms 内没有中断时
  打印 `BMI088 wait timeout.`。
- 陀螺仪单位 rad/s，发布前先减去零偏再旋转；加速度计单位 g。全零的原始读数会被丢弃，
  保留上一次的值。
- 加热 PWM 以 30 kHz 运行；一个 50 ms 周期的 LibXR 定时器任务用 `pid_param` 计算占空比，
  使芯片温度趋近 `target_temperature`（°C）。默认 PID 参数的增益为 0，即不加热；
  需要恒温时请按板子调好 PID。
- 陀螺仪零偏保存在 Database 键 `bmi088_gyro_data` 中，上电时读取。
- `OnMonitor()` 在数据出现 NaN/Inf 时告警，并在 gyro 中断间隔偏离 `gyro_freq`
  理想周期超过 0.3 ms 时打印 `BMI088 Frequency Error`。

## 时间戳约定

`gyro_topic_name` 与 `accl_topic_name` 两个 Topic 使用同一次 gyro data-ready 中断采集到的
时间戳（µs）发布。payload 中不携带采样时间，消费者应读取 Topic envelope timestamp。

## Topic

| Topic | 类型 | 内容 |
| --- | --- | --- |
| `gyro_topic_name`（默认 `bmi088_gyro`） | `Eigen::Matrix<float, 3, 1>` | 角速度，rad/s，已去零偏并旋转 |
| `accl_topic_name`（默认 `bmi088_accl`） | `Eigen::Matrix<float, 3, 1>` | 加速度，g，已旋转 |

## RamFS 命令

模块在 RamFS 中注册 `bmi088` 命令：

- `bmi088`：打印用法。
- `bmi088 show <time_ms> <interval_ms>`：每 `interval_ms`（限制在 2-1000 ms）打印一次
  加速度、角速度和温度，持续 `time_ms`。
- `bmi088 list_offset`：打印当前陀螺仪零偏。
- `bmi088 cali`：陀螺仪零偏校准。设备需保持静止：等待 3 s 后采集 120 s 求平均零偏，
  再采集 60 s 打印残差，最后把零偏写入 Database。

## 依赖

无其他模块依赖，仅使用 LibXR。

## 构造接口

```cpp
BMI088(LibXR::GPIO& accl_cs,
       LibXR::GPIO& gyro_cs,
       LibXR::GPIO& gyro_int,
       LibXR::SPI& spi,
       LibXR::PWM& heater_pwm,
       LibXR::Database& database,
       LibXR::RamFS& ramfs,
       const Param& param = {...});
```

依赖：

- `accl_cs`：加速度计片选 GPIO（输出，低有效）。
- `gyro_cs`：陀螺仪片选 GPIO（输出，低有效）。
- `gyro_int`：陀螺仪 INT3 数据就绪中断 GPIO，模块将其配置为下降沿中断。
- `spi`：连接 BMI088 的 `LibXR::SPI`，片选由模块通过上面两个 GPIO 控制。
- `heater_pwm`：IMU 加热电阻的 `LibXR::PWM`。
- `database`：保存陀螺仪零偏的 `LibXR::Database`。
- `ramfs`：注册 `bmi088` 命令的 `LibXR::RamFS`。

配置（`Param` 字段，括号内为默认值）：

- `gyro_freq`：陀螺仪输出频率 / 带宽（`GYRO_2000HZ_BW532HZ`），可选
  `GYRO_2000HZ_BW532HZ`、`GYRO_2000HZ_BW230HZ`、`GYRO_1000HZ_BW116HZ`、
  `GYRO_400HZ_BW46HZ`、`GYRO_200HZ_BW23HZ`、`GYRO_100HZ_BW12HZ`、
  `GYRO_200HZ_BW64HZ`、`GYRO_100HZ_BW32HZ`。
- `accl_freq`：加速度计输出频率（`ACCL_1600HZ`），可选 1600 / 800 / 400 / 200 / 100 /
  50 / 25 / 12.5 Hz（`ACCL_12_5HZ`）。
- `gyro_range`：陀螺仪量程（`DEG_2000DPS`），可选 2000 / 1000 / 500 / 250 / 125 dps。
- `accl_range`：加速度计量程（`ACCL_24G`），可选 3 / 6 / 12 / 24 g。
- `rotation`：传感器坐标系到应用坐标系的四元数 `{w, x, y, z}`（单位四元数）。
- `pid_param`：温控 PID 参数 `LibXR::PID<float>::Param`（`k = 1`，其余为 0）。
  输出直接作为 PWM 占空比（0.0-1.0）。
- `gyro_topic_name` / `accl_topic_name`：Topic 名称（`"bmi088_gyro"` / `"bmi088_accl"`）。
- `target_temperature`：目标温度，°C（45）。
- `task_stack_depth`：采样线程栈深（2048）。

## 使用

```sh
xrobot module add xrobot-org/BMI088
xrobot setup
xrobot instance add xrobot-org/BMI088
```

`xrobot instance add` 在 `User/xrobot.yaml` 中写入一个实例，依赖项留空，默认值按源码写出；
把依赖项填为 BSP 中用 `XR_REGISTER` 注册的对象名：

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
          rotation:
            - 1.0f
            - 0.0f
            - 0.0f
            - 0.0f
          pid_param:
            k: 1.0f
            p: 0.0f
            i: 0.0f
            d: 0.0f
            i_limit: 0.0f
            out_limit: 0.0f
            cycle: 'false'
          gyro_topic_name: '"bmi088_gyro"'
          accl_topic_name: '"bmi088_accl"'
          target_temperature: '45'
          task_stack_depth: '2048'
```

BSP 侧：

```cpp
XR_REGISTER(bmi088_accl_cs, LibXR::GPIO);
XR_REGISTER(bmi088_gyro_cs, LibXR::GPIO);
XR_REGISTER(bmi088_gyro_int, LibXR::GPIO);
XR_REGISTER(spi1, LibXR::SPI);
XR_REGISTER(imu_heat_pwm, LibXR::PWM);
XR_REGISTER(database, LibXR::Database);
XR_REGISTER(ramfs, LibXR::RamFS);
```

填好后再次运行 `xrobot setup`，生成 `User/xrobot_main.hpp`。

`xrobot module show .`（在本仓库中）或 `xrobot module show Modules/xrobot-org/BMI088`
（在 BSP 中）打印 manifest 和当前的构造函数。

旋转四元数可以用 <https://www.andre-gaschler.com/rotationconverter/> 计算。
