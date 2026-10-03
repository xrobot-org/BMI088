#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 博世 BMI088 6 轴 IMU（SPI）驱动模块 / Driver Module for the Bosch BMI088 6-axis IMU over SPI
depends: []
=== END MANIFEST === */
// clang-format on

#include <memory>

#include "database.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "ramfs.hpp"
#include "spi.hpp"
#include "thread.hpp"
#include "transform.hpp"

#define BMI088_REG_ACCL_CHIP_ID (0x00)
#define BMI088_REG_ACCL_ERR (0x02)
#define BMI088_REG_ACCL_STATUS (0x03)
#define BMI088_REG_ACCL_X_LSB (0x12)
#define BMI088_REG_ACCL_X_MSB (0x13)
#define BMI088_REG_ACCL_Y_LSB (0x14)
#define BMI088_REG_ACCL_Y_MSB (0x15)
#define BMI088_REG_ACCL_Z_LSB (0x16)
#define BMI088_REG_ACCL_Z_MSB (0x17)
#define BMI088_REG_ACCL_SENSORTIME_0 (0x18)
#define BMI088_REG_ACCL_SENSORTIME_1 (0x19)
#define BMI088_REG_ACCL_SENSORTIME_2 (0x1A)
#define BMI088_REG_ACCL_INT_STAT_1 (0x1D)
#define BMI088_REG_ACCL_TEMP_MSB (0x22)
#define BMI088_REG_ACCL_TEMP_LSB (0x23)
#define BMI088_REG_ACCL_CONF (0x40)
#define BMI088_REG_ACCL_RANGE (0x41)
#define BMI088_REG_ACCL_INT1_IO_CONF (0x53)
#define BMI088_REG_ACCL_INT2_IO_CONF (0x54)
#define BMI088_REG_ACCL_INT1_INT2_MAP_DATA (0x58)
#define BMI088_REG_ACCL_SELF_TEST (0x6D)
#define BMI088_REG_ACCL_PWR_CONF (0x7C)
#define BMI088_REG_ACCL_PWR_CTRL (0x7D)
#define BMI088_REG_ACCL_SOFTRESET (0x7E)

#define BMI088_REG_GYRO_CHIP_ID (0x00)
#define BMI088_REG_GYRO_X_LSB (0x02)
#define BMI088_REG_GYRO_X_MSB (0x03)
#define BMI088_REG_GYRO_Y_LSB (0x04)
#define BMI088_REG_GYRO_Y_MSB (0x05)
#define BMI088_REG_GYRO_Z_LSB (0x06)
#define BMI088_REG_GYRO_Z_MSB (0x07)
#define BMI088_REG_GYRO_INT_STAT_1 (0x0A)
#define BMI088_REG_GYRO_RANGE (0x0F)
#define BMI088_REG_GYRO_BANDWIDTH (0x10)
#define BMI088_REG_GYRO_LPM1 (0x11)
#define BMI088_REG_GYRO_SOFTRESET (0x14)
#define BMI088_REG_GYRO_INT_CTRL (0x15)
#define BMI088_REG_GYRO_INT3_INT4_IO_CONF (0x16)
#define BMI088_REG_GYRO_INT3_INT4_IO_MAP (0x18)
#define BMI088_REG_GYRO_SELF_TEST (0x3C)

#define BMI088_CHIP_ID_ACCL (0x1E)
#define BMI088_CHIP_ID_GYRO (0x0F)

#define BMI088_ACCL_RX_BUFF_LEN (19)
#define BMI088_GYRO_RX_BUFF_LEN (6)

/**
 * @brief BMI088 6 轴 IMU 驱动模块，负责初始化、数据采集、温控与 Topic 发布。
 *        Driver Module for the BMI088 6-axis IMU: initialization, data acquisition,
 *        temperature control and Topic publishing.
 */
class BMI088
{
 public:
  /**
   * @brief SPI 总线上的传感器。
   *        Sensor on the SPI bus.
   */
  enum class Device : uint8_t
  {
    ACCELMETER,  ///< 加速度计
                 ///< Accelerometer
    GYROSCOPE    ///< 陀螺仪
                 ///< Gyroscope
  };

  /**
   * @brief 陀螺仪量程。
   *        Gyroscope range.
   */
  enum class GyroRange : uint8_t
  {
    DEG_2000DPS = 0x00,  ///< ±2000 dps
    DEG_1000DPS = 0x01,  ///< ±1000 dps
    DEG_500DPS = 0x02,   ///< ±500 dps
    DEG_250DPS = 0x03,   ///< ±250 dps
    DEG_125DPS = 0x04    ///< ±125 dps
  };

  /**
   * @brief 加速度计量程。
   *        Accelerometer range.
   */
  enum class AcclRange : uint8_t
  {
    ACCL_3G = 0x00,   ///< ±3 g
    ACCL_6G = 0x01,   ///< ±6 g
    ACCL_12G = 0x02,  ///< ±12 g
    ACCL_24G = 0x03   ///< ±24 g
  };

  /**
   * @brief 陀螺仪输出频率与带宽。
   *        Gyroscope output data rate and bandwidth.
   */
  enum class GyroFreq : uint8_t
  {
    GYRO_2000HZ_BW532HZ = 0x00,  ///< 2000 Hz，带宽 532 Hz
                                 ///< 2000 Hz, bandwidth 532 Hz
    GYRO_2000HZ_BW230HZ = 0x01,  ///< 2000 Hz，带宽 230 Hz
                                 ///< 2000 Hz, bandwidth 230 Hz
    GYRO_1000HZ_BW116HZ = 0x02,  ///< 1000 Hz，带宽 116 Hz
                                 ///< 1000 Hz, bandwidth 116 Hz
    GYRO_400HZ_BW46HZ = 0x03,    ///< 400 Hz，带宽 46 Hz
                                 ///< 400 Hz, bandwidth 46 Hz
    GYRO_200HZ_BW23HZ = 0x04,    ///< 200 Hz，带宽 23 Hz
                                 ///< 200 Hz, bandwidth 23 Hz
    GYRO_100HZ_BW12HZ = 0x05,    ///< 100 Hz，带宽 12 Hz
                                 ///< 100 Hz, bandwidth 12 Hz
    GYRO_200HZ_BW64HZ = 0x06,    ///< 200 Hz，带宽 64 Hz
                                 ///< 200 Hz, bandwidth 64 Hz
    GYRO_100HZ_BW32HZ = 0x07,    ///< 100 Hz，带宽 32 Hz
                                 ///< 100 Hz, bandwidth 32 Hz
  };

  /**
   * @brief 加速度计输出频率。
   *        Accelerometer output data rate.
   */
  enum class AcclFreq : uint8_t
  {
    ACCL_1600HZ = 0x0C,  ///< 1600 Hz
    ACCL_800HZ = 0x0B,   ///< 800 Hz
    ACCL_400HZ = 0x0A,   ///< 400 Hz
    ACCL_200HZ = 0x09,   ///< 200 Hz
    ACCL_100HZ = 0x08,   ///< 100 Hz
    ACCL_50HZ = 0x07,    ///< 50 Hz
    ACCL_25HZ = 0x06,    ///< 25 Hz
    ACCL_12_5HZ = 0x05   ///< 12.5 Hz
  };

  /// 角度到弧度的换算系数 (rad/deg)
  /// Degree-to-radian factor (rad/deg)
  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;

  /**
   * @brief 拉低指定传感器的片选。
   *        Assert the chip select of the given sensor.
   *
   * @param device 目标传感器。
   *               Target sensor.
   */
  void Select(Device device)
  {
    if (device == Device::ACCELMETER)
    {
      cs_accl_->Write(false);
    }
    else
    {
      cs_gyro_->Write(false);
    }
  }

  /**
   * @brief 释放指定传感器的片选。
   *        Release the chip select of the given sensor.
   *
   * @param device 目标传感器。
   *               Target sensor.
   */
  void Deselect(Device device)
  {
    if (device == Device::ACCELMETER)
    {
      cs_accl_->Write(true);
    }
    else
    {
      cs_gyro_->Write(true);
    }
  }

  /**
   * @brief 写一个寄存器，并在写后等待 1 ms。
   *        Write one register, then wait 1 ms.
   *
   * @param device 目标传感器。
   *               Target sensor.
   * @param reg 寄存器地址。
   *            Register address.
   * @param data 写入的值。
   *             Value to write.
   */
  void WriteSingle(Device device, uint8_t reg, uint8_t data)
  {
    Select(device);
    spi_->MemWrite(reg, data, op_spi_);
    Deselect(device);

    /* For accelmeter, two write operations need at least 2us */
    LibXR::Thread::Sleep(1);
  }

  /**
   * @brief 读一个寄存器；加速度计读取的首个字节为占位字节，返回其后的数据字节。
   *        Read one register; the first byte of an accelerometer read is a dummy byte
   *        and the following data byte is returned.
   *
   * @param device 目标传感器。
   *               Target sensor.
   * @param reg 寄存器地址。
   *            Register address.
   * @return 寄存器的值。
   *         Register value.
   */
  uint8_t ReadSingle(Device device, uint8_t reg)
  {
    Select(device);
    spi_->MemRead(reg, {rw_buffer_, 2}, op_spi_);
    Deselect(device);

    if (device == Device::ACCELMETER)
    {
      return rw_buffer_[1];
    }
    else
    {
      return rw_buffer_[0];
    }
  }

  /**
   * @brief 从起始寄存器连续读取 len 字节到内部缓冲区。
   *        Read len bytes from the start register into the internal buffer.
   *
   * @param device 目标传感器。
   *               Target sensor.
   * @param reg 起始寄存器地址。
   *            Start register address.
   * @param len 读取字节数。
   *            Number of bytes to read.
   */
  void Read(Device device, uint8_t reg, uint8_t len)
  {
    Select(device);
    spi_->MemRead(reg, {rw_buffer_, len}, op_spi_);
    Deselect(device);
  }

  /**
   * @brief BMI088 配置参数。
   *        BMI088 configuration parameters.
   */
  struct Param
  {
    GyroFreq gyro_freq;                 ///< 陀螺仪输出频率与带宽
                                        ///< Gyroscope output data rate and bandwidth
    AcclFreq accl_freq;                 ///< 加速度计输出频率
                                        ///< Accelerometer output data rate
    GyroRange gyro_range;               ///< 陀螺仪量程
                                        ///< Gyroscope range
    AcclRange accl_range;               ///< 加速度计量程
                                        ///< Accelerometer range
    LibXR::Quaternion<float> rotation;  ///< 传感器到应用坐标系的四元数 (w, x, y, z)
    ///< Quaternion (w, x, y, z), sensor to application frame
    LibXR::PID<float>::Param pid_param;  ///< 温控 PID，输出为 PWM 占空比 (0.0-1.0)
    ///< Temperature PID, output is the PWM duty cycle (0.0-1.0)
    const char* gyro_topic_name;  ///< 陀螺仪 Topic 名称
    ///< Gyroscope Topic name
    const char* accl_topic_name;  ///< 加速度计 Topic 名称
    ///< Accelerometer Topic name
    float target_temperature;  ///< 目标温度 (°C)
    ///< Target temperature (°C)
    size_t task_stack_depth;  ///< 采样线程栈深
    ///< Sampling thread stack depth
  };

  /**
   * @brief 构造 BMI088：注册中断与 RamFS 命令，初始化传感器，创建采样线程与温控任务。
   *        Construct BMI088: register the interrupt and the RamFS command, initialize the
   *        sensors, and create the sampling thread and the temperature-control task.
   *
   * @param accl_cs 加速度计片选 GPIO。
   *                Accelerometer chip-select GPIO.
   * @param gyro_cs 陀螺仪片选 GPIO。
   *                Gyroscope chip-select GPIO.
   * @param gyro_int 陀螺仪 INT3 数据就绪中断 GPIO。
   *                 Gyroscope INT3 data-ready interrupt GPIO.
   * @param spi 连接 BMI088 的 SPI。
   *            SPI connected to the BMI088.
   * @param heater_pwm 加热电阻的 PWM。
   *                   PWM of the heating resistor.
   * @param database 保存陀螺仪零偏的 Database。
   *                 Database that stores the gyroscope zero offset.
   * @param ramfs 接收 `bmi088` 命令的 RamFS。
   *              RamFS that receives the `bmi088` command.
   * @param param 配置参数。
   *              Configuration parameters.
   */
  BMI088(
      LibXR::GPIO& accl_cs,
      LibXR::GPIO& gyro_cs,
      LibXR::GPIO& gyro_int,
      LibXR::SPI& spi,
      LibXR::PWM& heater_pwm,
      LibXR::Database& database,
      LibXR::RamFS& ramfs,
      const Param& param = {.gyro_freq = BMI088::GyroFreq::GYRO_2000HZ_BW532HZ, .accl_freq = BMI088::AcclFreq::ACCL_1600HZ, .gyro_range = BMI088::GyroRange::DEG_2000DPS, .accl_range = BMI088::AcclRange::ACCL_24G, .rotation = {1.0f, 0.0f, 0.0f, 0.0f}, .pid_param = {.k = 1.0f, .p = 0.0f, .i = 0.0f, .d = 0.0f, .i_limit = 0.0f, .out_limit = 0.0f, .cycle = false}, .gyro_topic_name = "bmi088_gyro", .accl_topic_name = "bmi088_accl", .target_temperature = 45, .task_stack_depth = 2048})
      : gyro_range_(param.gyro_range),
        accel_range_(param.accl_range),
        gyro_freq_(param.gyro_freq),
        accl_freq_(param.accl_freq),
        target_temperature_(param.target_temperature),
        topic_gyro_(LibXR::Topic::CreateTopic<decltype(gyro_data_)>(param.gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<decltype(accl_data_)>(param.accl_topic_name)),
        cs_accl_(std::addressof(accl_cs)),
        cs_gyro_(std::addressof(gyro_cs)),
        int_gyro_(std::addressof(gyro_int)),
        spi_(std::addressof(spi)),
        pwm_(std::addressof(heater_pwm)),
        rotation_(std::move(param.rotation)),
        pid_heat_(param.pid_param),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("bmi088", CommandFunc, this)),
        gyro_data_key_(database, "bmi088_gyro_data",
                       Eigen::Matrix<float, 3, 1>(0.0, 0.0, 0.0))
  {
    ramfs.Add(cmd_file_);

    int_gyro_->DisableInterrupt();

    auto gyro_int_cb = LibXR::GPIO::Callback::Create(
        [](bool in_isr, BMI088* bmi088)
        {
          auto timestamp = LibXR::Timebase::GetMicroseconds();
          bmi088->dt_gyro_ = timestamp - bmi088->last_gyro_int_time_;
          bmi088->last_gyro_int_time_ = timestamp;
          bmi088->sample_timestamp_ = timestamp;

          bmi088->new_data_.PostFromCallback(in_isr);
        },
        this);

    int_gyro_->SetConfig({.direction = LibXR::GPIO::Direction::FALL_INTERRUPT,
                          .pull = LibXR::GPIO::Pull::NONE});

    int_gyro_->RegisterCallback(gyro_int_cb);

    while (!Init())
    {
      XR_LOG_ERROR("BMI088: Init failed. Try again.");
      LibXR::Thread::Sleep(100);
    }

    XR_LOG_PASS("BMI088: Init succeeded.");

    thread_.Create(this, ThreadFunc, "bmi088_thread", param.task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);

    void (*temp_ctrl_func)(BMI088*) = [](BMI088* bmi088)
    { bmi088->ControlTemperature(0.05f); };

    auto temp_ctrl_task = LibXR::Timer::CreateTask(temp_ctrl_func, this, 50);

    LibXR::Timer::Add(temp_ctrl_task);
    LibXR::Timer::Start(temp_ctrl_task);
  }

  /**
   * @brief 软复位两个传感器，校验芯片 ID，并写入量程、频率与数据就绪中断配置。
   *        Soft-reset both sensors, check the chip IDs, and write the range, frequency
   *        and data-ready interrupt configuration.
   *
   * @return 初始化成功返回 true，芯片 ID 不符时返回 false。
   *         True on success, false when a chip ID does not match.
   */
  bool Init()
  {
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_SOFTRESET, 0xB6);
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_SOFTRESET, 0xB6);

    LibXR::Thread::Sleep(30);

    /* Need to read chip id twice */
    ReadSingle(Device::ACCELMETER, BMI088_REG_ACCL_CHIP_ID);
    ReadSingle(Device::GYROSCOPE, BMI088_REG_GYRO_CHIP_ID);

    auto accl_id = ReadSingle(Device::ACCELMETER, BMI088_REG_ACCL_CHIP_ID);
    auto gyro_id = ReadSingle(Device::GYROSCOPE, BMI088_REG_GYRO_CHIP_ID);

    if (accl_id != BMI088_CHIP_ID_ACCL)
    {
      return false;
    }
    if (gyro_id != BMI088_CHIP_ID_GYRO)
    {
      return false;
    }

    /* Accl init. */
    /* Filter setting: OSR4. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_CONF,
                0x80 | static_cast<uint8_t>(accl_freq_));

    /* 0x00: +-3G. 0x01: +-6G. 0x02: +-12G. 0x03: +-24G. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_RANGE,
                static_cast<uint8_t>(accel_range_));

    /* Turn on accl. Now we can read data. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_PWR_CTRL, 0x04);
    LibXR::Thread::Sleep(50);

    /* Gyro init. */
    /* 0x00: +-2000. 0x01: +-1000. 0x02: +-500. 0x03: +-250. 0x04: +-125. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_RANGE,
                static_cast<uint8_t>(gyro_range_));

    /* ODR: 0x02: 1000Hz. 0x03: 400Hz. 0x06: 200Hz. 0x07: 100Hz. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_BANDWIDTH,
                static_cast<uint8_t>(gyro_freq_));

    /* INT3 and INT4 as output. Push-pull. Active low. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_INT3_INT4_IO_CONF, 0x00);

    /* Map data ready interrupt to INT3. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_INT3_INT4_IO_MAP, 0x01);

    /* Enable new data interrupt. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_INT_CTRL, 0x80);

    LibXR::Thread::Sleep(50);
    int_gyro_->EnableInterrupt();

    return true;
  }

  /**
   * @brief 监控回调：数据含 NaN 或 Inf 时输出警告；陀螺仪中断间隔偏离理想周期超过
   *        0.3 ms 时输出频率错误警告。
   *        Monitor callback: log a warning when the data contains NaN or Inf, and a
   *        frequency-error warning when the gyroscope interrupt interval deviates from
   *        the ideal period by more than 0.3 ms.
   */
  void OnMonitor(void)
  {
    if (std::isinf(gyro_data_.x()) || std::isinf(gyro_data_.y()) ||
        std::isinf(gyro_data_.z()) || std::isinf(accl_data_.x()) ||
        std::isinf(accl_data_.y()) || std::isinf(accl_data_.z()) ||
        std::isnan(gyro_data_.x()) || std::isnan(gyro_data_.y()) ||
        std::isnan(gyro_data_.z()) || std::isnan(accl_data_.x()) ||
        std::isnan(accl_data_.y()) || std::isnan(accl_data_.z()))
    {
      XR_LOG_WARN("BMI088: NaN data detected. gyro: %f %f %f, accl: %f %f %f",
                  gyro_data_.x(), gyro_data_.y(), gyro_data_.z(), accl_data_.x(),
                  accl_data_.y(), accl_data_.z());
    }

    float ideal_gyro_dt = 0.0f;
    switch (gyro_freq_)
    {
      case GyroFreq::GYRO_2000HZ_BW532HZ:
      case GyroFreq::GYRO_2000HZ_BW230HZ:
        ideal_gyro_dt = 0.0005f;
        break;
      case GyroFreq::GYRO_1000HZ_BW116HZ:
        ideal_gyro_dt = 0.001f;
        break;
      case GyroFreq::GYRO_400HZ_BW46HZ:
        ideal_gyro_dt = 0.0025f;
        break;
      case GyroFreq::GYRO_200HZ_BW23HZ:
      case GyroFreq::GYRO_200HZ_BW64HZ:
        ideal_gyro_dt = 0.005f;
        break;
      case GyroFreq::GYRO_100HZ_BW12HZ:
      case GyroFreq::GYRO_100HZ_BW32HZ:
        ideal_gyro_dt = 0.01f;
        break;
    }

    if (std::fabs(ideal_gyro_dt - dt_gyro_.ToSecondf()) > 0.0003f)
    {
      XR_LOG_WARN("BMI088 Frequency Error: %6f", dt_gyro_.ToSecondf());
    }
  }

  /**
   * @brief 采样线程：启动加热 PWM，等待陀螺仪中断，读取并发布陀螺仪与加速度计数据。
   *        Sampling thread: start the heater PWM, wait for the gyroscope interrupt, then
   *        read and publish the gyroscope and accelerometer data.
   *
   * @param bmi088 BMI088 实例。
   *               BMI088 instance.
   */
  static void ThreadFunc(BMI088* bmi088)
  {
    /* Start PWM */
    bmi088->pwm_->SetConfig({30000});
    bmi088->pwm_->SetDutyCycle(0);
    bmi088->pwm_->Enable();

    while (true)
    {
      if (bmi088->new_data_.Wait(50) == LibXR::ErrorCode::OK)
      {
        const auto sample_timestamp = bmi088->sample_timestamp_;

        bmi088->RecvGyro();
        bmi088->ParseGyroData();
        bmi088->RecvAccel();
        bmi088->ParseAccelData();
        bmi088->topic_accl_.Publish(bmi088->accl_data_, sample_timestamp);
        bmi088->topic_gyro_.Publish(bmi088->gyro_data_, sample_timestamp);
      }
      else
      {
        XR_LOG_WARN("BMI088 wait timeout.");
      }
    }
  }

  /**
   * @brief 温控步骤：用 PID 计算加热 PWM 占空比。
   *        Temperature-control step: compute the heater PWM duty cycle with the PID.
   *
   * @param dt 控制周期，单位 s。
   *           Control period in s.
   */
  void ControlTemperature(float dt)
  {
    auto duty_cycle = pid_heat_.Calculate(target_temperature_, temperature_, dt);
    pwm_->SetDutyCycle(duty_cycle);
  }

  /**
   * @brief 读取加速度计原始数据（含温度）到内部缓冲区。
   *        Read the raw accelerometer data (including the temperature) into the internal
   *        buffer.
   */
  void RecvAccel(void)
  {
    Read(Device::ACCELMETER, BMI088_REG_ACCL_X_LSB, BMI088_ACCL_RX_BUFF_LEN);
  }

  /**
   * @brief 读取陀螺仪原始数据到内部缓冲区。
   *        Read the raw gyroscope data into the internal buffer.
   */
  void RecvGyro(void)
  {
    Read(Device::GYROSCOPE, BMI088_REG_GYRO_X_LSB, BMI088_GYRO_RX_BUFF_LEN);
  }

  /**
   * @brief 当前加速度计量程下一个 LSB 对应的加速度。
   *        Acceleration represented by one LSB at the current accelerometer range.
   *
   * @return 单位 g/LSB。
   *         Value in g/LSB.
   */
  float GetAcclLSB(void)
  {
    switch (accel_range_)
    {
      case AcclRange::ACCL_24G:
        return 1.0 / 1365.0;
        break;

      case AcclRange::ACCL_12G:
        return 1.0 / 2730.0;
        break;

      case AcclRange::ACCL_6G:
        return 1.0 / 5460.0;
        break;

      case AcclRange::ACCL_3G:
        return 1.0 / 10920.0;
        break;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 解析缓冲区中的加速度与温度；三轴原始值全为 0 时不更新加速度。
   *        Parse the acceleration and temperature in the buffer; the acceleration is not
   *        updated when all three raw axes are 0.
   */
  void ParseAccelData(void)
  {
    std::array<int16_t, 3> raw_int16;
    std::array<float, 3> raw;

    float range = GetAcclLSB();

    for (int i = 0; i < 3; i++)
    {
      raw_int16[i] =
          static_cast<int16_t>((static_cast<uint8_t>(rw_buffer_[i * 2 + 2]) << 8) |
                               static_cast<uint8_t>(rw_buffer_[i * 2 + 1]));
      raw[i] = static_cast<float>(raw_int16[i]) * range;
    }

    int16_t raw_temp = static_cast<int16_t>((static_cast<uint8_t>(rw_buffer_[17]) << 3) |
                                            (static_cast<uint8_t>(rw_buffer_[18]) >> 5));
    if (raw_temp > 1023)
    {
      raw_temp -= 2048;
    }

    temperature_ = static_cast<float>(raw_temp) * 0.125f + 23.0f;

    if (raw[0] == 0.0f && raw[1] == 0.0f && raw[2] == 0.0f)
    {
      return;
    }

    accl_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(raw[0], raw[1], raw[2]);
  }

  /**
   * @brief 当前陀螺仪量程下一个 LSB 对应的角速度。
   *        Angular velocity represented by one LSB at the current gyroscope range.
   *
   * @return 单位 dps/LSB。
   *         Value in dps/LSB.
   */
  float GetGyroLSB()
  {
    switch (gyro_range_)
    {
      case GyroRange::DEG_2000DPS:
        return 1.0 / 16.384;
        break;
      case GyroRange::DEG_1000DPS:
        return 1.0 / 32.768;
        break;
      case GyroRange::DEG_500DPS:
        return 1.0 / 65.536;
        break;
      case GyroRange::DEG_250DPS:
        return 1.0 / 131.072;
        break;
      case GyroRange::DEG_125DPS:
        return 1.0 / 262.144;
        break;
      default:
        return 0.0f;
    }
  }

  /**
   * @brief 解析缓冲区中的角速度，减去零偏并旋转；三轴原始值全为 0 时不更新角速度。
   *        Parse the angular velocity in the buffer, subtract the zero offset and rotate
   *        it; the angular velocity is not updated when all three raw axes are 0.
   */
  void ParseGyroData(void)
  {
    std::array<int16_t, 3> raw_int16;
    std::array<float, 3> raw;
    float range = GetGyroLSB();

    for (int i = 0; i < 3; i++)
    {
      raw_int16[i] =
          static_cast<int16_t>((static_cast<uint8_t>(rw_buffer_[i * 2 + 1]) << 8) |
                               static_cast<uint8_t>(rw_buffer_[i * 2]));
      raw[i] = static_cast<float>(raw_int16[i]) * range * M_DEG2RAD_MULT;
    }

    if (in_cali_)
    {
      gyro_cali_.data()[0] += raw_int16[0];
      gyro_cali_.data()[1] += raw_int16[1];
      gyro_cali_.data()[2] += raw_int16[2];
      cali_counter_++;
    }

    if (raw[0] == 0.0f && raw[1] == 0.0f && raw[2] == 0.0f)
    {
      return;
    }

    gyro_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(
                                 Eigen::Matrix<float, 3, 1>(raw[0], raw[1], raw[2]) -
                                 gyro_data_key_.data_);
  }

 private:
  static int CommandFunc(BMI088* bmi088, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf<"Usage:\r\n">();
      LibXR::STDIO::Printf<
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n">();
      LibXR::STDIO::Printf<
          "  list_offset                  - Show current gyro calibration "
          "offset.\r\n">();
      LibXR::STDIO::Printf<
          "  cali                         - Start gyroscope "
          "calibration.\r\n">();
    }
    else if (argc == 2)
    {
      if (strcmp(argv[1], "list_offset") == 0)
      {
        LibXR::STDIO::Printf<"Current calibration offset - x: %f, y: %f, z: %f\r\n">(
            bmi088->gyro_data_key_.data_.x(), bmi088->gyro_data_key_.data_.y(),
            bmi088->gyro_data_key_.data_.z());
      }
      else if (strcmp(argv[1], "cali") == 0)
      {
        bmi088->gyro_data_key_.data_.x() = 0.0, bmi088->gyro_data_key_.data_.y() = 0.0,
        bmi088->gyro_data_key_.data_.z() = 0.0;
        bmi088->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0.0, 0.0, 0.0);
        bmi088->cali_counter_ = 0;
        bmi088->in_cali_ = true;
        LibXR::STDIO::Printf<
            "Starting gyroscope calibration. Please keep the device "
            "steady.\r\n">();
        LibXR::Thread::Sleep(3000);
        for (int i = 0; i < 120; i++)
        {
          LibXR::STDIO::Printf<"Progress: %d / 120\r">(i);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf<"\r\nProgress: Done\r\n">();
        bmi088->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        bmi088->gyro_data_key_.data_.x() =
            static_cast<float>(static_cast<double>(bmi088->gyro_cali_.data()[0]) /
                               static_cast<double>(bmi088->cali_counter_) *
                               bmi088->GetGyroLSB() * M_DEG2RAD_MULT);
        bmi088->gyro_data_key_.data_.y() =
            static_cast<float>(static_cast<double>(bmi088->gyro_cali_.data()[1]) /
                               static_cast<double>(bmi088->cali_counter_) *
                               bmi088->GetGyroLSB() * M_DEG2RAD_MULT);
        bmi088->gyro_data_key_.data_.z() =
            static_cast<float>(static_cast<double>(bmi088->gyro_cali_.data()[2]) /
                               static_cast<double>(bmi088->cali_counter_) *
                               bmi088->GetGyroLSB() * M_DEG2RAD_MULT);

        LibXR::STDIO::Printf<"\r\nCalibration result - x: %f, y: %f, z: %f\r\n">(
            bmi088->gyro_data_key_.data_.x(), bmi088->gyro_data_key_.data_.y(),
            bmi088->gyro_data_key_.data_.z());

        LibXR::STDIO::Printf<"Analyzing calibration quality...\r\n">();
        bmi088->gyro_cali_ = Eigen::Matrix<int64_t, 3, 1>(0.0, 0.0, 0.0);
        bmi088->cali_counter_ = 0;
        bmi088->in_cali_ = true;
        for (int i = 0; i < 60; i++)
        {
          LibXR::STDIO::Printf<"Progress: %d / 60\r">(i);
          LibXR::Thread::Sleep(1000);
        }
        LibXR::STDIO::Printf<"\r\nProgress: Done\r\n">();
        bmi088->in_cali_ = false;
        LibXR::Thread::Sleep(1000);

        LibXR::STDIO::Printf<"\r\nCalibration error - x: %f, y: %f, z: %f\r\n">(
            static_cast<double>(bmi088->gyro_cali_.data()[0]) /
                    static_cast<double>(bmi088->cali_counter_) * bmi088->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                bmi088->gyro_data_key_.data_.x(),
            static_cast<double>(bmi088->gyro_cali_.data()[1]) /
                    static_cast<double>(bmi088->cali_counter_) * bmi088->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                bmi088->gyro_data_key_.data_.y(),
            static_cast<double>(bmi088->gyro_cali_.data()[2]) /
                    static_cast<double>(bmi088->cali_counter_) * bmi088->GetGyroLSB() *
                    M_DEG2RAD_MULT -
                bmi088->gyro_data_key_.data_.z());

        bmi088->gyro_data_key_.Set(bmi088->gyro_data_key_.data_);
        LibXR::STDIO::Printf<"Calibration data saved.\r\n">();
      }
    }
    else if (argc == 4)
    {
      if (strcmp(argv[1], "show") == 0)
      {
        int time = std::atoi(argv[2]);
        int delay = std::atoi(argv[3]);

        delay = std::clamp(delay, 2, 1000);

        while (time > 0)
        {
          LibXR::STDIO::Printf<
              "Accel: x = %+5f, y = %+5f, z = %+5f | "
              "Gyro: x = %+5f, y = %+5f, z = %+5f | Temp: %+5f\r\n">(
              bmi088->accl_data_.x(), bmi088->accl_data_.y(), bmi088->accl_data_.z(),
              bmi088->gyro_data_.x(), bmi088->gyro_data_.y(), bmi088->gyro_data_.z(),
              bmi088->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    }
    else
    {
      LibXR::STDIO::Printf<"Error: Invalid arguments.\r\n">();
      return -1;
    }

    return 0;
  }

  GyroRange gyro_range_ = GyroRange::DEG_2000DPS;
  AcclRange accel_range_ = AcclRange::ACCL_24G;
  GyroFreq gyro_freq_ = GyroFreq::GYRO_2000HZ_BW230HZ;
  AcclFreq accl_freq_ = AcclFreq::ACCL_1600HZ;

  bool in_cali_ = false;
  uint32_t cali_counter_ = 0;
  Eigen::Matrix<std::int64_t, 3, 1> gyro_cali_;

  float temperature_ = 0.0f;

  LibXR::MicrosecondTimestamp sample_timestamp_ = 0;
  LibXR::MicrosecondTimestamp last_gyro_int_time_ = 0;
  LibXR::MicrosecondTimestamp::Duration dt_gyro_ = 0;

  float target_temperature_ = 25.0f;

  uint8_t rw_buffer_[20];
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO *cs_accl_, *cs_gyro_, *int_gyro_;
  LibXR::SPI* spi_;
  LibXR::PWM* pwm_;

  LibXR::Quaternion<float> rotation_;

  LibXR::PID<float> pid_heat_;
  LibXR::Semaphore sem_spi_, new_data_;
  LibXR::SPI::OperationRW op_spi_;

  LibXR::RamFS::File cmd_file_;

  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;

  LibXR::Thread thread_;
};
