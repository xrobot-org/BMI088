#pragma once

// clang-format off
/* === MODULE MANIFEST ===
module_name: BMI088
module_description: 博世 BMI088 6 轴惯性测量单元（IMU）的驱动模块 / Driver module for Bosch BMI088 6-axis Inertial Measurement Unit (IMU)
constructor_args:
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
  - task_stack_depth: 512
required_hardware: spi_bmi088/spi1/SPI1 bmi088_accl_cs bmi088_gyro_cs bmi088_accl_int bmi088_gyro_int pwm_bmi088_heat ramfs database
repository: https://github.com/xrobot-org/BMI088
=== END MANIFEST === */
// clang-format on

/* Recommended Website for calculate rotation:
  https://www.andre-gaschler.com/rotationconverter/ */

#include "app_framework.hpp"
#include "gpio.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "pwm.hpp"
#include "spi.hpp"
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

#define BMI088_ACCL_RX_BUFF_LEN (20)
#define BMI088_GYRO_RX_BUFF_LEN (7)

template <typename HardwareContainer>
class BMI088 : public LibXR::Application {
 public:
  enum class Device { ACCELMETER, GYROSCOPE };

  static constexpr float M_DEG2RAD_MULT = 0.01745329251f;

  void Select(Device device) {
    if (device == Device::ACCELMETER) {
      cs_accl_->Write(false);
    } else {
      cs_gyro_->Write(false);
    }
  }

  void Deselect(Device device) {
    if (device == Device::ACCELMETER) {
      cs_accl_->Write(true);
    } else {
      cs_gyro_->Write(true);
    }
  }

  void WriteSingle(Device device, uint8_t reg, uint8_t data) {
    write_buffer_[0] = static_cast<uint8_t>(reg & 0x7f);
    write_buffer_[1] = data;
    Select(device);
    spi_->Write({write_buffer_, 2}, op_spi_);
    Deselect(device);

    LibXR::Thread::Sleep(1);

    if (ReadSingle(device, reg) != data) {
      ASSERT(false);
    }

    /* For accelmeter, two write operations need at least 2us */
    LibXR::Thread::Sleep(1);
  }

  uint8_t ReadSingle(Device device, uint8_t reg) {
    write_buffer_[0] = static_cast<uint8_t>(reg | 0x80);
    write_buffer_[1] = 0x00;
    write_buffer_[2] = 0x00;
    read_buffer_[0] = 0x00;
    read_buffer_[1] = 0x00;
    read_buffer_[2] = 0x00;
    Select(device);
    spi_->ReadAndWrite({read_buffer_, 3}, {write_buffer_, 3}, op_spi_);
    Deselect(device);

    if (device == Device::ACCELMETER) {
      return read_buffer_[2];
    } else {
      return read_buffer_[1];
    }
  }

  void Read(Device device, uint8_t reg, uint8_t len) {
    memset(write_buffer_, 0, len + 1);
    memset(read_buffer_, 0, len + 1);
    write_buffer_[0] = static_cast<uint8_t>(reg | 0x80);
    Select(device);
    spi_->ReadAndWrite({read_buffer_, len}, {write_buffer_, len}, op_spi_);
    Deselect(device);
  }

  BMI088(HardwareContainer &hw, LibXR::ApplicationManager &app,
         LibXR::Quaternion<float> &&rotation,
         LibXR::PID<float>::Param &&pid_param, const char *gyro_topic_name,
         const char *accl_topic_name, float target_temperature,
         size_t task_stack_depth)
      : target_temperature_(target_temperature),
        topic_gyro_(gyro_topic_name, sizeof(gyro_data_)),
        topic_accl_(accl_topic_name, sizeof(accl_data_)),
        cs_accl_(hw.template FindOrExit<LibXR::GPIO>({"bmi088_accl_cs"})),
        cs_gyro_(hw.template FindOrExit<LibXR::GPIO>({"bmi088_gyro_cs"})),
        int_accl_(hw.template FindOrExit<LibXR::GPIO>({"bmi088_accl_int"})),
        int_gyro_(hw.template FindOrExit<LibXR::GPIO>({"bmi088_gyro_int"})),
        spi_(
            hw.template FindOrExit<LibXR::SPI>({"spi_bmi088", "spi1", "SPI1"})),
        pwm_(hw.template FindOrExit<LibXR::PWM>({"pwm_bmi088_heat"})),
        rotation_(std::move(rotation)),
        pid_heat_(pid_param),
        op_spi_(sem_spi_),
        cmd_file_(LibXR::RamFS::CreateFile("bmi088", CommandFunc, this)),
        gyro_data_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                       "bmi088_gyro_data",
                       Eigen::Matrix<float, 3, 1>(0.0, 0.0, 0.0)) {
    app.Register(*this);

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    int_accl_->DisableInterrupt();
    int_gyro_->DisableInterrupt();

    auto accl_int_cb = LibXR::Callback<>::Create(
        [](bool in_isr, BMI088<HardwareContainer> *bmi088) {
          auto time = LibXR::Timebase::GetMicroseconds();
          bmi088->dt_accl_ = time - bmi088->last_accl_int_time_;
          bmi088->last_accl_int_time_ = time;
          bmi088->new_data_.PostFromCallback(in_isr);
          bmi088->new_data_accl_.PostFromCallback(in_isr);
        },
        this);

    auto gyro_int_cb = LibXR::Callback<>::Create(
        [](bool in_isr, BMI088<HardwareContainer> *bmi088) {
          auto time = LibXR::Timebase::GetMicroseconds();
          bmi088->dt_gyro_ = time - bmi088->last_gyro_int_time_;
          bmi088->last_gyro_int_time_ = time;
          bmi088->new_data_.PostFromCallback(in_isr);
          bmi088->new_data_gyro_.PostFromCallback(in_isr);
        },
        this);

    int_accl_->RegisterCallback(accl_int_cb);
    int_gyro_->RegisterCallback(gyro_int_cb);

    while (!Init()) {
      // LibXR::STDIO::Printf("BMI088: Init failed. Try again.\r\n");
      LibXR::Thread::Sleep(100);
    }

    LibXR::STDIO::Printf("BMI088: Init succeeded.\r\n");

    thread_.Create(this, ThreadFunc, "bmi088_thread", task_stack_depth,
                   LibXR::Thread::Priority::REALTIME);
  }

  bool Init() {
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_SOFTRESET, 0xB6);
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_SOFTRESET, 0xB6);

    LibXR::Thread::Sleep(30);

    /* Need to read chip id twice */
    ReadSingle(Device::ACCELMETER, BMI088_REG_ACCL_CHIP_ID);
    ReadSingle(Device::GYROSCOPE, BMI088_REG_GYRO_CHIP_ID);

    auto accl_id = ReadSingle(Device::ACCELMETER, BMI088_REG_ACCL_CHIP_ID);
    auto gyro_id = ReadSingle(Device::GYROSCOPE, BMI088_REG_GYRO_CHIP_ID);

    if (accl_id != BMI088_CHIP_ID_ACCL) {
      return false;
    }
    if (gyro_id != BMI088_CHIP_ID_GYRO) {
      return false;
    }

    /* Accl init. */
    /* Filter setting: Normal. */
    /* ODR: 0xAB: 800Hz. 0xAA: 400Hz. 0xA9: 200Hz. 0xA8: 100Hz. 0xA6: 25Hz. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_CONF, 0xAB);

    /* 0x00: +-3G. 0x01: +-6G. 0x02: +-12G. 0x03: +-24G. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_RANGE, 0x03);

    /* INT1 as output. Push-pull. Active low. Output. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_INT1_IO_CONF, 0x08);

    /* Map data ready interrupt to INT1. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_INT1_INT2_MAP_DATA, 0x04);

    /* Turn on accl. Now we can read data. */
    WriteSingle(Device::ACCELMETER, BMI088_REG_ACCL_PWR_CTRL, 0x04);
    LibXR::Thread::Sleep(50);

    int_accl_->EnableInterrupt();

    /* Gyro init. */
    /* 0x00: +-2000. 0x01: +-1000. 0x02: +-500. 0x03: +-250. 0x04: +-125. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_RANGE, 0x00);

    /* ODR: 0x02: 1000Hz. 0x03: 400Hz. 0x06: 200Hz. 0x07: 100Hz. */
    WriteSingle(Device::GYROSCOPE, BMI088_REG_GYRO_BANDWIDTH, 0x02);

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

  void OnMonitor(void) override {
    if (std::isinf(gyro_data_.x()) || std::isinf(gyro_data_.y()) ||
        std::isinf(gyro_data_.z()) || std::isinf(accl_data_.x()) ||
        std::isinf(accl_data_.y()) || std::isinf(accl_data_.z()) ||
        std::isnan(gyro_data_.x()) || std::isnan(gyro_data_.y()) ||
        std::isnan(gyro_data_.z()) || std::isnan(accl_data_.x()) ||
        std::isnan(accl_data_.y()) || std::isnan(accl_data_.z())) {
      LibXR::STDIO::Printf(
          "BMI088: NaN data detected. gyro: %f %f %f, accl: %f %f %f\r\n",
          gyro_data_.x(), gyro_data_.y(), gyro_data_.z(), accl_data_.x(),
          accl_data_.y(), accl_data_.z());
    }

    /* Use other timer as HAL timebase (Because the priority of SysTick is
      lowest) and set the priority to the highest to avoid this issue */
    if (fabs(dt_gyro_) > 1500 || fabs(dt_gyro_) < 300 ||
        fabs(dt_accl_) > 1500 || fabs(dt_accl_) < 300) {
      LibXR::STDIO::Printf("BMI088 Frequency Error: gyro: %6f, accl: %6f\r\n",
                           dt_gyro_.to_secondf(), dt_accl_.to_secondf());
    }
  }

  static void ThreadFunc(BMI088<HardwareContainer> *bmi088) {
    /* Start PWM */
    bmi088->pwm_->SetConfig({1000});
    bmi088->pwm_->SetDutyCycle(0);
    bmi088->pwm_->Enable();

    while (true) {
      if (bmi088->new_data_.Wait(50) == ErrorCode::OK) {
        if (bmi088->new_data_accl_.Wait(0) == ErrorCode::OK) {
          bmi088->RecvAccel();
          bmi088->ParseAccelData();

          bmi088->topic_accl_.Publish(bmi088->accl_data_);
        }

        if (bmi088->new_data_gyro_.Wait(0) == ErrorCode::OK) {
          bmi088->RecvGyro();
          bmi088->ParseGyroData();
          bmi088->topic_gyro_.Publish(bmi088->gyro_data_);
          bmi088->pwm_->SetDutyCycle(bmi088->pid_heat_.Calculate(
              bmi088->target_temperature_, bmi088->temperature_,
              bmi088->dt_gyro_));
        }
      } else {
        LibXR::STDIO::Printf("BMI088 wait timeout.");
      }
    }
  }

  void RecvAccel(void) {
    Read(Device::ACCELMETER, BMI088_REG_ACCL_X_LSB, BMI088_ACCL_RX_BUFF_LEN);
  }

  void RecvGyro(void) {
    Read(Device::GYROSCOPE, BMI088_REG_GYRO_X_LSB, BMI088_GYRO_RX_BUFF_LEN);
  }

  void ParseAccelData(void) {
    std::array<int16_t, 3> raw_int16;
    std::array<float, 3> raw;
    for (int i = 0; i < 3; i++) {
      raw_int16[i] = (read_buffer_[i * 2 + 3] << 8) | read_buffer_[i * 2 + 2];
      raw[i] = static_cast<float>(raw_int16[i]) / 1365.0f;
    }

    int16_t raw_temp = (read_buffer_[18] << 3) | (read_buffer_[19] >> 5);
    if (raw_temp > 1023) {
      raw_temp -= 2048;
    }

    temperature_ = static_cast<float>(raw_temp) * 0.125f + 23.0f;

    if (raw[0] == 0.0f && raw[1] == 0.0f && raw[2] == 0.0f) {
      return;
    }

    accl_data_ = rotation_ * Eigen::Matrix<float, 3, 1>(raw[0], raw[1], raw[2]);
  }

  void ParseGyroData(void) {
    std::array<int16_t, 3> raw_int16;
    std::array<float, 3> raw;
    for (int i = 0; i < 3; i++) {
      raw_int16[i] = (read_buffer_[i * 2 + 2] << 8) | read_buffer_[i * 2 + 1];
      raw[i] = static_cast<float>(raw_int16[i]) / 16.384f * M_DEG2RAD_MULT;
    }

    if (raw[0] == 0.0f && raw[1] == 0.0f && raw[2] == 0.0f) {
      return;
    }

    gyro_data_ =
        rotation_ * Eigen::Matrix<float, 3, 1>(raw[0], raw[1], raw[2]) -
        gyro_data_key_.data_;
  }

 private:
  static int CommandFunc(BMI088<HardwareContainer> *bmi088, int argc,
                         char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  show [time_ms] [interval_ms] - Print sensor data "
          "periodically.\r\n");
      LibXR::STDIO::Printf(
          "  list_offset                  - Show current gyro calibration "
          "offset.\r\n");
      LibXR::STDIO::Printf(
          "  cali                         - Start gyroscope calibration.\r\n");
    } else if (argc == 2) {
      if (strcmp(argv[1], "list_offset") == 0) {
        LibXR::STDIO::Printf(
            "Current calibration offset - x: %f, y: %f, z: %f\r\n",
            bmi088->gyro_data_key_.data_.x(), bmi088->gyro_data_key_.data_.y(),
            bmi088->gyro_data_key_.data_.z());
      } else if (strcmp(argv[1], "cali") == 0) {
        bmi088->gyro_data_key_.data_.x() = 0.0,
        bmi088->gyro_data_key_.data_.y() = 0.0,
        bmi088->gyro_data_key_.data_.z() = 0.0;
        LibXR::STDIO::Printf(
            "Starting gyroscope calibration. Please keep the device "
            "steady.\r\n");
        double x = 0.0, y = 0.0, z = 0.0;
        for (int i = 0; i < 30000; i++) {
          x += static_cast<double>(bmi088->gyro_data_.x()) / 30000.0;
          y += static_cast<double>(bmi088->gyro_data_.y()) / 30000.0;
          z += static_cast<double>(bmi088->gyro_data_.z()) / 30000.0;
          if (i % 100 == 0) {
            LibXR::STDIO::Printf("Progress: %d / 30000\r", i);
          }
          LibXR::Thread::Sleep(1);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        bmi088->gyro_data_key_.data_.x() = static_cast<float>(x);
        bmi088->gyro_data_key_.data_.y() = static_cast<float>(y);
        bmi088->gyro_data_key_.data_.z() = static_cast<float>(z);

        LibXR::STDIO::Printf("\r\nCalibration result - x: %f, y: %f, z: %f\r\n",
                             bmi088->gyro_data_key_.data_.x(),
                             bmi088->gyro_data_key_.data_.y(),
                             bmi088->gyro_data_key_.data_.z());

        LibXR::STDIO::Printf("Analyzing calibration quality...\r\n");
        x = y = z = 0.0;
        for (int i = 0; i < 30000; i++) {
          x += static_cast<double>(bmi088->gyro_data_.x()) / 30000.0;
          y += static_cast<double>(bmi088->gyro_data_.y()) / 30000.0;
          z += static_cast<double>(bmi088->gyro_data_.z()) / 30000.0;
          if (i % 100 == 0) {
            LibXR::STDIO::Printf("Progress: %d / 30000\r", i);
          }
          LibXR::Thread::Sleep(1);
        }
        LibXR::STDIO::Printf("\r\nProgress: Done\r\n");

        LibXR::STDIO::Printf("\r\nCalibration error - x: %f, y: %f, z: %f\r\n",
                             x, y, z);

        bmi088->gyro_data_key_.Set(bmi088->gyro_data_key_.data_);
        LibXR::STDIO::Printf("Calibration data saved.\r\n");
      }
    } else if (argc == 4) {
      if (strcmp(argv[1], "show") == 0) {
        int time = std::stoi(argv[2]);
        int delay = std::stoi(argv[3]);
        delay = std::clamp(delay, 2, 1000);

        while (time > 0) {
          LibXR::STDIO::Printf(
              "Accel: x = %+5f, y = %+5f, z = %+5f | "
              "Gyro: x = %+5f, y = %+5f, z = %+5f | Temp: %+5f\r\n",
              bmi088->accl_data_.x(), bmi088->accl_data_.y(),
              bmi088->accl_data_.z(), bmi088->gyro_data_.x(),
              bmi088->gyro_data_.y(), bmi088->gyro_data_.z(),
              bmi088->temperature_);
          LibXR::Thread::Sleep(delay);
          time -= delay;
        }
      }
    } else {
      LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
      return -1;
    }

    return 0;
  }

  float temperature_ = 0.0f;

  LibXR::TimestampUS last_gyro_int_time_ = 0;
  LibXR::TimestampUS last_accl_int_time_ = 0;
  LibXR::TimestampUS::TimeDiffUS dt_gyro_ = 0;
  LibXR::TimestampUS::TimeDiffUS dt_accl_ = 0;

  float target_temperature_ = 25.0f;

  uint8_t read_buffer_[20], write_buffer_[20];
  Eigen::Matrix<float, 3, 1> gyro_data_, accl_data_;
  LibXR::Topic topic_gyro_, topic_accl_;
  LibXR::GPIO *cs_accl_, *cs_gyro_, *int_accl_, *int_gyro_;
  LibXR::SPI *spi_;
  LibXR::PWM *pwm_;

  LibXR::Quaternion<float> rotation_;

  LibXR::PID<float> pid_heat_;

  LibXR::Semaphore sem_spi_, new_data_, new_data_gyro_, new_data_accl_;
  LibXR::SPI::OperationRW op_spi_;

  LibXR::RamFS::File cmd_file_;

  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;

  LibXR::Thread thread_;
};
