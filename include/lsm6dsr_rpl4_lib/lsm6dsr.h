#ifndef LSM6DSR_RPL4_LIB_LSM6DSR_H
#define LSM6DSR_RPL4_LIB_LSM6DSR_H

#include "lsm6dsr_rpl4_lib/registers.h"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/rpl4.hpp"

namespace lsm6dsr_rpl4_lib {
class LSM6DSR {
 public:
  LSM6DSR(std::shared_ptr<rpl::Spi> spi, rpl::Spi::ChipSelect cs,
          std::shared_ptr<rpl::Gpio> gpio = nullptr);
  ~LSM6DSR() = default;

  /**
   * @brief Initializes the LSM6DSR sensor.
   * @return true if initialization is successful, false otherwise.
   */
  bool Init();

  /**
   * @brief Read accelerometer data.
   *
   * @param x Pointer to store the x-axis acceleration.
   * @param y Pointer to store the y-axis acceleration.
   * @param z Pointer to store the z-axis acceleration.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadAccel(float* x, float* y, float* z);

  /**
   * @brief Read gyroscope data.
   *
   * @param x Pointer to store the x-axis gyroscope data.
   * @param y Pointer to store the y-axis gyroscope data.
   * @param z Pointer to store the z-axis gyroscope data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadGyro(float* x, float* y, float* z);

  /**
   * @brief Write data to one specific register.
   *
   * @param address The register address to write to.
   * @param data Pointer to the data to write.
   * @return true if writing is successful, false if writing fails.
   */
  bool WriteRegister(Register address, uint8_t* data);

  /**
   * @brief Read data from one specific register.
   *
   * @param address The register address to read from.
   * @param data Pointer to store the read data.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadRegister(Register address, uint8_t* data);

  /**
   * @brief Write data to multiple registers starting from a specific address.
   *
   * @param start_address The register address to start writing from.
   * @param data Pointer to the data to write.
   * @param length The number of registers to write.
   * @return true if writing is successful, false if writing fails.
   */
  bool WriteRegisters(Register start_address, uint8_t* data, size_t length);

  /**
   * @brief Read data from multiple registers starting from a specific address.
   *
   * @param start_address The register address to start reading from.
   * @param data Pointer to store the read data.
   * @param length The number of registers to read.
   * @return true if reading is successful, false if reading fails.
   */
  bool ReadRegisters(Register start_address, uint8_t* data, size_t length);

  /**
   * @brief Read the WHO_AM_I register to verify the device identity.
   *
   * @return The value of the WHO_AM_I register.
   */
  uint8_t WhoAmI();

  enum class EnableState : bool {
    kEnabled = true,
    kDisabled = false,
  };

  /**
   * @brief Write the I3C enabled state to the register.
   *        When using SPI or I2c, this should be set to kDisabled.
   *
   * @param enable_state The state to enable or disable I3C.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteI3cEnabled(EnableState enable_state);

  /**
   * @brief Write the auto-increment enabled state to the register.
   *        kEnabled for reading or writing multiple registers in a row,
   *
   * @param enable_state The state to enable or disable auto-increment.
   * @return true if the operation is successful.
   * @return false if the operation fails.
   */
  bool WriteAutoIncrementEnabled(EnableState enable_state);

  enum class GyroDataRate : uint8_t {
    kPowerDown = 0b0000,
    k12_5Hz = 0b0001,
    k26Hz = 0b0010,
    k52Hz = 0b0011,
    k104Hz = 0b0100,
    k208Hz = 0b0101,
    k416Hz = 0b0110,
    k833Hz = 0b0111,
    k1660Hz = 0b1000,
    k3330Hz = 0b1001,
    k6660Hz = 0b1010,
  };

  /**
   * @brief Write the gyroscope data rate to the register.
   *
   * @param data_rate The desired gyroscope data rate.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteGyroDataRate(GyroDataRate data_rate);

  enum class GyroFullScale : uint8_t {
    k125dps = 0b0010,
    k250dps = 0b0000,
    k500dps = 0b0100,
    k1000dps = 0b1000,
    k2000dps = 0b1100,
    k4000dps = 0b0001,
  };

  /**
   * @brief Write the gyroscope full scale to the register.
   *
   * @param full_scale The desired gyroscope full scale.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteGyroFullScale(GyroFullScale full_scale);

  /**
   * @brief Read the gyroscope full scale from the register.
   *
   * @return The current gyroscope full scale setting.
   */
  GyroFullScale ReadGyroFullScale();

  /**
   * @brief Configure the gyroscope sensitivity based on the full scale setting.
   *
   * @param full_scale The full scale setting to configure sensitivity.
   */
  void ConfigureGyroSensitivity(GyroFullScale full_scale);

  enum class AccelDataRate : uint8_t {
    kPowerDown = 0b0000,
    k1_6Hz = 0b1011,
    k12_5Hz = 0b0001,
    k26Hz = 0b0010,
    k52Hz = 0b0011,
    k104Hz = 0b0100,
    k208Hz = 0b0101,
    k416Hz = 0b0110,
    k833Hz = 0b0111,
    k1660Hz = 0b1000,
    k3330Hz = 0b1001,
    k6660Hz = 0b1010,
  };

  /**
   * @brief Write the accelerometer data rate to the register.
   *
   * @param data_rate The desired accelerometer data rate.
   * @return true if the operation is successful, false if operation fails.
   */
  bool WriteAccDataRate(AccelDataRate data_rate);

  enum class AccelFullScale : uint8_t {
    k2g = 0b00,
    k4g = 0b10,
    k8g = 0b11,
    k16g = 0b01,
  };

  /**
   * @brief Write the accelerometer full scale to the register.
   *
   * @param full_scale The desired accelerometer full scale.
   * @return true if the operation is successful.
   * @return false if the operation fails.
   */
  bool WriteAccFullScale(AccelFullScale full_scale);

  /**
   * @brief Read the accelerometer full scale from the register.
   *
   * @return The current accelerometer full scale setting.
   */
  AccelFullScale ReadAccFullScale();

  /**
   * @brief Configure the accelerometer sensitivity based on the full scale
   *        setting.
   *
   * @param full_scale The full scale setting to configure sensitivity.
   */
  void ConfigureAccSensitivity(AccelFullScale full_scale);

  /**
   * @brief Read accelerometer and gyroscope data.
   *
   * @param gyro_x pointer to store the x-axis gyroscope data.
   * @param gyro_y pointer to store the y-axis gyroscope data.
   * @param gyro_z pointer to store the z-axis gyroscope data.
   * @param acc_x pointer to store the x-axis acceleration data.
   * @param acc_y pointer to store the y-axis acceleration data.
   * @param acc_z pointer to store the z-axis acceleration data.
   * @return true if the operation is successful, false if operation fails.
   */
  bool ReadAccAndGyro(float& gyro_x, float& gyro_y, float& gyro_z, float& acc_x,
                      float& acc_y, float& acc_z);

  /**
   * @brief Reset the memory of the LSM6DSR device. All registers will be set to
   *        their default values. You sould wait for 15ms after this.
   * @return true if the reset is successful, false if it fails.
   */
  bool ResetMemory();

  /**
   * @brief Reboot the LSM6DSR device. You should wait for 50us after this.
   * @return true if the reboot is successful, false if it fails.
   */
  bool RebootDevice();

 private:
  std::shared_ptr<rpl::Spi> spi_;
  std::shared_ptr<rpl::Gpio> gpio_;
  rpl::Spi::ChipSelect cs_;
  float acc_sensitivity_ = 0.0f;
  float gyro_sensitivity_ = 0.0f;

  void EnableCs();
  void DisableCs();
};

}  // namespace lsm6dsr_rpl4_lib

#endif  // LSM6DSR_RPL4_LIB_LSM6DSR_H