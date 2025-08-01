#include "lsm6dsr_rpl4_lib/lsm6dsr.h"

#include <iostream>

#include "lsm6dsr_rpl4_lib/registers.h"
#include "rpl4/peripheral/gpio.hpp"
#include "rpl4/peripheral/spi.hpp"

namespace lsm6dsr_rpl4_lib {

LSM6DSR::LSM6DSR(std::shared_ptr<rpl::SpiBase> spi, uint8_t cs_num,
                 std::shared_ptr<rpl::Gpio> cs_gpio)
    : spi_(spi), cs_num_(cs_num), cs_gpio_(cs_gpio) {}

bool LSM6DSR::Init() {
  bool is_success = true;
  if (cs_gpio_) {
    cs_gpio_->SetAltFunction(rpl::Gpio::AltFunction::kOutput);
    cs_gpio_->SetPullRegister(rpl::Gpio::PullRegister::kNoRegister);
    cs_gpio_->Write(true);
  }
  is_success &= WriteI3cEnabled(LSM6DSR::EnableState::kDisabled);
  is_success &= WriteAutoIncrementEnabled(LSM6DSR::EnableState::kEnabled);
  return is_success;
}

bool LSM6DSR::ReadAccel(float* x, float* y, float* z) {
  return true;  // or false on failure
}

bool LSM6DSR::ReadGyro(float* x, float* y, float* z) {
  return true;  // or false on failure
}

bool LSM6DSR::WriteRegister(Register address, uint8_t* data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  tx_buf[0] = static_cast<uint8_t>(address);
  tx_buf[1] = *data;

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 2);
  DisableCs();

  return true;
}

bool LSM6DSR::ReadRegister(Register address, uint8_t* data) {
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  tx_buf[0] = static_cast<uint8_t>(address) | 0b10000000;
  tx_buf[1] = 0;

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, 2);
  DisableCs();

  *data = rx_buf[1];

  return true;
}

bool LSM6DSR::WriteRegisters(Register start_address, uint8_t* data,
                             size_t length) {
  uint8_t tx_buf[length + 1];
  uint8_t rx_buf[length + 1];

  tx_buf[0] = static_cast<uint8_t>(start_address);
  for (size_t i = 0; i < length; ++i) { tx_buf[i + 1] = data[i]; }

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, length + 1);
  DisableCs();

  return true;
}

bool LSM6DSR::ReadRegisters(Register start_address, uint8_t* data,
                            size_t length) {
  uint8_t tx_buf[length + 1];
  uint8_t rx_buf[length + 1];
  tx_buf[0] = static_cast<uint8_t>(start_address) | 0b10000000;

  EnableCs();
  spi_->TransmitAndReceiveBlocking(tx_buf, rx_buf, length + 1);
  DisableCs();

  for (size_t i = 0; i < length; ++i) { data[i] = rx_buf[i + 1]; }

  return true;
}

uint8_t LSM6DSR::WhoAmI() {
  uint8_t who_am_i;
  if (ReadRegister(Register::WHO_AM_I, &who_am_i)) {
    // Ideally 0x6B
    std::cout << "Who Am I: 0x" << std::hex << static_cast<int>(who_am_i)
              << std::dec << std::endl;
    return who_am_i;
  } else {
    return 0;
  }
}

bool LSM6DSR::WriteI3cEnabled(EnableState enable_state) {
  uint8_t ctrl9_xl;
  ReadRegister(Register::CTRL9_XL, &ctrl9_xl);

  if (enable_state == EnableState::kEnabled) {
    ctrl9_xl |= 0b00000001;  // Set the I3C_EN bit
  } else {
    ctrl9_xl &= ~0b00000001;  // Clear the I3C_EN bit
  }

  return WriteRegister(Register::CTRL9_XL, &ctrl9_xl);
}

bool LSM6DSR::WriteAutoIncrementEnabled(EnableState enable_state) {
  uint8_t ctrl3_c;
  ReadRegister(Register::CTRL3_C, &ctrl3_c);

  if (enable_state == EnableState::kEnabled) {
    ctrl3_c |= 0b00000100;  // Set the IF_INC bit
  } else {
    ctrl3_c &= ~0b00000100;  // Clear the IF_INC bit
  }

  return WriteRegister(Register::CTRL3_C, &ctrl3_c);
}

bool LSM6DSR::WriteGyroDataRate(LSM6DSR::GyroDataRate data_rate) {
  uint8_t test = 0b10101100;
  WriteRegister(Register::CTRL1_XL, &test);

  uint8_t ctrl2_g;
  ReadRegister(Register::CTRL2_G, &ctrl2_g);

  // Clear the data rate bits
  ctrl2_g &= ~0b11110000;

  // Set the new data rate
  ctrl2_g |= (static_cast<uint8_t>(data_rate) & 0b1111) << 4;

  return WriteRegister(Register::CTRL2_G, &ctrl2_g);
}

bool LSM6DSR::WriteGyroFullScale(LSM6DSR::GyroFullScale full_scale) {
  uint8_t ctrl2_g;
  ReadRegister(Register::CTRL2_G, &ctrl2_g);

  // Clear the full scale bits
  ctrl2_g &= ~0b00001111;

  // Set the new full scale
  ctrl2_g |= static_cast<uint8_t>(full_scale) & 0b1111;

  return WriteRegister(Register::CTRL2_G, &ctrl2_g);
}

LSM6DSR::GyroFullScale LSM6DSR::ReadGyroFullScale() {
  uint8_t ctrl2_g;
  ReadRegister(Register::CTRL2_G, &ctrl2_g);
  return static_cast<GyroFullScale>(ctrl2_g & 0b00001111);
}

void LSM6DSR::ConfigureGyroSensitivity(GyroFullScale full_scale) {
  switch (full_scale) {
    case GyroFullScale::k125dps:
      gyro_sensitivity_ = 0.004375f;  // Sensitivity for 125 dps
      break;
    case GyroFullScale::k250dps:
      gyro_sensitivity_ = 0.00875f;  // Sensitivity for 250 dps
      break;
    case GyroFullScale::k500dps:
      gyro_sensitivity_ = 0.0175f;  // Sensitivity for 500 dps
      break;
    case GyroFullScale::k1000dps:
      gyro_sensitivity_ = 0.035f;  // Sensitivity for 1000 dps
      break;
    case GyroFullScale::k2000dps:
      gyro_sensitivity_ = 0.07f;  // Sensitivity for 2000 dps
      break;
    case GyroFullScale::k4000dps:
      gyro_sensitivity_ = 0.14f;  // Sensitivity for 4000 dps
      break;
    default:
      gyro_sensitivity_ = 0.0f;  // Invalid full scale
      break;
  }
}

bool LSM6DSR::WriteAccDataRate(AccelDataRate data_rate) {
  uint8_t ctrl1_xl;
  ReadRegister(Register::CTRL1_XL, &ctrl1_xl);

  // Clear the data rate bits
  ctrl1_xl &= ~0b11110000;

  // Set the new data rate
  ctrl1_xl |= (static_cast<uint8_t>(data_rate) & 0b1111) << 4;

  return WriteRegister(Register::CTRL1_XL, &ctrl1_xl);
}

bool LSM6DSR::WriteAccFullScale(LSM6DSR::AccelFullScale full_scale) {
  uint8_t ctrl1_xl;
  ReadRegister(Register::CTRL1_XL, &ctrl1_xl);

  // Clear the full scale bits
  ctrl1_xl &= ~0b00001100;

  // Set the new full scale
  ctrl1_xl |= (static_cast<uint8_t>(full_scale) & 0b11) << 2;

  return WriteRegister(Register::CTRL1_XL, &ctrl1_xl);
}

LSM6DSR::AccelFullScale LSM6DSR::ReadAccFullScale() {
  uint8_t ctrl1_xl;
  ReadRegister(Register::CTRL1_XL, &ctrl1_xl);
  return static_cast<AccelFullScale>((ctrl1_xl & 0b00001100) >> 2);
}

void LSM6DSR::ConfigureAccSensitivity(AccelFullScale full_scale) {
  switch (full_scale) {
    case AccelFullScale::k2g:
      acc_sensitivity_ = 0.061f;  // Sensitivity for 2g
      break;
    case AccelFullScale::k4g:
      acc_sensitivity_ = 0.122f;  // Sensitivity for 4g
      break;
    case AccelFullScale::k8g:
      acc_sensitivity_ = 0.244f;  // Sensitivity for 8g
      break;
    case AccelFullScale::k16g:
      acc_sensitivity_ = 0.488f;  // Sensitivity for 16g
      break;
    default:
      acc_sensitivity_ = 0.0f;  // Invalid full scale
      break;
  }
}

bool LSM6DSR::ReadAccAndGyro(float& gyro_x, float& gyro_y, float& gyro_z,
                             float& acc_x, float& acc_y, float& acc_z) {
  uint8_t data[12];
  bool status = ReadRegisters(Register::OUTX_L_G, data, 12);
  if (!status) { return status; }
  // Combine the low and high bytes for gyro and accelerometer data
  gyro_x = static_cast<int16_t>((data[1] << 8) | data[0]) * gyro_sensitivity_;
  gyro_y = static_cast<int16_t>((data[3] << 8) | data[2]) * gyro_sensitivity_;
  gyro_z = static_cast<int16_t>((data[5] << 8) | data[4]) * gyro_sensitivity_;

  acc_x = static_cast<int16_t>((data[7] << 8) | data[6]) * acc_sensitivity_;
  acc_y = static_cast<int16_t>((data[9] << 8) | data[8]) * acc_sensitivity_;
  acc_z = static_cast<int16_t>((data[11] << 8) | data[10]) * acc_sensitivity_;

  return true;
}

bool LSM6DSR::ResetMemory() {
  uint8_t ctrl3_c;
  ReadRegister(Register::CTRL3_C, &ctrl3_c);

  // Set the BOOT bit
  ctrl3_c |= 0b1000000;

  // Write back to the CTRL3_C register
  return WriteRegister(Register::CTRL3_C, &ctrl3_c);
}

bool LSM6DSR::RebootDevice() {
  uint8_t ctrl3_c;
  ReadRegister(Register::CTRL3_C, &ctrl3_c);

  // Set the SW_RESET bit
  ctrl3_c |= 0b00000001;

  // Write back to the CTRL3_C register
  return WriteRegister(Register::CTRL3_C, &ctrl3_c);
}

void LSM6DSR::EnableCs() {
  spi_->SetChipSelectForCommunication(cs_num_);
  if (cs_gpio_) { cs_gpio_->Write(false); }
}

void LSM6DSR::DisableCs() {
  if (cs_gpio_) { cs_gpio_->Write(true); }
}

}  // namespace lsm6dsr_rpl4_lib