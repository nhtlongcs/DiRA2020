#include "Hal.h"

#include <cstdio>
#include <cstring>
#include <iostream>

namespace dira_lcd {
//----I2C-----------------------------------------------------------------------------------------------------//
I2C::~I2C() { HALClose(); }

/**
 * @brief I2C::HALOpen
 * @details Open I2C device with default information
 * @return
 */
bool I2C::HALOpen() {
  char buf[32];
  if (m_i2c >= 0) return true;
  if (m_i2c_bus == 255) {
    std::cerr << "No I2C bus has been set" << std::endl;
    return false;
  }
  sprintf(buf, "/dev/i2c-%d", m_i2c_bus);
  m_i2c = open(buf, O_RDWR);
  if (m_i2c < 0) {
    std::cerr << "Failed to open I2C bus " << m_i2c_bus << std::endl;
    m_i2c = -1;
    return false;
  }
  return true;
}

/**
 * @brief I2C::HALClose
 * @details Close I2C device
 * @return
 */
bool I2C::HALClose() {
  if (m_i2c >= 0) {
    close(m_i2c);
    m_i2c = -1;
    m_current_slave = 255;
  }
  return true;
}

/**
 * @brief I2C::HALRead
 * @details Read data from I2C device
 * @param slave_addr
 * @param reg_addr
 * @param length
 * @param data
 * @param error_msg
 * @return
 */
bool I2C::HALRead(unsigned char slave_addr, unsigned char reg_addr,
                  unsigned char length, unsigned char *data,
                  const char *error_msg) {
  int tries, total, result;
  if (!this->HALWrite(slave_addr, reg_addr, 0, NULL, error_msg)) return false;

  total = 0;
  tries = 0;

  while ((total < length) && (tries < 5)) {
    result = read(m_i2c, data + total, length - total);

    if (result < 0) {
      if (strlen(error_msg) > 0) {
        std::cerr << "I2C read error from " << slave_addr << ", " << reg_addr
                  << " - " << error_msg << std::endl;
      }
      return false;
    }

    total += result;
    if (total == length) {
      break;
    }

    tries++;
  }

  if (total < length) {
    if (strlen(error_msg) > 0) {
      std::cerr << "I2C read from " << slave_addr << ", " << reg_addr << " - "
                << error_msg << std::endl;
    }
    return false;
  }
  return true;
}

/**
 * @brief I2C::HALRead
 * @param slave_addr
 * @param length
 * @param data
 * @param error_msg
 * @return
 */
bool I2C::HALRead(unsigned char slave_addr, unsigned char length,
                  unsigned char *data, const char *error_msg) {
  int tries, result, total;
  if (!this->selectSlave(slave_addr, error_msg)) {
    return false;
  }

  total = 0;
  tries = 0;

  while ((total < length) && (tries < 5)) {
    result = read(m_i2c, data + total, length - total);

    if (result < 0) {
      if (strlen(error_msg) > 0) {
        std::cerr << "I2C read error from " << slave_addr << " - " << error_msg
                  << std::endl;
      }
      return false;
    }

    total += result;
    if (total == length) {
      break;
    }
    ++tries;
  }

  if (total < length) {
    if (strlen(error_msg) > 0)
      std::cerr << "I2C read from " << slave_addr << " - " << error_msg
                << std::endl;
    return false;
  }
  return true;
}

/**
 * @brief I2C::selectSlave
 * @param slave_addr
 * @param error_msg
 * @return
 */
bool I2C::selectSlave(unsigned char slave_addr, const char *error_msg) {
  if (m_current_slave == slave_addr) {
    return true;
  }

  if (!this->HALOpen()) {
    std::cerr << "Failed to open I2C port - " << error_msg << std::endl;
    return false;
  }

  if (ioctl(m_i2c, I2C_SLAVE, slave_addr) < 0) {
    std::cerr << "I2C slave select " << slave_addr << " failed - " << error_msg
              << std::endl;
    return false;
  }
  m_current_slave = slave_addr;
  return true;
}

/**
 * @brief I2C::HALWrite
 * @param slave_addr
 * @param reg_addr
 * @param data
 * @param error_msg
 * @return
 */
bool I2C::HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                   const unsigned char data, const char *error_msg) {
  return this->HALWrite(slave_addr, reg_addr, 1, &data, error_msg);
}

/**
 * @brief I2C::HALWrite
 * @param slave_addr
 * @param reg_addr
 * @param length
 * @param data
 * @param error_msg
 * @return
 */
bool I2C::HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                   unsigned char length, const unsigned char *data,
                   const char *error_msg) {
  int result;
  char *if_type;
  unsigned char tx_buff[MAX_WRITE_LEN + 1];

  if (!this->selectSlave(slave_addr, error_msg)) return false;
  if_type = (char *)"I2C";

  if (length == 0) {
    char len = 1;
    result = this->ifWrite(&reg_addr, len);

    if (result < 0) {
      if (strlen(error_msg) > 0) {
        std::cerr << if_type << " write of regAddr failed - " << error_msg
                  << std::endl;
      }
      return false;
    } else if (result != 1) {
      if (strlen(error_msg) > 0) {
        std::cerr << if_type << " write of regAddr failed (nothing written) - "
                  << error_msg << std::endl;
      }
      return false;
    }
  } else {
    tx_buff[0] = reg_addr;
    memcpy(tx_buff + 1, data, length);
    char len = length + 1;
    result = this->ifWrite(tx_buff, len);

    if (result < 0) {
      if (strlen(error_msg) > 0) {
        std::cerr << if_type << " data write of " << length
                  << " bytes failed - " << error_msg << std::endl;
      }
      return false;
    } else if (result < (int)length) {
      if (strlen(error_msg) > 0)
        std::cerr << if_type << " data write of " << length
                  << " bytes failed, only " << result << " - " << error_msg
                  << std::endl;
      return false;
    }
  }
  return true;
}

/**
 * @brief I2C::ifWrite
 * @param data
 * @param length
 * @return
 */
bool I2C::ifWrite(unsigned char *data, unsigned char length) {
  return write(m_i2c, data, length);
}

}  // namespace dira_lcd