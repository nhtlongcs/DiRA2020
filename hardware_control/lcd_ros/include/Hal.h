#pragma once

#include <fcntl.h>
#include <math.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

int constexpr MAX_WRITE_LEN = 255;
int constexpr MAX_READ_LEN = 255;
int constexpr MAX_BUF = 64;

namespace dira_lcd {
class Comm {
 public:
  virtual bool HALOpen() = 0;
  virtual bool HALClose() = 0;
  virtual bool HALRead(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char *data,
                       char const *error_msg) = 0;
  virtual bool HALRead(unsigned char slave_addr, unsigned char length,
                       unsigned char *data, char const *error_msg) = 0;
  virtual bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                        unsigned char length, unsigned char const *data,
                        char const *error_msg) = 0;
  virtual bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                        unsigned char const data, char const *error_msg) = 0;

 protected:
  virtual bool ifWrite(unsigned char *data, unsigned char length) = 0;
};

class I2C : public Comm {
 public:
  virtual ~I2C();

  bool HALOpen();
  bool HALClose();
  bool HALRead(unsigned char slave_addr, unsigned char reg_addr,
               unsigned char length, unsigned char *data,
               char const *error_msg);
  bool HALRead(unsigned char slave_addr, unsigned char length,
               unsigned char *data, char const *error_msg);
  bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char length, unsigned char const *data,
                char const *error_msg);
  bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char const data, char const *error_msg);

  bool selectSlave(unsigned char slave_addr, char const *error_msg);

  int m_i2c_bus;  // I2C bus of the imu (eg 1 for Raspberry Pi usually)
 private:
  int m_i2c = -1;
  unsigned char m_current_slave = 255;

 protected:
  bool ifWrite(unsigned char *data, unsigned char length);
};

class SPI : public Comm {
 public:
  virtual ~SPI();

  bool HALOpen();
  bool HALClose();
  bool HALRead(unsigned char slave_addr, unsigned char reg_addr,
               unsigned char length, unsigned char *data,
               char const *error_msg);
  bool HALRead(unsigned char slave_addr, unsigned char length,
               unsigned char *data, char const *error_msg);
  bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char length, unsigned char const *data,
                char const *error_msg);
  bool HALWrite(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char const data, char const *error_msg);

 public:
  unsigned char m_spi_bus = 255;
  unsigned char m_spi_select = 255;
  unsigned int m_spi_speed = 500000;

 protected:
  bool ifWrite(unsigned char *data, unsigned char length);

 private:
  int m_spi = 1;
};
}  // namespace dira_lcd
