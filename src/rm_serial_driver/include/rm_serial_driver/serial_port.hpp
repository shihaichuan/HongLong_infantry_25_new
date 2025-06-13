// C++ system
#include <atomic>
#include <functional>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// C system
namespace rm_serial_driver {

enum PkgState : uint8_t {
  COMPLETE = 0,
  HEADER_INCOMPLETE,
  PAYLOAD_INCOMPLETE,
  CRC_HEADER_ERRROR,
  CRC_PKG_ERROR,
  OTHER
};

enum StopBit : uint8_t { ONE = 0, ONE_POINT_FIVE, TWO };

enum Parity : uint8_t { NONE = 0, ODD, EVEN, MARK, SPACE };

class SerialConfig {
public:
  SerialConfig() = delete;
  SerialConfig(int bps, int databit, bool flow, StopBit stopbits,
               Parity paritys, std::string name)
      : baudrate(bps), databits(databit), flowcontrol(flow), stopbit(stopbits),
        parity(paritys), devname(name) {}
  ~SerialConfig();

  
  int baudrate = 961200;
  int databits = 8;
  bool flowcontrol = 0;
  StopBit stopbit = StopBit::ONE;
  Parity parity = Parity::NONE;
  std::string devname = "/dev/ttyACM0";
};

class Port {
public:
  Port(std::shared_ptr<SerialConfig> ptr);
  ~Port();

  // port function
  int openPort();
  bool closePort();
  bool init();
  bool reopen();
  bool isPortInit();
  bool isPortOpen();
  //给串口赋予权限
  bool setPermission(std::string name);

  // rx tx function
  int transmit(uint8_t *buff, int writeSize);
  int receive(uint8_t *buffer);
  int fd;

private:
  std::shared_ptr<SerialConfig> config;
  std::vector<std::string> device_names = {"/dev/ttyACM0", "/dev/ttyACM1",
                                      "/dev/ttyACM2"};
  int flags = 0;
  // int num_per_read = 0;
  // int num_per_write = 0;
  bool isinit = false;
  bool isopen = false;
  PkgState frameState;
};
} // namespace rm_serial_driver