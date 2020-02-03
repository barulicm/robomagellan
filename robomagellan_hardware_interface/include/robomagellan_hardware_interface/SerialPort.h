#ifndef ROBOMAGELLAN_HARDWARE_INTERFACE_SERIALPORT_H
#define ROBOMAGELLAN_HARDWARE_INTERFACE_SERIALPORT_H

class SerialPort {
public:

  SerialPort() = default;

  ~SerialPort();

  bool Open(std::string device, unsigned int baud);

  void Close();

  void Write(std::string message);

  std::string ReadLine();

  bool IsOpen();

private:

  int port_handle_{-1};

  void SetProperties(unsigned int baud);

};

#endif //ROBOMAGELLAN_HARDWARE_INTERFACE_SERIALPORT_H
