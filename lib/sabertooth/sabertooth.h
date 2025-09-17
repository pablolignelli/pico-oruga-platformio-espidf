/*
Arduino Library for SyRen/Sabertooth Packet Serial
Copyright (c) 2012-2013 Dimension Engineering LLC
http://www.dimensionengineering.com/arduino

Permission to use, copy, modify, and/or distribute this software for any
purpose with or without fee is hereby granted, provided that the above
copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER
RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE
USE OR PERFORMANCE OF THIS SOFTWARE.
*/

#include <hal/uart_types.h>
#ifndef __Sabertooth_h
#define __Sabertooth_h

/*!
\class Sabertooth
\brief Controls a %Sabertooth or %SyRen motor driver running in Packet Serial mode.
*/
class Sabertooth
{
public:
  /*!
  Initializes a new instance of the Sabertooth class.
  The driver _address is set to the value given, and the Arduino TX serial port is used.
  \param address The driver _address.
  */
  // Sabertooth(char _address);

  /*!
  Initializes a new instance of the Sabertooth class.
  The driver _address is set to the value given, and the specified serial port is used.
  \param address The driver _address.
  \param port    The port to use.
  */
  Sabertooth();
  static void init(char address, uart_port_t uart_num, int tx_pin);

public:
  /*!
  Gets the driver _address.
  \return The driver _address.
  */
  static inline char address()
  {
    return _address;
  }

  /*!
  Gets the serial port.
  \return The serial port.
  */
  static inline uart_port_t uart_num()
  {
    return _uart_num;
  }

  /*!
  Sends the autobaud character.
  \param dontWait If false, a delay is added to give the driver time to start up.
  */
  static void autobaud(bool dontWait = false);

  /*!
  Sends the autobaud character.
  \param port     The port to use.
  \param dontWait If false, a delay is added to give the driver time to start up.
  */
  static void autobaud(uart_port_t uart_num, bool dontWait = false);

  /*!
  Sends a packet serial command to the motor driver.
  \param command The number of the command.
  \param value   The command's value.
  */
  static void command(char command, char value);

public:
  /*!
  Sets the power of motor 1.
  \param power The power, between -127 and 127.
  */
  static void motor(int power);

  /*!
  Sets the power of the specified motor.
  \param motor The motor number, 1 or 2.
  \param power The power, between -127 and 127.
  */
  static void motor(char motor, int power);

  /*!
  Sets the driving power.
  \param power The power, between -127 and 127.
  */
  static void drive(int power);

  /*!
  Sets the turning power.
  \param power The power, between -127 and 127.
  */
  static void turn(int power);

  /*!
  Stops.
  */
  static void stop();

public:
  /*!
  Sets the minimum voltage.
  \param value The voltage. The units of this value are driver-specific and are specified in the Packet Serial chapter of the driver's user manual.
  */
  static void setMinVoltage(char value);

  /*!
  Sets the maximum voltage.
  Maximum voltage is stored in EEPROM, so changes persist between power cycles.
  \param value The voltage. The units of this value are driver-specific and are specified in the Packet Serial chapter of the driver's user manual.
  */
  static void setMaxVoltage(char value);

  /*!
  Sets the baud rate.
  Baud rate is stored in EEPROM, so changes persist between power cycles.
  \param baudRate The baud rate. This can be 2400, 9600, 19200, 38400, or on some drivers 115200.
  */
  static void setBaudRate(long baudRate);

  /*!
  Sets the deadband.
  Deadband is stored in EEPROM, so changes persist between power cycles.
  \param value The deadband value.
               Motor powers in the range [-deadband, deadband] will be considered in the deadband, and will
               not prevent the driver from entering nor cause the driver to leave an idle brake state.
               0 resets to the default, which is 3.
  */
  static void setDeadband(char value);

  /*!
  Sets the ramping.
  Ramping is stored in EEPROM, so changes persist between power cycles.
  \param value The ramping value. Consult the user manual for possible values.
  */
  static void setRamping(char value);

  /*!
  Sets the serial timeout.
  \param milliseconds The maximum time in milliseconds between packets. If this time is exceeded,
                      the driver will stop the motors. This value is rounded up to the nearest 100 milliseconds.
                      This library assumes the command value is in units of 100 milliseconds. This is true for
                      most drivers, but not all. Check the packet serial chapter of the driver's user manual
                      to make sure.
  */
  static void setTimeout(int milliseconds);

private:
  static void throttleCommand(char cmd, int power);

private:
  static char _address;
  // SabertoothStream &port;
  static uart_port_t _uart_num;
};

#endif
