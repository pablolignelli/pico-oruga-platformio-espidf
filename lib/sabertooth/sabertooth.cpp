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

#include "sabertooth.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/uart.h>
#include <esp_log.h>

static const char *TAG = "sabert";

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

char Sabertooth::_address;
uart_port_t Sabertooth::_uart_num;

/*
Sabertooth::Sabertooth(char address)
  : _address(address), _port(SabertoothTXPinSerial)
{}
*/

Sabertooth::Sabertooth() {}

void Sabertooth::init(char address, uart_port_t uart_num, int tx_pin)
{
  ESP_LOGD(TAG, "Setting up...");
  
  _address = address;
  _uart_num = uart_num;

  // Setup UART buffered IO with event queue
  ESP_ERROR_CHECK(uart_driver_install(_uart_num,
                                      UART_HW_FIFO_LEN(_uart_num)+4, UART_HW_FIFO_LEN(_uart_num)+4,
                                      0, NULL, 0));

  uart_config_t uart_config = {
      .baud_rate = 9600,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      //.rx_flow_ctrl_thresh = 122,
  };
  // Configure UART parameters
  // SABERTOOTH_SERIAL.begin(9600, SERIAL_8N1, -1, SABERTOOTH_TX_PIN);
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
  ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  ESP_LOGD(TAG, "Setting up done.");
}

void Sabertooth::autobaud(bool dontWait)
{
  autobaud(_uart_num, dontWait);
}

void Sabertooth::autobaud(uart_port_t uart_num, bool dontWait)
{
  if (!dontWait)
  {
    vTaskDelay(pdMS_TO_TICKS(1500));
  }
  char aa = 0xAA;
  uart_write_bytes(uart_num, &aa, sizeof(aa));

  if (!dontWait)
  {
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void Sabertooth::command(char cmd, char value)
{
  char s[] = {
      _address,
      cmd,
      value,
      static_cast<char>((static_cast<char>(_address + cmd + value)) & static_cast<char>(0b01111111))};

  //ESP_LOGD(TAG, "command [%02x %02x %02x %02x]", s[0], s[1], s[2], s[3]);
  uart_write_bytes(_uart_num, &s, sizeof(s));
}

void Sabertooth::throttleCommand(char cmd, int power)
{
  power = abs(power);
  if (power > 126)
    power = 126;
  command(cmd, (char)power);
}

void Sabertooth::motor(int power)
{
  motor(1, power);
}

void Sabertooth::motor(char motor, int power)
{
  if (motor < 1 || motor > 2)
  {
    return;
  }
  throttleCommand((motor == 2 ? 4 : 0) + (power < 0 ? 1 : 0), power);
}

void Sabertooth::drive(int power)
{
  throttleCommand(power < 0 ? 9 : 8, power);
}

void Sabertooth::turn(int power)
{
  throttleCommand(power < 0 ? 11 : 10, power);
}

void Sabertooth::stop()
{
  motor(1, 0);
  motor(2, 0);
}

void Sabertooth::setMinVoltage(char value)
{
  if (value > 120)
    value = 120;
  command(2, value);
}

void Sabertooth::setMaxVoltage(char value)
{
  if (value > 127)
    value = 127;
  command(3, value);
}

void Sabertooth::setBaudRate(long baudRate)
{
#if defined(ARDUINO) && ARDUINO >= 100
  port().flush();
#endif

  char value;
  switch (baudRate)
  {
  case 2400:
    value = 1;
    break;
  case 9600:
  default:
    value = 2;
    break;
  case 19200:
    value = 3;
    break;
  case 38400:
    value = 4;
    break;
  case 115200:
    value = 5;
    break;
  }
  command(15, value);

  // (1) flush() does not seem to wait until transmission is complete.
  //     As a result, a Serial.end() directly after this appears to
  //     not always transmit completely. So, we manually add a delay.
  // (2) Sabertooth takes about 200 ms after setting the baud rate to
  //     respond to commands again (it restarts).
  // So, this 500 ms delay should deal with this.
  vTaskDelay(pdMS_TO_TICKS(500));
}

void Sabertooth::setDeadband(char value)
{
  if (value > 127)
    value = 127;
  command(17, value);
}

void Sabertooth::setRamping(char value)
{
  // command(16, (char)constrain(value, 0, 80));
  if (value > 80)
  {
    value = 80;
  }
  command(16, value);
}

void Sabertooth::setTimeout(int milliseconds)
{
  if (milliseconds < 0)
    milliseconds = 0;
  else if (milliseconds > 12700)
    milliseconds = 12700;
  milliseconds += 99;

  command(14, (char)(milliseconds / 100));
}
