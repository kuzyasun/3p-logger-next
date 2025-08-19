# Lolin S2 Mini Documentation

## Overview

The **Lolin S2 Mini** is a compact WiFi development board based on the ESP32-S2FN4R2 chip. It's designed to be compatible with LOLIN D1 mini shields and supports multiple development frameworks.

**Official Documentation**: https://www.wemos.cc/en/latest/s2/s2_mini.html

## Specifications

### Hardware Specifications

| Feature | Specification |
|---------|---------------|
| **Chip** | ESP32-S2FN4R2 |
| **Cores** | Single-core Xtensa LX7 (240MHz) |
| **RAM** | 320KB SRAM + 2MB PSRAM |
| **Flash** | 4MB SPI Flash |
| **USB** | Type-C USB with OTG support |
| **WiFi** | 802.11b/g/n (802.11n up to 150 Mbps) |
| **GPIO Pins** | 27 programmable GPIO pins |
| **Size** | 34.3 × 25.4mm |
| **Weight** | 2.4g |
| **Operating Voltage** | 3.3V |

### Supported Frameworks

- **MicroPython** (default firmware)
- **Arduino**
- **CircuitPython**
- **ESP-IDF**

### Communication Interfaces

| Interface | Count | Details |
|-----------|-------|---------|
| **UART** | 2 | UART0 (USB CDC), UART1 (configurable pins) |
| **SPI** | 4 | SPI1, SPI2, SPI3, SPI4 (General Purpose SPI) |
| **I2C** | 2 | I2C0, I2C1 (configurable pins) |
| **RMT** | 4 | Remote Control Transceiver (TX/RX) |
| **DMA** | Yes | Direct Memory Access supported |
| **PWM** | 8 | LED PWM controller channels |
| **ADC** | 2 | ADC1 (10 channels), ADC2 (10 channels) |
| **DAC** | 2 | 8-bit DAC channels |
| **Touch** | 14 | Touch sensing IOs |
| **GPIO** | 47 | Programmable GPIO pins (27 exposed on board) |

## Pin Layout

The Lolin S2 Mini has 27 GPIO pins arranged in a compact form factor. The pinout diagram is available at:
https://www.wemos.cc/en/latest/_static/boards/s2_mini_v1.0.0_4_16x9.jpg

### Pin Assignment

| Pin | GPIO | Function | Notes |
|-----|------|----------|-------|
| 1 | 3V3 | Power | 3.3V power supply |
| 2 | GND | Ground | Ground |
| 3 | IO0 | GPIO0 | Boot mode (must be HIGH for normal operation) |
| 4 | IO1 | GPIO1 | ADC1_CH0, TOUCH_CH1 |
| 5 | IO2 | GPIO2 | ADC1_CH1, TOUCH_CH2 |
| 6 | IO3 | GPIO3 | ADC1_CH2, TOUCH_CH3 |
| 7 | IO4 | GPIO4 | ADC1_CH3, TOUCH_CH4 |
| 8 | IO5 | GPIO5 | ADC1_CH4, TOUCH_CH5 |
| 9 | IO6 | GPIO6 | ADC1_CH5, TOUCH_CH6 |
| 10 | IO7 | GPIO7 | ADC1_CH6, TOUCH_CH7 |
| 11 | IO8 | GPIO8 | ADC1_CH7, TOUCH_CH8 |
| 12 | IO9 | GPIO9 | ADC1_CH8, TOUCH_CH9 |
| 13 | IO10 | GPIO10 | ADC1_CH9, TOUCH_CH10 |
| 14 | IO11 | GPIO11 | ADC2_CH0, TOUCH_CH11 |
| 15 | IO12 | GPIO12 | ADC2_CH1, TOUCH_CH12 |
| 16 | IO13 | GPIO13 | ADC2_CH2, TOUCH_CH13 |
| 17 | IO14 | GPIO14 | ADC2_CH3, TOUCH_CH14 |
| 18 | IO15 | GPIO15 | ADC2_CH4, XTAL_32K_P |
| 19 | IO16 | GPIO16 | ADC2_CH5, XTAL_32K_N |
| 20 | IO17 | GPIO17 | ADC2_CH6, DAC_1 |
| 21 | IO18 | GPIO18 | ADC2_CH7, DAC_2 |
| 22 | IO19 | GPIO19 | USB_D+ |
| 23 | IO20 | GPIO20 | USB_D- |
| 24 | IO21 | GPIO21 | |
| 25 | IO22 | GPIO22 | |
| 26 | IO23 | GPIO23 | |
| 27 | IO24 | GPIO24 | |
| 28 | IO25 | GPIO25 | |
| 29 | IO26 | GPIO26 | |
| 30 | IO27 | GPIO27 | |
| 31 | IO28 | GPIO28 | |
| 32 | IO29 | GPIO29 | |
| 33 | IO30 | GPIO30 | |
| 34 | IO31 | GPIO31 | |
| 35 | IO32 | GPIO32 | |
| 36 | IO33 | GPIO33 | |
| 37 | IO34 | GPIO34 | |
| 38 | IO35 | GPIO35 | |
| 39 | IO36 | GPIO36 | |
| 40 | IO37 | GPIO37 | |
| 41 | IO38 | GPIO38 | |
| 42 | IO39 | GPIO39 | |
| 43 | IO40 | GPIO40 | |
| 44 | IO41 | GPIO41 | |
| 45 | IO42 | GPIO42 | |
| 46 | IO43 | GPIO43 | |
| 47 | IO44 | GPIO44 | |
| 48 | IO45 | GPIO45 | Strapping pin (must be LOW for normal operation) |
| 49 | IO46 | GPIO46 | Input-only pin |

## GPIO Pin Compatibility

### Safe GPIO Pins for Lolin S2 Mini

| Function | Safe Pins | Notes |
|----------|-----------|-------|
| **I2C** | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 | Avoid 0 (boot), 45 (strapping), 46 (input-only) |
| **PWM** | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 | 8 channels available |
| **ADC1** | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 | 0-3.3V range |
| **ADC2** | 11, 12, 13, 14, 15, 16, 17, 18, 19, 20 | Not available during WiFi |
| **UART** | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 | Avoid 0, 45, 46 |
| **SPI** | 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44 | Flexible assignment |

### Restricted Pins

- **GPIO 0**: Boot mode (must be HIGH for normal operation)
- **GPIO 45**: Strapping pin (must be LOW for normal operation)
- **GPIO 46**: Input-only pin

### Special Function Pins

- **GPIO 19, 20**: USB D+ and D- (when using USB OTG)
- **GPIO 17, 18**: DAC outputs
- **GPIO 15, 16**: 32K crystal pins (XTAL_32K_P, XTAL_32K_N)

## Communication Interfaces Details

### UART (Universal Asynchronous Receiver-Transmitter)

**Supported UARTs**: 2
- **UART0**: USB CDC (built-in, used for programming and debugging)
- **UART1**: Configurable pins (can be assigned to any GPIO)

**Features**:
- Full-duplex communication
- Configurable baud rates (up to 5 Mbps)
- Hardware flow control support
- DMA support for efficient data transfer
- Interrupt-driven operation

**Recommended UART1 Pins**:
```c
#define UART1_TX_GPIO 21
#define UART1_RX_GPIO 22
```

### SPI (Serial Peripheral Interface)

**Supported SPI Buses**: 4 (SPI1, SPI2, SPI3, SPI4)

**Features**:
- Master and slave modes
- Configurable clock speeds (up to 80 MHz)
- DMA support for efficient data transfer
- Multiple chip select lines
- Full-duplex communication

**SPI Pin Configuration**:
```c
// SPI1 Configuration
#define SPI1_SCLK_GPIO 6
#define SPI1_MOSI_GPIO 7
#define SPI1_MISO_GPIO 2
#define SPI1_CS_GPIO 10

// SPI2 Configuration
#define SPI2_SCLK_GPIO 12
#define SPI2_MOSI_GPIO 13
#define SPI2_MISO_GPIO 14
#define SPI2_CS_GPIO 15
```

### I2C (Inter-Integrated Circuit)

**Supported I2C Buses**: 2 (I2C0, I2C1)

**Features**:
- Master and slave modes
- Standard (100 kHz) and Fast (400 kHz) modes
- Fast Plus mode (1.7 MHz) supported
- DMA support for efficient data transfer
- Clock stretching support
- Multi-master support

**Recommended I2C Pin Configurations**:
```c
// I2C0 Configuration
#define I2C0_SDA_GPIO 8
#define I2C0_SCL_GPIO 9

// I2C1 Configuration
#define I2C1_SDA_GPIO 4
#define I2C1_SCL_GPIO 5
```

### RMT (Remote Control Transceiver)

**Supported RMT Channels**: 4 (TX/RX pairs)

**Features**:
- Infrared remote control signal generation and reception
- Custom protocol support
- High precision timing
- DMA support
- Carrier frequency generation

**RMT Pin Configuration**:
```c
// RMT TX Channel
#define RMT_TX_GPIO 17

// RMT RX Channel
#define RMT_RX_GPIO 18
```

### DMA (Direct Memory Access)

**DMA Support**: Yes

**Features**:
- Multiple DMA channels available
- Supports UART, SPI, I2C, ADC, DAC
- Efficient data transfer without CPU intervention
- Configurable transfer sizes
- Interrupt-driven completion notification

**DMA Benefits**:
- Reduced CPU overhead
- Higher data transfer rates
- Better power efficiency
- Improved real-time performance

### PWM (Pulse Width Modulation)

**Supported PWM Channels**: 8

**Features**:
- Configurable frequency and duty cycle
- High resolution (16-bit)
- Multiple output channels
- LED dimming support
- Servo control support

**PWM Pin Configuration**:
```c
// PWM Channels
#define PWM_CH0_GPIO 1
#define PWM_CH1_GPIO 2
#define PWM_CH2_GPIO 3
#define PWM_CH3_GPIO 4
#define PWM_CH4_GPIO 5
#define PWM_CH5_GPIO 6
#define PWM_CH6_GPIO 7
#define PWM_CH7_GPIO 8
```

## Recommended Pin Configurations

### For I2C Communication

```c
// Lolin S2 Mini - Recommended
#define I2C_IMU_SDA_GPIO 8
#define I2C_IMU_SCL_GPIO 9
```

### For PWM/Servo Control

```c
// Lolin S2 Mini - Recommended
#define SERVO_PAN_GPIO 4
#define SERVO_TILT_GPIO 5
```

### For ADC/Battery Monitoring

```c
// Lolin S2 Mini - ADC1 (always available)
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 1
```

### For UART Communication

```c
// Lolin S2 Mini - Recommended
#define UART_TX_GPIO 21
#define UART_RX_GPIO 22
```

### For SPI Communication

```c
// Lolin S2 Mini - SPI Configuration
#define SPI_SCLK_GPIO 6
#define SPI_MOSI_GPIO 7
#define SPI_MISO_GPIO 2
#define SPI_CS_GPIO 10
```

### For RMT (Infrared)

```c
// Lolin S2 Mini - RMT Configuration
#define RMT_TX_GPIO 17
#define RMT_RX_GPIO 18
```

## Hardware Features

### Built-in Components

- **USB Type-C Connector**: For programming and power
- **Reset Button**: Hardware reset functionality
- **BOOT Button**: GPIO 0 (download mode when pressed during boot)
- **Onboard LED**: Not specified in documentation (may vary by version)

### Power Supply

- **Operating Voltage**: 3.3V
- **USB Power**: 5V via Type-C connector
- **External Power**: 3.3V via 3V3 pin

## Development Setup

### PlatformIO Configuration

```ini
[env:lolin_s2_mini]
board = lolin_s2_mini
platform = espressif32@6.12.0
framework = espidf
monitor_speed = 115200
upload_speed = 921600
board_build.f_cpu = 160000000L
monitor_rts = 1
monitor_dtr = 1
build_flags =	
	-D CONFIG_FREERTOS_HZ=1000
	-D BOARDLED_PIN=15
	-D DEFAULT_RX=16
	-D DEFAULT_TX=18		
  	-D CONFIG_ESP_CONSOLE_USB_CDC=1
  	-D CONFIG_USB_ENABLED=1
  	-D CONFIG_USB_CDC_ENABLED=1
```

## Development Tips

1. **Always check pin compatibility** before connecting hardware
2. **Use pull-up resistors** for I2C lines (4.7kΩ recommended)
3. **Avoid using restricted pins** for critical functions
4. **Test pin functionality** with simple blink examples first
5. **Consider power requirements** when using multiple peripherals
6. **GPIO 0 must be HIGH** for normal operation (connected to BOOT button)
7. **GPIO 45 must be LOW** for normal operation (strapping pin)
8. **GPIO 46 is input-only** and cannot be used as output

### Communication Interface Tips

9. **UART**: UART0 is used for USB CDC - use UART1 for external communication
10. **SPI**: Use DMA for high-speed data transfer (up to 80 MHz clock)
11. **I2C**: Use Fast mode (400 kHz) for better performance with compatible devices
12. **RMT**: Perfect for infrared remote control applications
13. **DMA**: Enable DMA for UART, SPI, and I2C to reduce CPU overhead
14. **PWM**: Use for LED dimming, servo control, and motor speed control

## Hardware Requirements

- **Power Supply**: 3.3V regulated power supply
- **I2C Pull-ups**: 4.7kΩ resistors on SDA and SCL lines
- **Servo Power**: Separate 5V supply for servo motors
- **USB Cable**: High-quality USB Type-C cable for programming and debugging

## Troubleshooting

### Common Issues

1. **Board not detected**: Check USB cable and drivers
2. **Upload fails**: Hold BOOT button during upload
3. **GPIO not working**: Check if pin is restricted or used by system
4. **I2C communication issues**: Verify pull-up resistors and pin assignments

### Communication Interface Issues

5. **UART communication problems**: 
   - Check baud rate settings
   - Verify TX/RX pin connections
   - Ensure UART0 is not used for external communication

6. **SPI communication issues**:
   - Verify clock polarity and phase settings
   - Check chip select pin configuration
   - Ensure proper voltage levels (3.3V)

7. **I2C communication failures**:
   - Add external pull-up resistors (4.7kΩ)
   - Check device addresses
   - Verify clock frequency settings

8. **RMT signal problems**:
   - Check carrier frequency settings
   - Verify signal timing
   - Ensure proper IR LED/receiver connections

9. **DMA transfer issues**:
   - Check buffer alignment requirements
   - Verify transfer size limits
   - Ensure proper interrupt handling

## Resources

- **Official Documentation**: https://www.wemos.cc/en/latest/s2/s2_mini.html
- **Schematic**: https://www.wemos.cc/en/latest/_static/files/sch_s2_mini_v1.0.0.pdf
- **Dimensions**: https://www.wemos.cc/en/latest/_static/files/dim_s2_mini_v1.0.0.pdf
- **ESP32-S2 Datasheet**: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
- **Purchase Link**: https://www.aliexpress.com/item/1005003145192016.html

## Comparison with ESP32-S2-Saola-1

| Feature | Lolin S2 Mini | ESP32-S2-Saola-1 |
|---------|---------------|-------------------|
| **Size** | 34.3 × 25.4mm | Larger |
| **GPIO Pins** | 27 | 43 |
| **USB** | Type-C | Micro USB |
| **Shield Compatibility** | D1 mini shields | Standard headers |
| **Form Factor** | Compact | Standard development board |
| **Price** | Lower | Higher |

The Lolin S2 Mini is a more compact and cost-effective alternative to the ESP32-S2-Saola-1, with fewer GPIO pins but better shield compatibility and a smaller form factor.
