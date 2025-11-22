# Real-Time Clock Display on 16x2 LCD Using STM32F407 (Bare-Metal)

This project implements a complete real-time clock (RTC) system using the STM32F407 Discovery (STM32F407G-DISC1), the DS1307 RTC module, and a 16x2  character LCD.  
The firmware is written entirely in bare-metal C, including custom drivers for GPIO, I2C, SysTick, and the LCD.

The microcontroller reads time and date continuously from the DS1307 over I²C and displays it on the LCD in a human-readable format.

---

## Features

- Bare-metal I2C driver (START, STOP, ACK, repeated START, addressing)
- DS1307 RTC read/write support (time, date, day, month, year)
- 12-hour (AM/PM) and 24-hour format support
- 16x2 LCD driver in 4-bit mode (custom GPIO-based implementation)
- Time and date auto-refresh every 1 second using SysTick
- BCD <-> binary conversion logic integrated
- Minimal dependencies (no HAL/LL)

---

## Hardware Used

- STM32F407 Discovery Board (STM32F407G-DISC1)
- DS1307 RTC Module
- 16x2 Character LCD 
- Jumper wires  
- Potentiometer for LCD contrast

---

## Pin Connections

### DS1307 --> STM32F407 (I2C1)
| DS1307 Pin | STM32F407 Pin |
|------------|----------------|
| SDA        | PB7 (I2C1 SDA) |
| SCL        | PB6 (I2C1 SCL) |
| VCC        | 5V             |
| GND        | GND            |

---

## 16x2 LCD --> STM32F407 (GPIOD)
| LCD Pin | STM32F407 Pin |
|---------|----------------|
| RS      | PD0 |
| RW      | PD1 |
| EN      | PD2 |
| D4      | PD3 |
| D5      | PD4 |
| D6      | PD5 |
| D7      | PD6 |
| VSS     | GND |
| VDD     | 5V |
| V0      | Potentiometer (contrast) |

---
