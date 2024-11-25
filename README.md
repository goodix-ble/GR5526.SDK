# GR5526 Series SoC



## 1. Introduction

- The Goodix GR5526 family is a single-mode, low-power Bluetooth 5.3 System-on-Chip (SoC). It can be configured as a Broadcaster, an Observer, a Central, a Peripheral, and supports the combination of all the above roles, making it an ideal choice for Internet of Things (IoT), LE Audio, and smart wearable devices. In addition, it supports Bluetooth Low Energy (Bluetooth LE) Direction Finding: angle of arrival (AoA) and angle of departure (AoD), as well as LE Isochronous Channels.

- Based on ARM® Cortex®-M4F CPU core, the GR5526 series integrates Bluetooth 5.3 Protocol Stack, a 2.4 GHz RF transceiver, on-chip programmable Flash memory, RAM, and multiple peripherals. GR5526 delivers a feature-rich display and graphics solution by providing the option of graphics acceleration (GPU + DC) and system-in-package (SiP) pseudostatic RAM (PSRAM) to accommodate display while still leaving plenty of resource for wearable schemes.



## 2. Key Features


- Bluetooth Low Energy 5.3 transceiver integrating Controller and Host layers
  - Supported data rates: 1 Mbps, 2 Mbps, and Long Range (500 kbps, 125 kbps)
  - TX power: -20 dBm to +7 dBm
  - -98 dBm sensitivity (in 1 Mbps mode)
  - -94 dBm sensitivity (in 2 Mbps mode)
  - -101 dBm sensitivity (in Long Range 500 kbps mode)
  - -104 dBm sensitivity (in Long Range 125 kbps mode)
  - TX current: 4.0 mA @ 0 dBm, 1 Mbps
  - RX current: 3.5 mA @ 1 Mbps
  - AoA/AoD, LE Isochronous Channels
- ARM® Cortex®-M4F 32-bit micro-processor with floating point support
  - Up to 96 MHz clock frequency
  - Built-in Memory Protection Unit (MPU) supporting eight programmable regions
  - Hardware Floating Point Unit (FPU)
  - Built-in Nested Vectored Interrupt Controller (NVIC)
  - Non-maskable Interrupt (NMI) input
  - Serial Wire Debug (SWD) with 16 breakpoints, two watchpoints, and a debug timestamp counter
  - 51 µA/MHz execution from Flash @ 3.3 V, 96 MHz
- On-chip memory
  - 512 KB data SRAM with retention capabilities
  - 8 KB cache SRAM with retention capabilities
  - Stack ROM (including boot ROM and Bluetooth LE Stack)
  - 1 MB internal QSPI Flash
  - 8 MB internal PSRAM (for GR5526VGBIP and GR5526RGNIP only)
- Digital peripherals
  - Two general-purpose DMA engines, each with 6 channels and up to 16 programmable request/trigger sources
  - USB 2.0 full speed (12 Mbps) controller with on-chip PHY and dedicated DMA controller
  - Internal Octal SPI DDR interfaces to support 8 MB internal PSRAM at up to 48 MHz (for GR5526VGBIP and GR5526RGNIP only)
- Analog peripherals
  - One 13-bit Sense ADC with a sampling rate of 1 Msps. It supports up to 8 external I/O channels and 3 internal signal channels
  - Built-in temperature and voltage sensors
  - Low-power comparator, supporting wakeup from deep sleep mode
- Flexible serial peripherals
  - 6 * UART modules up to 4 Mbps, with all modules supporting flow control and IrDA
  - 6 * I2C modules for peripheral communication, up to 3.4 MHz
  - 1 * 8-bit/16-bit/32-bit SPI master interface and 1 x SPI slave interface for host communication
  - 2 * I2S interfaces (1 I2S master interface and 1 I2S slave interface)
  - PDM interface with hardware sampling rate converter
  - 1 * ISO7816 interface

- Display/Graphics
  - 2.5D GPU for graphics acceleration (for GR5526VGBIP and GR5526RGNIP only)
  - 1 * Dual-lane SPI (DSPI) interface for display, with Mobile Industry Processor Interface (MIPI) Display Bus Interface (DBI) Type-C support
  - 3 * Quad SPI (QSPI) interfaces, up to 48 MHz, supporting direct access via memory mapping when connecting with external NOR Flash
  - Display Control (DC) module with MIPI DBI Type-C support and 2D graphics blending integrated (for GR5526VGBIP and GR5526RGNIP only)
- Security
  - Complete secure computing engine:
    - AES 128-bit/192-bit/256-bit symmetric encryption (ECB, CBC)
    - Hash-based Message Authentication Code (HMAC-SHA256)
    - Public key cryptography (PKC)
    - True random number generator (TRNG)
  - Comprehensive security operation mechanism:
    - Secure boot
    - Encrypted firmware runs directly from Flash
    - eFuse for encrypted key storage
    - Differentiate application data key and firmware key, supporting one data key per each device/product
- I/O Peripherals
  - Up to 50 multiplexed I/O pins in total
    - Up to 34 general-purpose I/O(GPIO) pins
    - Up to 8 always-on I/O(AON IO) pins, supporting wakeup from deep sleep mode
    - Up to 8 mixed signal I/O(MSIO) pin, configurable to be digital/analog signal interfaces
- Timer
  - Two general-purpose, 32-bit timer modules
  - A timer module composed of two programmable 32-bit or 16 bit down counters
  - An internal sleep timer that can be used to wake the device up from deep sleep mode
  - Two PWM modules with edge alignment mode and center alignment mode, each with 3 channels
  - 2 * real-time counters (RTC): 1 * Calendar, 1 * RTC
- Power management
  - On-chip DC-DC to provide RF Analog voltage and supply core LDO
  - On-chip I/O LDO to provide I/O voltage and supply external components, maximum I/O LDO drive strength: 30 mA
  - Programmable thresholds for brownout detection (BOD)
  - Supply voltage: 2.4 V – 4.35 V
  - I/O voltage: 1.8 V – 3.6 V
- Low-power consumption modes
  - Sleep mode: 3.3 µA (Typical) at 3.3 V VBAT input with 128 KB SRAM retention on and LFXO_32K off; woken up by 8 sources of always-on domain
  - Ultra deep sleep mode: 2.4 µA (Typical); internal power (all SRAM included) and LFXO_32K removed from entire chip except always-on domain; woken up by Sleep Timer and AON GPIOs
  - OFF mode: 200 nA (Typical); nothing on except VBAT, and chip in reset mode
- Packages
  - BGA83: 4.3 mm * 4.3 mm * 0.96 mm, 0.4 mm pitch
  - QFN68: 7.0 mm * 7.0 mm * 0.85 mm , 0.35 mm pitch
- Operating temperature range: -40°C to +85°C



## 3. Product Details

|                       |                    | GR5526VGBIP                                                  | GR5526VGBI                                                   | **GR5526RGNIP**                                              | GR5526RGNI                                                   |
| :-------------------- | :----------------- | :----------------------------------------------------------- | :----------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Status                |                    | Active                                                       | Active                                                       | Active                                                       | Active                                                       |
| Protocol              | Bluetooth LE [1]   | 5.3                                                          | 5.3                                                          | 5.3                                                          | 5.3                                                          |
|                       | ISO                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | Direction Finding  | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | Bluetooth Mesh     | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
| Core System           | CPU                | Cortex®-M4F                                                  | Cortex®-M4F                                                  | Cortex®-M4F                                                  | Cortex®-M4F                                                  |
|                       | Clocks             | 96 MHz / 32 KHz                                              | 96 MHz / 32 KHz                                              | 96 MHz / 32 KHz                                              | 96 MHz / 32 KHz                                              |
|                       | Cache              | 8 KB                                                         | 8 KB                                                         | 8 KB                                                         | 8 KB                                                         |
|                       | RAM                | 512 KB                                                       | 512 KB                                                       | 512 KB                                                       | 512 KB                                                       |
|                       | PSRAM              | 8 MB                                                         |                                                              | 8 MB                                                         |                                                              |
|                       | Flash              | 1 MB                                                         | 1 MB                                                         | 1 MB                                                         | 1 MB                                                         |
| GPU                   | 2.5D GPU           | ●                                                            |                                                              | ●                                                            |                                                              |
| Security              | Root-Of-Trust      | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | Secure Key Store   | 4                                                            | 4                                                            | 4                                                            | 4                                                            |
|                       | PKC                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | RSA                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | AES                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | ECC                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | TRNG               | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
| Radio                 | Frequency          | 2.4 GHz                                                      | 2.4 GHz                                                      | 2.4 GHz                                                      | 2.4 GHz                                                      |
|                       | Maximum Tx Power   | 7 dBm                                                        | 7 dBm                                                        | 7 dBm                                                        | 7 dBm                                                        |
|                       | Rx Sensitivity     | -98 dBm（@1 Mbps）                                           | -98 dBm（@1 Mbps）                                           | -98 dBm（@1 Mbps）                                           | -98 dBm（@1 Mbps）                                           |
| Peripheral            | UART               | 6                                                            | 6                                                            | 6                                                            | 6                                                            |
|                       | SPI                | 1 * SPIM / 1 * SPIS                                          | 1 * SPIM / 1 * SPIS                                          | 1 * SPIM / 1 * SPIS                                          | 1 * SPIM / 1 * SPIS                                          |
|                       | I2C                | 6                                                            | 6                                                            | 6                                                            | 6                                                            |
|                       | USB                | 1 * USB 2.0                                                  | 1 * USB 2.0                                                  | 1 * USB 2.0                                                  | 1 * USB 2.0                                                  |
|                       | QSPI               | 3                                                            | 3                                                            | 3                                                            | 3                                                            |
|                       | DSPI               | 1                                                            | 1                                                            | 1                                                            | 1                                                            |
|                       | DC                 | 1                                                            |                                                              | 1                                                            |                                                              |
|                       | Timers             | 4                                                            | 4                                                            | 4                                                            | 4                                                            |
|                       | PWM                | 2                                                            | 2                                                            | 2                                                            | 2                                                            |
|                       | RTC                | 2                                                            | 2                                                            | 2                                                            | 2                                                            |
|                       | I2S                | 1 * I2SM / 1 * I2SS                                          | 1 * I2SM / 1 * I2SS                                          | 1 * I2SM / 1 * I2SS                                          | 1 * I2SM / 1 * I2SS                                          |
|                       | PDM                | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | ADC                | 13-bit                                                       | 13-bit                                                       | 13-bit                                                       | 13-bit                                                       |
|                       | Comparator         | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | Temperature sensor | ●                                                            | ●                                                            | ●                                                            | ●                                                            |
|                       | GPIO               | 50                                                           | 50                                                           | 48                                                           | 48                                                           |
| Packages              | TypeDimensions     | BGA83 4.3 * 4.3 mm                                           | BGA83 4.3 * 4.3 mm                                           | QFN68 7.0 * 7.0 mm                                           | QFN68 7.0 * 7.0 mm                                           |
| Certification         |                    | SIG BQB (QDID: 179976, [181083](https://launchstudio.bluetooth.com/ListingDetails/144553)) | SIG BQB (QDID: 179976, [181083](https://launchstudio.bluetooth.com/ListingDetails/144553)) | SIG BQB (QDID: 179976, [181083](https://launchstudio.bluetooth.com/ListingDetails/144553)) | SIG BQB (QDID: 179976, [181083](https://launchstudio.bluetooth.com/ListingDetails/144553)) |
| Operating Temperature |                    | -40 ℃ ~ 85 ℃                                                 | -40 ℃ ~ 85 ℃                                                 | -40 ℃ ~ 85 ℃                                                 | -40 ℃ ~ 85 ℃                                                 |
| Supply Voltage Range  |                    | 2.4 V - 4.35 V                                               | 2.4 V - 4.35 V                                               | 2.4 V - 4.35 V                                               | 2.4 V - 4.35 V                                               |
| Development Kits      |                    | GR5526 Starter Kit                                           | GR5526 Starter Kit                                           | GR5526 Starter Kit                                           | GR5526 Starter Kit                                           |



- Notes:  [1] 1M-PHY, 2M-PHY, Long Range supported by default.



## 4. Change Log


- Click to view the [change log](../../wiki)

