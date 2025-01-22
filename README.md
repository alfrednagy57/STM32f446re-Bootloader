# STM32F446RE Bootloader

## Overview
This repository contains a UDS-like protocol bootloader designed for the STM32F446RE microcontroller, developed using STM32CubeIDE HAL drivers. The bootloader implements 11 diagnostic services inspired by the Universal Diagnostic Services (UDS) protocol, enabling secure, reliable, and efficient firmware updates.

## Features
- **UDS-Like Services**: Implements 11 services for comprehensive firmware management, including memory operations, system control, and diagnostic utilities.
- **CRC Integrity Check**: Ensures data integrity during firmware transmission.
- **Modular and Scalable Design**: Easily adaptable to other STM32 devices.
- **UART Communication**: Seamless communication between the host and target.

## Supported Services
The bootloader supports the following services:
1. Bootloader version retrieval
2. Supported services inquiry
3. Chip ID retrieval
4. Read protection status check
5. Address execution
6. Flash memory erasure
7. Memory write
8. Read/write protection management
9. Memory read
10. Sector protection status check
11. Option byte reading

## Development Environment
- **Microcontroller**: STM32F446RE
- **IDE**: STM32CubeIDE
- **Communication Protocol**: UART

## Getting Started
1. Clone the repository:
   ```bash
   git clone https://github.com/alfrednagy57/STM32f446re-Bootloader.git
   ```
2. Open the project in STM32CubeIDE.
3. Flash the firmware to the STM32F446RE using a compatible programmer.

## Usage
- Use a terminal or custom software to send supported command packets to the bootloader via UART.
- Refer to the project documentation for details on packet structure and response formats.

---
## testing
![PXL_20240712_200756368](https://github.com/alfrednagy57/user-attachments/blob/main/IMAGES_/Screenshot%202025-01-03%20211914.png?raw=true)
![PXL_20240712_200756369](https://github.com/alfrednagy57/user-attachments/blob/main/IMAGES_/Screenshot%202025-01-03%20212042.png?raw=true)
![PXL_20240712_200756370](https://github.com/alfrednagy57/user-attachments/blob/main/IMAGES_/Screenshot%202025-01-03%20212125.png?raw=true)

## Future Enhancements
- Add an encryption layer using AES-256 CBC for user authentication.
- Integrate SHA-256 for advanced integrity checks.
- Enhance system architecture with shared flash sectors for optimized functionality.

## License
This project is licensed under the MIT License.

---
## Contact

For any inquiries, please contact the project maintainer at [linkedin](https://www.linkedin.com/in/alfred-nagy-882445224/).
