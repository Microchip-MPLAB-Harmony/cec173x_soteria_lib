![Microchip logo](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_logo.png)
![Harmony logo small](https://raw.githubusercontent.com/wiki/Microchip-MPLAB-Harmony/Microchip-MPLAB-Harmony.github.io/images/microchip_mplab_harmony_logo_small.png)

# Microchip MPLAB® Harmony 3 Release Notes

## CEC173x_soteria_lib Release v3.2.0

### New Features

  - Internal Flash as local memory for Uboot
  - PLDM AP FW update

### Bug fixes

  - EC KHB status read through I2C
  - AP firmware restore fail via AP1 I2C port + Violation in AP1
  - Byte match violation and I2C match log register
  - In violation reauth disabled, the AP reauth active flag is not cleared after violation handling
  - I2C not responding to WR + WR + RD
  - Write INT SPI IOC bit when QMSPI is configured
  - Fix leading Zeroes in AK Certificate
  - Fix measurements not returning error response after Challenge failure
  - I2C command done gets executed before mcopy completion

### Improvements

  - Handle Invalid certificate in spdm
  - Handling multiple I2C commands from multiple hosts
  - Hash table update - support in I2C or PLDM
  - Upgraded compiler version from 4.10 to 4.20
  - Core Data ISO improvements for waiting of events until its done
  - Misra Mandatory issues fixed
  - SMBUS memory cleanup w.r.t code space
  - Add Trace levels for each module
  - Auto roll back protection check for 6HT
  - SPI image generator update for supporting UBOOT image from internal flash
  - Skip restore if active image fails in golden recovery

### Known Issues

  - None

### Development Tools

For CEC173x family of devices:

  - [MPLAB® X IDE v6.05](https://www.microchip.com/mplab/mplab-x-ide)
  - [MPLAB® XC32 C/C++ Compiler v4.21](https://www.microchip.com/mplab/compilers)

### Notes

-  None

## CEC173x_soteria_lib Release v3.1.0

### New Features

  - Moved from CEC1736_S0_SX to CEC1736_S0_2ZW glacier device part id in MPLABX IDE
  - Add 256 bytes of Opaq data in SPDM Measurements response
  - Add Digest for Multiple Certificate Chain
  - Implemented AM bit to restore flash 3B/4B mode    

### Bug fixes

  - GET_CERT spdm command fix
  - Fix CertChainHash in CHALLENGE_AUTH in SPDM
  - Fix hash computation in crypto_data_iso.c
  - Fix Signature in Challenge and Measurements response
  - Fix Challenge response 
      - if invalid slot number is given
      - if TCB Hash is selected
      - when TCB Hash or No Measurement summary hash is selected
      - to get total number of measurement blocks available
  - Fix Measurements response to get measurement block at particular index
  - Fix Number of blocks, MeasurementRecordLength and MeasurementSize
  - Fixed coverity L1 issues
  - Fix Uboot main timeout triggred even after uboot main read is complete
  - Fix AP getting reset when validating AP FW images through i2c command
  - Golden Recovery fix for fallback images having incorrect restore address
  - Fix for Reauth not working when LTMON and VIOL ISR occurs
  - Fix for AP1 reauth fail when AP0 is reading runtime image
  - I2C Pre/Postboot Permission Update Fix for AP1
  - Fixed I2C read for 0x24 command
  - Fix Serial Number I2c read
  - Fix I2C command not getting executed after sending SPDM request with Invalid Certificate
  - Fix LED Control status

### Improvements

  - Handle NEG_ALGO spdm request if length spans more than MCTP_PACKET_MAX
  - Handle Invalid SPDM Requests
  - Added support for glacier option 13 
  - Reset AP after N retries of runtime images, if enabled in APCFG
  - I2C Synchronization b/w SPDM MCTP
  - Check MTMON first fetch for AP1
  - Remove run-time permission when validating AP FW images through i2c command
  - Capture the mismatch address during the Data Mismatch Violation
  - Process transfer, verify, apply response from UA
  - SMBUS, MCTP, SPDM task synchronization using event bits
  - Increase OEM_task1 and OEM_task2 stack from 128 to 256 bytes 
  - Clear I2C-SMBUS Response Queue before sending I2c read request from SMBUS       
  - In I2C Command 95, for validate all AP1,authenticate AP1 HT along with images
  - Byte latching for SPIMON Logs through I2C command
  - Flash iso release i2c command handling
  - Update HT and AP image auth status of AP FW Validate I2C Command in status register 0x96
  - Masked copy status update when copy failed due to masked regions

### Known Issues

  - None

### Development Tools

For CEC173x family of devices:

  - [MPLAB® X IDE v6.00](https://www.microchip.com/mplab/mplab-x-ide)
  - [MPLAB® XC32 C/C++ Compiler v4.10](https://www.microchip.com/mplab/compilers)

### Notes

  -  None

## CEC173x_soteria_lib Release v3.0.1

### New Features

  - Updated user guide

### Bug fixes

  - None

### Improvements

  - None

### Development Tools

For CEC173x family of devices:

  - [MPLAB® X IDE v6.00](https://www.microchip.com/mplab/mplab-x-ide)
  - [MPLAB® XC32 C/C++ Compiler v4.00](https://www.microchip.com/mplab/compilers)

### Notes

  -  None

## CEC173x_soteria_lib Release v3.0.0

### New Features

  - Added HRM active/inactive logic from glacier option 5
  - Implemented Minimum delay between AP0 and AP1 reset releas - 100ms                              
  - I2C register for AP KHB authentication status update
  - AP1 LED control command initiate
                 
### Bug fixes

  - Fix for reauthentication required in violations only and not required for First Fetch
  - Corrected GPIO configuration for BMC, PCH access after flash operations and Reset Release
  - Fix added to for APCFG table status command
  - Fixed pass component response (13H)
  - PLDM request firmware data - SOM, EOM, TO correction for request /response messages
  - Memfault issue when spdm certificate not present is fixed
  - Issue related to multiple Challenge authuentication requests fixed
  - Fixed Endianness in multi bytes SPDM responses


### Improvements

  - In the case of not asserting AP reset pin, Reauthentication required when using Flash for I2C commands
  - Runtime region protection - Set permissions to required bits in spimon register
  - Ensure Region start and end address configured in 4k units
  - SPI MON configurations can be enabled from AP_CFG - address wrap detection enable
  - PLDM Request Firmware data & Get Firmware parameters (02H) - multiple pkt response is handled as per spec 
  - Key revocation and Rollback protection Check in PLDM for EC Firmware update
  - Portion Length in GET_CERT SPDM Command
  - Added 256 bytes of OpaqData in Challenge SPDM response
  - For multiple response, msg type is not required in packets other than first packet
  - MCTP level packet validation for out of sequence
  - MCTP layer support for receiving/transmitting multiple pkts
  - MCTP unset packetizing when multiple packets are dropped
  - Strong smash protection feature enabled for MPU aware codeline
  - Trace levels - converted to 3 levels, to get more space
  - Code cleanup related to GPIO initializations of unused pins
  - Coverity fixes for compliance of Coverity Secure Reports

### Development Tools

For CEC173x family of devices:
  - [MPLAB® X IDE v6.00](https://www.microchip.com/mplab/mplab-x-ide)
  - [MPLAB® XC32 C/C++ Compiler v4.00](https://www.microchip.com/mplab/compilers)

### Notes

  -  None
