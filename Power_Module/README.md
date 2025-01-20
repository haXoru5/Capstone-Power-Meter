# User Manual

## Environment Setup and Building

### Option 1: nRF Connect SDK with Zephyr RTOS
The nRF Connect SDK is used for development. To set up the environment, use the following steps. **Note: environment has been tested in Linux only, but process should be similar for Windows/Mac**
1. Follow the Nordic [tutorial](https://youtu.be/EAJdOqsL9m8?si=1R-WEse3zI5K7DNv) for installation of nRF Connect for VSCode. This should include:
    - [nRF Command Line Tools](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools/Download) **Note: Nordic is in the process of replacing this with nRF Util. This has not been tested for this configuration**
    - VSCode nRF Connect Extension Pack
2. Under the Welcome tab in nRF Connect, click on Create a new board
3. Name it PMS_nRF52840, select nRF52840 QFAA as the SoC, and place it in ~/ncs/~~~/zephyr. Set the company name to CycleSense
4. Navigate to the newly created PMS_nRF52840 folder and replace the contents with the ones in the PMS_nRF52840 folder in this repo
5. To build, click Add build configuration in the Applications tabm
6. Select PMS_nRF52840/nRF52840 and click Build Configuration
7. VSCode will build the .hex file to be used for flashing. It is found under ~/PROJECT_NAME/build/PROJECT_NAME/zephyr/

### Option 2: nRF5 SDK with µVision
nRF5 SDK is being deprecated in favour of nRF Connect SDK but this will be a fallback
1. Install µVision 5
2. Open keil, in the packs manager that opens go to nordic -> nrf52 -> nrF52840 install all 18 device specifc packs
3. Download [nRF5 SDK](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK), version 17.1.0. Only nRF_SDK_17.1.0_ddde560.zip is required
4. Unzip the folder. Its path may not contain spaces
5. Starter projects may be found in: examples -> peripheral -> EXAMPLE_NAME -> pca10056 -> blank -> arm5_no_packs and open EXAMPLE_NAME.uvprojx. Let uVision install all the extra packs that it wants.
6. Click rebuild all
7. the built .bin file may be found in the _build folder located in the arm5_no_packs folder.

##Flashing

Flashing is done using DAPLink, an open source version of J-Link. It uses [pyOCD](https://pyocd.io/) as the software and [nanoDAP](https://github.com/wuxx/nanoDAP-HS/tree/master) as the hardware.
1. Install pyOCD using `pip install pyocd`
2. Install nanoDAP firmware???
3. To erase the chip, run `pyocd erase -t nrf52840 --chip`
4. To flash the chip, run `pyocd flash -t nrf52840 BINARY_FILE.hex`
DAPLink also supports debugging, documentation to be added at a later date
