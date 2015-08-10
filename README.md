# Carpool-gunradio
Carpool PHY layer is built atop open source GNURadio http://gnuradio.org/

## Introduction
This is the source code of Carpool. The source is maintained locally and not published until the related paper is accepted.  

**“Less Transmissions, More Throughput: Bringing Carpool to Public WLANs”, IEEE Transactions on Mobile Computing (TMC 2015)__**

Carpool is implemented both in PHY and MAC layer, which adopt the standard requirement of IEEE 802.11n. The PHY layer is built atop the OFDM implementation of GNURadio/USRP2, equipped with RFX2450. The MAC layer is realized by the trace-driven simulation under MATLAB with specific network setting.

GNU Radio is an open source development toolkit that provides the digital signal processing (DSP) blocks to implement software defined radios.  To ensure the runtime performance, the DSP blocks are implemented using C++. Through exploiting Python to connect different DSP blocks, we can fast emulate and prototype the low-level functionality of wireless protocol families (e.g. Wi-Fi, RFID, Bluetooth and Cellular, etc.).

## Environment and Installation
Our project is built on the Ubuntu 12.04, with GNURadio 3.6 series. Please notes that the latest GNURadio version is 3.7 with major upgrade. In order to ensure the compatibility of our source code, make sure you have installed GnuRadio 3.6 series. 

### 1. To check the version of GNURadio:
### 2. To install Gnuradio:
### 3. To work with Gnuradio:
