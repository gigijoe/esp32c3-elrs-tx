# ESP32-C3 ELRS External TX Module Control

This project is intent to make use of ELRS TX Module to do real time task, such as remote button trigger.

Button ------ [D5] [ESP32-C3] [UART] ------ [SIG][ELRS TX][RF] ...... [RF][ELRS RX][PWM] ------ [PWM RELAY] - ON/OFF  

# Feature

* Read device information
* Read parameters
* Broadcast ping
* Set packet rate
* Set Tx power
* Set model ID
* Set model match
* Send channels data
                                        
# Dependence

https://github.com/AlfredoSystems/AlfredoCRSF/tree/main

https://michiel.vanderwulp.be/esp32-c3-supermini-blink-platformio.html

https://michiel.vanderwulp.be/platformio-core-cli-installation-linux.html

## PlatformIO

* alias get_pio='source ~/.platformio/penv/bin/activate'
* get_pio
* pio run
* pio run -t upload

## Reference

https://github.com/AlfredoSystems/AlfredoCRSF/tree/main

https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/CrsfProtocol/crsf_protocol.h

https://github.com/kkbin505/Arduino-Transmitter-for-ELRS/blob/main/SimpleTX/crsf.cpp

https://github.com/danxdz/simpleTx_esp32/blob/master/lib/crsf/crsf.cpp

https://gist.github.com/GOROman/9c7eadf78eb522bbb801beb9162a8db5

https://gist.github.com/GOROman/6401fbdafcd6048edb9c32b190d708b9

https://github.com/tbs-fpv/tbs-crsf-spec/blob/main/crsf.md

## Half duplex uart

https://github.com/ExpressLRS/ExpressLRS/blob/master/src/lib/Handset/CRSFHandset.cpp
