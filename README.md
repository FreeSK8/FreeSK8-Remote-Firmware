# FreeSK8-Remote-Firmware

To compile follow steps 1-4 of the [ESP IDF Setup Instructions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#setting-up-development-environment) to setup your development environment. 

With ESP IDF installed execute: 
```
. $HOME/esp/esp-idf/export.sh
```
then
```
idf.py -p /dev/ttyUSB0 build flash monitor
```
