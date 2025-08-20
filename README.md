
## Upload and monitor by USB
- press boot button
- then connect USB cable to board
- release boot button
- download Zadig https://zadig.akeo.ie/
- oped Zadig and select "Options->List all devices", select ESP32-S2 (Interface 0),  select "WinUsb" install
- select select ESP32-S2 (Interface 2) "USB serial (CDC)"  driver,  Install
- reboot
- select new COM port for firmware upload in platformio


 to run menu config 
 ```
 pio run --target menuconfig --environment lolin_s2_mini
 ```
