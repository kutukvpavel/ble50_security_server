target remote :3333
symbol-file /home/paul/Documents/esp32_sensors/ble50_security_server/build/ble50_sec_gatts_demo.elf
mon reset halt
flushregs
thb app_main