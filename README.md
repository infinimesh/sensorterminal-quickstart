# Sensor-Terminal QuickStart
Welcome to the Infinite Devices Sensor-Terminal!

First, you need to update the firmware on the SoC:
	git clone https://github.com/Seeed-Studio/ambd_flash_tool
    cd ambd_flash_tool
	python3 ambd_flash_tool.py erase
	python3 ambd_flash_tool.py flash	

Next, install and run [Arduino](https://www.arduino.cc/en/software). In Preferences, add
    https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json
and install "Seeed SAMD Boards".

Open [quickstart.ino] from this repo. Set all parameters in Tools according to this screenshot (serial port might be different, or course) and hit play. Your Sensor-Terminal should show the time after getting it from the Internet. For more verbose debug output, activate the Serial Monitor.

Happy hacking!