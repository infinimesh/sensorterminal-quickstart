# Sensor-Terminal QuickStart
Welcome to the Infinite Devices Sensor-Terminal!

First, you need to update the firmware on the SoC:

	git clone https://github.com/Seeed-Studio/ambd_flash_tool
    cd ambd_flash_tool
	python3 ambd_flash_tool.py erase
	python3 ambd_flash_tool.py flash	

Next, install and run [Arduino](https://www.arduino.cc/en/software). In Preferences, add

    https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json

and install "Seeed SAMD Boards". Copy the content of [library-package](library-package) in the libraries folder of your Arduino folder (in Documents or "My Documents").

Open [sensorterminal-quickstart.ino](sensorterminal-quickstart.ino) from this repo. Set all parameters in Tools according to this screenshot (serial port might be different, or course) and hit play.

![](arduinosettings.png)

Your Sensor-Terminal should show the time after getting it from the Internet. For more verbose debug output, activate the Serial Monitor.

## Infinimesh

Together with your Sensor-Terminal, you have received a login for the Infinimesh platform. First, create a new namespace, as you cannot create devices in your root namespace. Then, in the new namespace, create a device, give it a name and set it to enabled. At the moment, you have to provide a public cert even though we're not going to use it. See "old way" below on how to create it.

Click on the device and on "MQTT Basic Auth" under "Actions". In the popup, click "Enable". Copy the password.

In Arduino, in the tab Configuration.h, uncomment the line

    #define INFINIMESH

and enter your Wifi credentials, enter the device's ID into the TOPIC, change user to your device's name and password to the password you just copied. Hit play. The data from the Sensor-Terminal should appear in Infinimesh.

## Old way of connecting to Infinimesh

This is the way of connecting to Infinimesh over SSL authentication. As the Sensor-Terminal cannot do that yet, you need to use a proxy like [Node-RED](https://nodered.org/#get-started) to receive the MQTT messages from the Sensor-Terminal and send them on to Infinimesh.

Together with your Sensor-Terminal, you have received a login for the Infinimesh platform. To create your device in Infinimesh, you first need to create a device certificate. Run

    bash create-certs.sh device

to generate device.crt and device.key. Login to [Infinimesh](https://console.infinimesh.app/), click on the + button, give the device a name (e.g. "Terminal_40"), select your namespace, set the device to enabled and upload the device.crt file. (The device.key file is the private key, don't give it away!) Click submit and the device is created and given an id (hex number like 0x59). 

Here's a working setup for Node-RED, change the device ID to yours:

![](nodered.png)

Change the MQTT settings in your sketch to your Node-RED, uncomment the line

    #define INFINIMESH

and hit play, you should be sending your light value into Infinimesh!

## Connecting additional sensors using I2C

The example contains code for a BME680 environmental sensor connected to the left Grove port of the unit. Uncomment

    #define BME680SENSOR

to read out the sensor, display its values on the screen and send them to the MQTT server.

Happy hacking!

![](pinout.jpeg)
![](pinout2.jpeg)