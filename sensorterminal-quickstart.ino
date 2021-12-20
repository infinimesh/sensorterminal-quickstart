/*
 *  Infinite Devices Sensor Terminal QuickStart
 *
 *  Please edit Configuration.h to set up for you environment!
 */

#include "Configuration.h"

#include "rpcWiFi.h"          // WiFi library
#include "WiFiUdp.h"          // UDP library for NTP
#include "WiFiClientSecure.h" // TLS library
#include "PubSubClient.h"     // MQTT library   
#include "millisDelay.h"      // Non-blocking delay
#include "RTC_SAMD51.h"       // Real time clock library
#include "TFT_eSPI.h"         // TFT library
#include "Free_Fonts.h"       // Screen fonts
#include "LIS3DHTR.h"         // Accelerometer
#include "Wire.h"
#ifdef BME680SENSOR
  #include "Zanshin_BME680.h"
  BME680_Class BME680;  ///< Create an instance of the BME680 class
#endif

TFT_eSPI tft;

LIS3DHTR<TwoWire> lis;

millisDelay updateDelay;                // The update delay object used for ntp periodic update.
millisDelay loopDelay;                  // This timer is to redraw the screen only once per second.
 
unsigned int localPort = 2390;          // local port to listen for UDP packets
char timeServer[] = "de.pool.ntp.org";  // NTP server
 
const int NTP_PACKET_SIZE = 48;         // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE];     //buffer to hold incoming and outgoing packets
 
// declare a time object
DateTime now;
 
// define WiFI client
WiFiClient client;
PubSubClient mqtt(client);
 
//The udp library class
WiFiUDP udp;
 
// localtime
unsigned long devicetime;
 
RTC_SAMD51 rtc;

int userstate = 0;
 
// for use by the Adafuit RTClib library
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };

void setup() {
  lis.begin(Wire1);
 
  if (!lis) {
    tft.println("ERROR initializing accelerometer!");
    while(1);
  }
  lis.setOutputDataRate(LIS3DHTR_DATARATE_1HZ); //Data output rate
  lis.setFullScaleRange(LIS3DHTR_RANGE_2G); //Scale range set to 2g

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  pinMode(WIO_LIGHT, INPUT);
  pinMode(WIO_MIC, INPUT);
  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);
    tft.printf("RTL8720 Firmware Version: %s\n\n", rpc_system_version());

#ifdef BME680SENSOR
  while (!BME680.begin(I2C_STANDARD_MODE)) {  // Start BME680 using I2C, use first device found
    tft.print(F("-  Unable to find BME680. Trying again in 5 seconds.\n"));
    delay(5000);
  }  // of loop until device is located
  tft.print(F("- Setting 16x oversampling for all sensors\n"));
  BME680.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
  tft.print(F("- Setting IIR filter to a value of 4 samples\n"));
  BME680.setIIRFilter(IIR4);  // Use enumerated type values
  tft.print(F("- Setting gas measurement to 320\xC2\xB0\x43 for 150ms\n"));  // "�C" symbols
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds
#endif

    // setup network before rtc check 
    connectToWiFi(ssid, password);
 
    // get the time via NTP (udp) call to time server
    // getNTPtime returns epoch UTC time adjusted for timezone but not daylight savings
    // time
    devicetime = getNTPtime();
 
    // check if rtc present
    if (devicetime == 0) {
        tft.println("Failed to get time from network time server.");
    }
 
    if (!rtc.begin()) {
        tft.println("Couldn't find RTC");
        while (1) delay(10); // stop operating
    }
 
    // get and print the current rtc time
    now = rtc.now();
    tft.print("RTC time is: ");
    tft.println(now.timestamp(DateTime::TIMESTAMP_FULL));
 
    // adjust time using ntp time
    rtc.adjust(DateTime(devicetime));
 
    // print boot update details
    tft.println("RTC (boot) time updated.");
    // get and print the adjusted rtc time
    now = rtc.now();
    tft.print("Adjusted RTC (boot) time is: ");
    tft.println(now.timestamp(DateTime::TIMESTAMP_FULL));


#ifdef INFINIMESH
  mqtt.setServer(server, 1883);
  mqtt.connect(ID);
  tft.println("Connected to MQTT broker.");
#endif 

    // start millisdelays timers as required, adjust to suit requirements
    updateDelay.start(60 * 60 * 1000); // update time via ntp every hr
    loopDelay.start(1000); // Draw the display every second
}
 
void loop() {
  float x_values, y_values, z_values;
  static int32_t  temp, humidity, pressure, gas;  // BME readings

    if (updateDelay.justFinished()) {
        updateDelay.repeat();
 
        // update rtc time
        devicetime = getNTPtime();
        if (devicetime == 0) {
            tft.println("Failed to get time from network time server.");
        }
        else {
            rtc.adjust(DateTime(devicetime));
            tft.println("");
            tft.println("rtc time updated.");
            // get and print the adjusted rtc time
            now = rtc.now();
            tft.print("Adjusted RTC time is: ");
            tft.println(now.timestamp(DateTime::TIMESTAMP_FULL));
        }
    }
    
    if (loopDelay.justFinished()) {
      loopDelay.restart();
      if (digitalRead(WIO_KEY_C) == LOW) {
        userstate = 0;
      }
      if (digitalRead(WIO_KEY_B) == LOW) {
        userstate = 1;
      }
      if (digitalRead(WIO_KEY_A) == LOW) {
        userstate = 2;
      }
      tft.fillScreen(TFT_BLACK);
      tft.setFreeFont(FM12);
      now = rtc.now();
      int light = 0;
      int noise = 0;
      for (int i = 0; i < 10; i++) {
        light += analogRead(WIO_LIGHT);
        noise += abs((signed int)analogRead(WIO_MIC)-430); // About 430 is the DC bias of the mic in the Terminal I tested on
      }
      light /= 10;
      noise /= 10;
      for (int i = 0; i < 3; i++) {
        if (userstate == i) {
          tft.drawString("x", i*80, 0);
        } else {
          tft.drawString("o", i*80, 0);
        }
      }
      tft.drawString(button[userstate], 180, 0);
      tft.drawString(now.timestamp(DateTime::TIMESTAMP_DATE), 0, 20);
      tft.drawString(now.timestamp(DateTime::TIMESTAMP_TIME), 180, 20);
      tft.drawString("Lght:" + String(light), 0, 60);
      tft.drawString("Nois:" + String(noise), 160, 60);
#ifdef BME680SENSOR
      BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
      tft.drawString("Temp:" + String(temp/100), 0, 113);
      tft.drawString("Humi:" + String(humidity/1000), 0, 150);
      tft.drawString("Pres:" + String(pressure), 0, 187);
#endif
      x_values = lis.getAccelerationX();
      tft.drawString("x:" + String(x_values), 0, 80);
      y_values = lis.getAccelerationY();
      tft.drawString("y:" + String(y_values), 106, 80);
      z_values = lis.getAccelerationZ();
      tft.drawString("z:" + String(z_values), 212, 80);
      String data="{\"light\": " + String(light) + ", \"noise\": " + String(noise) + ", \"accx\": "+String(x_values)+", \"accy\": "+String(y_values)+", \"accz\": "+String(z_values)+", \"userstate\": \"" + button[userstate] + "\"}";

#ifdef INFINIMESH
      mqtt.publish(TOPIC, data.c_str());
#endif
    }
}
 
 
void connectToWiFi(const char* ssid, const char* pwd) {
    tft.println("Connecting to WiFi network: " + String(ssid));
 
    // delete old config
    WiFi.disconnect(true);
 
    tft.println("Waiting for WIFI connection...");
 
    //Initiate connection
    WiFi.begin(ssid, pwd);
 
    while (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(ssid, pwd);
        delay(500);
    }
    tft.println("Connected.");
    printWifiStatus();
 
}
 
unsigned long getNTPtime() {
 
    // module returns a unsigned long time valus as secs since Jan 1, 1970 
    // unix time or 0 if a problem encounted
 
    //only send data when connected
    if (WiFi.status() == WL_CONNECTED) {
        //initializes the UDP state
        //This initializes the transfer buffer
        udp.begin(WiFi.localIP(), localPort);
 
        sendNTPpacket(timeServer); // send an NTP packet to a time server
        // wait to see if a reply is available
        delay(1000);
 
        if (udp.parsePacket()) {
            tft.println("udp packet received");
            tft.println("");
            // We've received a packet, read the data from it
            udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
 
            //the timestamp starts at byte 40 of the received packet and is four bytes,
            // or two words, long. First, extract the two words:
 
            unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
            unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
            // combine the four bytes (two words) into a long integer
            // this is NTP time (seconds since Jan 1 1900):
            unsigned long secsSince1900 = highWord << 16 | lowWord;
            // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
            const unsigned long seventyYears = 2208988800UL;
            // subtract seventy years:
            unsigned long epoch = secsSince1900 - seventyYears;
 
            // adjust time for timezone offset in secs +/- from UTC
            // WA time offset from UTC is +8 hours (28,800 secs)
            // + East of GMT
            // - West of GMT
            long tzOffset = 3600L;
 
            // WA local time 
            unsigned long adjustedTime;
            return adjustedTime = epoch + tzOffset;
        }
        else {
            // were not able to parse the udp packet successfully
            // clear down the udp connection
            udp.stop();
            return 0; // zero indicates a failure
        }
        // not calling ntp time frequently, stop releases resources
        udp.stop();
    }
    else {
        // network not connected
        return 0;
    }
 
}
 
// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(const char* address) {
    // set all bytes in the buffer to 0
    for (int i = 0; i < NTP_PACKET_SIZE; ++i) {
        packetBuffer[i] = 0;
    }
    // Initialize values needed to form NTP request
    // (see URL above for details on the packets)
    packetBuffer[0] = 0b11100011;   // LI, Version, Mode
    packetBuffer[1] = 0;     // Stratum, or type of clock
    packetBuffer[2] = 6;     // Polling Interval
    packetBuffer[3] = 0xEC;  // Peer Clock Precision
    // 8 bytes of zero for Root Delay & Root Dispersion
    packetBuffer[12] = 49;
    packetBuffer[13] = 0x4E;
    packetBuffer[14] = 49;
    packetBuffer[15] = 52;
 
    // all NTP fields have been given values, now
    // you can send a packet requesting a timestamp:
    udp.beginPacket(address, 123); //NTP requests are to port 123
    udp.write(packetBuffer, NTP_PACKET_SIZE);
    udp.endPacket();
}
 
void printWifiStatus() {
    // print the SSID of the network you're attached to:
    tft.println("");
    tft.print("SSID: ");
    tft.println(WiFi.SSID());
 
    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    tft.print("IP Address: ");
    tft.println(ip);
 
    // print the received signal strength:
    long rssi = WiFi.RSSI();
    tft.print("signal strength (RSSI):");
    tft.print(rssi);
    tft.println(" dBm");
    tft.println("");
}
