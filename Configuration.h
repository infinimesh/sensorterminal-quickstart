/*
 * Uncomment these to add features!
 */
 
//#define INFINIMESH            // Connect to Infinimesh?
//#define BME680SENSOR          // Environmental sensor present?

/*
 *  WiFi Credentials for the hackathon. Change for a different network.
 */

const char ssid[] = "IoTHackathon";
const char password[] = "IwantInfinimesh!";

/* 
 *  MQTT settings
 */

const char *ID = "0x59";              // Name of our device, must be unique
const char *TOPIC = "data";           // Topic to subcribe to
const char *server = "192.168.1.154"; // Server address
