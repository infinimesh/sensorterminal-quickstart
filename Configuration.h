/*
 * Uncomment these to add features!
 */
 
//#define INFINIMESH            // Connect to Infinimesh?
//#define BME680SENSOR          // Environmental sensor present?

/*
 *  WiFi Credentials.
 */

const char ssid[] = "";
const char password[] = "";

/* 
 *  MQTT settings
 */

const char *ID = "0x59";              // Name of our device, must be unique
const char *TOPIC = "data";           // Topic to subcribe to
const char *server = "192.168.1.154"; // Server address
