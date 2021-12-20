/*
 * Uncomment these to add features!
 */
 
#define INFINIMESH            // Connect to Infinimesh?
//#define BME680SENSOR          // Environmental sensor present?

/*
 *  Button labels
 */
const char *button[] = {"undefined", "good", "bad"};

/*
 *  WiFi Credentials
 */

const char ssid[] = "";
const char password[] = "";

/* 
 *  MQTT settings
 */

const char *ID = "st35";              // Name of our device, must be unique
const char *TOPIC = "data";           // Topic to subcribe to
const char *server = "178.254.29.197"; // Server address
