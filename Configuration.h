/*
 * Uncomment these to add features!
 */
 
//#define INFINIMESH            // Connect to Infinimesh?
//#define BME680SENSOR          // Environmental sensor present?

/*
 *  Button labels
 */
const char *button[] = {"undefined", "good", "bad"};

/*
 *  WiFi Credentials
 */

const char ssid[] = "XXXXXXXXXX";
const char psk[] = "XXXXXXXXXX";

/* 
 *  MQTT settings
 */

const char *TOPIC = "devices/0xXX/state/reported/delta";  // Topic to subcribe to
const char *user = "stXX";            // MQTT basic auth
const char *password ="XXXXXXXXXXXXXXXXXXXXXXX";
const char *server = "mqtt.api.infinimesh.dev"; // Server address
