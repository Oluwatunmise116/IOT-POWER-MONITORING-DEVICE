// Wi-Fi credentials
const char* ssid = "WIFI_SSID";
const char* password = "WIFI_PASSWORD";

// HiveMQ Cloud credentials
const char* mqtt_server = "53dc450946d248b195af8d1f1b7fc031.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "iotpowermonitor"; // From HiveMQ Cluster > Access Management
const char* mqtt_pass = "Iotpowermonitor123"; // From HiveMQ Cluster > Access Management

// Topics
//const char* publishTopic = "fmgold/esp32/temp";
const char* publishTopic = "iotpower/esp32/energy";
const char* subscribeTopic = "iotpower/esp32/voltage";