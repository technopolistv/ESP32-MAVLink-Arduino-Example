#define MAVLINK_BAUD 57600

#define LED_GREEN 16
#define LED_RED 17

#define LOCAL_IP 192, 168, 23, 23
#define SUBNET 255, 255, 255, 255
#define GATEWAY 192, 168, 23, 1
#define LOCAL_PORT 14550

#define GROUNGSTATION_IP "192.168.23.2"
#define GROUNDSTATION_PORT 14550

bool armed = false;

int16_t system_mode = MAV_MODE_PREFLIGHT;
int16_t control_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

unsigned long previousHeartbeatMillis = 0;
unsigned long previousTimeoutMillis = 0;

const int16_t system_id = 1;
const int16_t component_id = 1;

const unsigned long heartbeat_interval = 1000;
const unsigned long timeout_interval = 5000;

const char* groundstation_host = GROUNGSTATION_IP;
const uint16_t groundstation_port = GROUNDSTATION_PORT;
const uint16_t udp_mavlink_port = LOCAL_PORT;

IPAddress ip(LOCAL_IP);
IPAddress subnet(SUBNET);
IPAddress gateway(GATEWAY);

WiFiUDP mavlink_udp;