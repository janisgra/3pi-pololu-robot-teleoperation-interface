/**
 * @file esp32c3_bridge.cpp
 * @brief ESP32-C3 SuperMini WiFi Bridge for Pololu 3pi+ 32U4 Robot
 * @author Janis
 * @version 1.0.0
 * 
 * Bridges UDP communication between a control server and the 3pi+ robot.
 * Receives JSON commands via WiFi UDP and forwards them to the robot via UART.
 * Robot responses are forwarded back to the UDP client.
 * 
 * Board: ESP32-C3 SuperMini (HW-466AB)
 * 
 * Wiring:
 *   ESP32-C3 GPIO5 (RX) <- 3pi+ TX1 (Pin 0)
 *   ESP32-C3 GPIO4 (TX) -> 3pi+ RX1 (Pin 1)
 *   ESP32-C3 GND        -- 3pi+ GND
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// ============================================================================
// DEFINITIONS
// ============================================================================

// WiFi Configuration
#define WIFI_SSID               "GL-NET"
#define WIFI_PASSWORD           "goodlife"

// UDP Configuration
#define UDP_PORT                5006
#define DISCOVERY_PORT          5004    // Broadcast beacon port for auto-discovery
#define DISCOVERY_INTERVAL_MS   2000    // Beacon interval when no client connected
#define DISCOVERY_SLOW_MS       10000   // Beacon interval when client is active

// Pin Configuration (ESP32-C3 SuperMini)
#define ROBOT_RX_PIN            5       // GPIO5 <- 3pi+ Pin 0 (TX1)
#define ROBOT_TX_PIN            4       // GPIO4 -> 3pi+ Pin 1 (RX1)
#define LED_PIN                 8       // Onboard LED (active LOW on SuperMini)

// Serial Configuration
#define ROBOT_BAUD              115200

// Timing
#define BRIDGE_HEARTBEAT_MS     5000
#define STATUS_PRINT_MS         10000
#define WIFI_RECONNECT_MS       10000

// Buffer Sizes
#define UDP_BUFFER_SIZE         512
#define SERIAL_BUFFER_SIZE      512

// ============================================================================
// GLOBAL STATE
// ============================================================================

WiFiUDP udp;
WiFiUDP discoveryUdp;                   // Separate socket for discovery beacons
IPAddress lastClientIP;
uint16_t lastClientPort = 0;

char udpBuffer[UDP_BUFFER_SIZE];
String serialBuffer;

uint32_t lastHeartbeat = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastWiFiCheck = 0;
uint32_t lastDiscoveryBeacon = 0;
uint32_t rxCount = 0;
uint32_t txCount = 0;
uint32_t lastSerialCharTime = 0;   // for UART buffer-timeout (stale data flush)

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void setLED(bool on) {
    digitalWrite(LED_PIN, on ? LOW : HIGH);  // Active LOW on SuperMini
}

void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        setLED(true);
        delay(delayMs);
        setLED(false);
        delay(delayMs);
    }
}

void sendHeartbeat() {
    if (lastClientPort == 0) return;
    
    JsonDocument doc;
    doc["type"] = "bridge_status";
    doc["board"] = "ESP32-C3";
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["rx"] = rxCount;
    doc["tx"] = txCount;
    doc["uptime"] = millis() / 1000;
    doc["heap"] = ESP.getFreeHeap();
    doc["temp"] = temperatureRead();
    
    String output;
    serializeJson(doc, output);
    
    udp.beginPacket(lastClientIP, lastClientPort);
    udp.print(output);
    udp.endPacket();
}

// ============================================================================
// DISCOVERY PROTOCOL
// ============================================================================

/**
 * Broadcast a discovery beacon on the subnet broadcast address.
 * Any client listening on DISCOVERY_PORT can find this bridge
 * without knowing its IP in advance.
 */
void broadcastDiscovery() {
    if (WiFi.status() != WL_CONNECTED) return;

    JsonDocument doc;
    doc["type"]    = "discovery";
    doc["service"] = "pololu-3pi-bridge";
    doc["ip"]      = WiFi.localIP().toString();
    doc["port"]    = UDP_PORT;
    doc["mac"]     = WiFi.macAddress();
    doc["board"]   = "ESP32-C3";
    doc["rssi"]    = WiFi.RSSI();
    doc["uptime"]  = millis() / 1000;

    String output;
    serializeJson(doc, output);

    // Compute proper broadcast address from IP and subnet mask
    IPAddress localIP = WiFi.localIP();
    IPAddress subnetMask = WiFi.subnetMask();
    IPAddress bcast;
    for (int i = 0; i < 4; i++) {
        bcast[i] = localIP[i] | ~subnetMask[i];
    }

    discoveryUdp.beginPacket(bcast, DISCOVERY_PORT);
    discoveryUdp.print(output);
    discoveryUdp.endPacket();

    Serial.printf("[Discovery] Beacon -> %s:%d (%d bytes)\n",
                  bcast.toString().c_str(), DISCOVERY_PORT, output.length());
}

/**
 * Handle incoming packets on the discovery port.
 * If a client sends {"cmd":"discover_ack"} we log the client and reply
 * with {"type":"discover_confirm",...} so the handshake completes.
 */
void handleDiscoveryResponse() {
    int packetSize = discoveryUdp.parsePacket();
    if (packetSize <= 0) return;

    char buf[256];
    int len = discoveryUdp.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = '\0';

    IPAddress remoteIP   = discoveryUdp.remoteIP();
    uint16_t  remotePort = discoveryUdp.remotePort();

    Serial.printf("[Discovery] RX from %s:%d -> %s\n",
                  remoteIP.toString().c_str(), remotePort, buf);

    if (strstr(buf, "\"cmd\":\"discover_ack\"") != nullptr) {
        lastClientIP = remoteIP;
        // lastClientPort is set when the client actually sends on the data port

        // Confirm discovery to the client
        JsonDocument ack;
        ack["type"]  = "discover_confirm";
        ack["ip"]    = WiFi.localIP().toString();
        ack["port"]  = UDP_PORT;
        ack["mac"]   = WiFi.macAddress();
        ack["board"] = "ESP32-C3";

        String ackOut;
        serializeJson(ack, ackOut);

        discoveryUdp.beginPacket(remoteIP, remotePort);
        discoveryUdp.print(ackOut);
        discoveryUdp.endPacket();

        Serial.printf("[Discovery] Client confirmed: %s\n",
                      remoteIP.toString().c_str());
    }
}

// ============================================================================
// UDP HANDLER
// ============================================================================

void handleUDP() {
    int packetSize = udp.parsePacket();
    if (packetSize <= 0) return;
    
    lastClientIP = udp.remoteIP();
    lastClientPort = udp.remotePort();
    
    int len = udp.read(udpBuffer, UDP_BUFFER_SIZE - 1);
    if (len <= 0) return;
    
    udpBuffer[len] = '\0';
    rxCount++;
    
    Serial.printf("[UDP RX] %s:%d -> %s\n", lastClientIP.toString().c_str(), lastClientPort, udpBuffer);

    // Check if this is a bridge-level ping (does not forward to robot).
    // Python json.dumps adds a space after ':' by default, so we check
    // for both "cmd":"bridge_ping" and "cmd": "bridge_ping".
    if (strstr(udpBuffer, "\"bridge_ping\"") != nullptr &&
        strstr(udpBuffer, "\"cmd\"") != nullptr) {
        JsonDocument pingDoc;
        DeserializationError err = deserializeJson(pingDoc, udpBuffer);
        uint32_t seq = 0;
        uint32_t clientTs = 0;
        if (!err) {
            seq = pingDoc["seq"] | 0;
            clientTs = pingDoc["ts"] | 0;
        }
        JsonDocument pongDoc;
        pongDoc["type"] = "bridge_pong";
        pongDoc["seq"] = seq;
        pongDoc["client_ts"] = clientTs;
        pongDoc["bridge_ts"] = millis();
        String pong;
        serializeJson(pongDoc, pong);
        udp.beginPacket(lastClientIP, lastClientPort);
        udp.print(pong);
        udp.endPacket();
        return;
    }

    Serial1.println(udpBuffer);
    
    setLED(false);
    delay(5);
    setLED(true);
}

// ============================================================================
// SERIAL HANDLER
// ============================================================================

void handleSerial() {
    uint32_t now = millis();

    // Flush stale partial data.  If characters arrived but no newline
    // followed within 200 ms, the line is almost certainly a corrupt
    // fragment (e.g. from a loose UART wire).  Discard it so the next
    // valid message is not concatenated to garbage.
    if (serialBuffer.length() > 0 && lastSerialCharTime > 0 &&
        (now - lastSerialCharTime > 200)) {
        Serial.printf("[UART] Flushing stale buffer (%d B): %s\n",
                      serialBuffer.length(), serialBuffer.c_str());
        serialBuffer = "";
    }

    while (Serial1.available()) {
        char c = Serial1.read();
        lastSerialCharTime = now;
        
        if (c == '\n') {
            if (serialBuffer.length() > 0) {
                Serial.printf("[Robot TX] %s\n", serialBuffer.c_str());
                
                if (lastClientPort > 0) {
                    udp.beginPacket(lastClientIP, lastClientPort);
                    udp.print(serialBuffer);
                    udp.endPacket();
                    txCount++;
                }
                serialBuffer = "";
            }
        } else if (c != '\r') {
            serialBuffer += c;
        }
    }
}

// ============================================================================
// WIFI CONNECTION
// ============================================================================

bool connectWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        setLED(attempts % 2 == 0);
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.printf("IP: %s, RSSI: %d dBm, MAC: %s\n", 
            WiFi.localIP().toString().c_str(), WiFi.RSSI(), WiFi.macAddress().c_str());
        return true;
    }
    
    Serial.println("\nWiFi connection failed!");
    return false;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);  // C3 needs extra time for USB CDC
    
    Serial.println();
    Serial.println("==========================================");
    Serial.println("  ESP32-C3 WiFi Bridge for Pololu 3pi+");
    Serial.println("==========================================");
    
    pinMode(LED_PIN, OUTPUT);
    setLED(false);
    
    Serial1.begin(ROBOT_BAUD, SERIAL_8N1, ROBOT_RX_PIN, ROBOT_TX_PIN);
    Serial.printf("Robot UART: RX=%d, TX=%d @ %lu baud\n", ROBOT_RX_PIN, ROBOT_TX_PIN, ROBOT_BAUD);
    
    serialBuffer.reserve(SERIAL_BUFFER_SIZE);
    
    connectWiFi();
    
    udp.begin(UDP_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_PORT);

    discoveryUdp.begin(DISCOVERY_PORT);
    Serial.printf("Discovery beacon on port %d\n", DISCOVERY_PORT);
    
    blinkLED(3, 100);
    setLED(true);
    
    Serial.println("Bridge ready");
    Serial.println("------------------------------------------");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    uint32_t now = millis();
    
    handleUDP();
    handleSerial();

    // Discovery beacon -- broadcasts faster when no client, slower when active
    uint32_t discoveryInterval = (lastClientPort > 0)
        ? DISCOVERY_SLOW_MS : DISCOVERY_INTERVAL_MS;
    if (now - lastDiscoveryBeacon >= discoveryInterval) {
        broadcastDiscovery();
        lastDiscoveryBeacon = now;
    }
    handleDiscoveryResponse();

    // Heartbeat
    if (lastClientPort > 0 && now - lastHeartbeat >= BRIDGE_HEARTBEAT_MS) {
        sendHeartbeat();
        lastHeartbeat = now;
    }
    
    // Status print
    if (now - lastStatusPrint >= STATUS_PRINT_MS) {
        Serial.printf("[Status] WiFi:%s RSSI:%d IP:%s RX:%lu TX:%lu Heap:%lu Temp:%.1fC\n",
            WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED",
            WiFi.RSSI(), WiFi.localIP().toString().c_str(), rxCount, txCount, 
            ESP.getFreeHeap(), temperatureRead());
        lastStatusPrint = now;
    }
    
    // WiFi reconnect
    if (now - lastWiFiCheck >= WIFI_RECONNECT_MS) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            setLED(false);
            WiFi.reconnect();
            delay(5000);
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Reconnected!");
                setLED(true);
            }
        }
        lastWiFiCheck = now;
    }
}
