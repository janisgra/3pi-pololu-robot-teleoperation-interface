/**
 * @file esp32_bridge.cpp
 * @brief ESP32-S3 WiFi Bridge for Pololu 3pi+ 32U4 Robot
 * @author Janis
 * @version 1.0.0
 * 
 * Bridges UDP communication between a control server and the 3pi+ robot.
 * Receives JSON commands via WiFi UDP and forwards them to the robot via UART.
 * Robot responses are forwarded back to the UDP client.
 * 
 * Board: ESP32-S3-WROOM-1
 * 
 * Wiring:
 *   ESP32 GPIO18 (RX) <- 3pi+ TX1 (Pin 0)
 *   ESP32 GPIO17 (TX) -> 3pi+ RX1 (Pin 1)
 *   ESP32 GND         -- 3pi+ GND
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
#define UDP_PORT                5005

// Pin Configuration (ESP32-S3-WROOM-1)
#define ROBOT_RX_PIN            18      // GPIO18 <- 3pi+ Pin 0 (TX1)
#define ROBOT_TX_PIN            17      // GPIO17 -> 3pi+ Pin 1 (RX1)
#define LED_PIN                 2       // Onboard LED (active HIGH)

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
IPAddress lastClientIP;
uint16_t lastClientPort = 0;

char udpBuffer[UDP_BUFFER_SIZE];
String serialBuffer;

uint32_t lastHeartbeat = 0;
uint32_t lastStatusPrint = 0;
uint32_t lastWiFiCheck = 0;
uint32_t rxCount = 0;
uint32_t txCount = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void blinkLED(int count, int delayMs) {
    for (int i = 0; i < count; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(delayMs);
        digitalWrite(LED_PIN, LOW);
        delay(delayMs);
    }
}

void sendHeartbeat() {
    if (lastClientPort == 0) return;
    
    JsonDocument doc;
    doc["type"] = "bridge_status";
    doc["board"] = "ESP32-S3";
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    doc["rx"] = rxCount;
    doc["tx"] = txCount;
    doc["uptime"] = millis() / 1000;
    doc["heap"] = ESP.getFreeHeap();
    
    String output;
    serializeJson(doc, output);
    
    udp.beginPacket(lastClientIP, lastClientPort);
    udp.print(output);
    udp.endPacket();
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
    Serial1.println(udpBuffer);
    
    digitalWrite(LED_PIN, LOW);
    delay(5);
    digitalWrite(LED_PIN, HIGH);
}

// ============================================================================
// SERIAL HANDLER
// ============================================================================

void handleSerial() {
    while (Serial1.available()) {
        char c = Serial1.read();
        
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
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.printf("IP: %s, RSSI: %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
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
    delay(500);
    
    Serial.println();
    Serial.println("==========================================");
    Serial.println("  ESP32-S3 WiFi Bridge for Pololu 3pi+");
    Serial.println("==========================================");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial1.begin(ROBOT_BAUD, SERIAL_8N1, ROBOT_RX_PIN, ROBOT_TX_PIN);
    Serial.printf("Robot UART: RX=%d, TX=%d @ %lu baud\n", ROBOT_RX_PIN, ROBOT_TX_PIN, ROBOT_BAUD);
    
    serialBuffer.reserve(SERIAL_BUFFER_SIZE);
    
    connectWiFi();
    
    udp.begin(UDP_PORT);
    Serial.printf("UDP listening on port %d\n", UDP_PORT);
    
    blinkLED(3, 100);
    digitalWrite(LED_PIN, HIGH);
    
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
    
    // Heartbeat
    if (lastClientPort > 0 && now - lastHeartbeat >= BRIDGE_HEARTBEAT_MS) {
        sendHeartbeat();
        lastHeartbeat = now;
    }
    
    // Status print
    if (now - lastStatusPrint >= STATUS_PRINT_MS) {
        Serial.printf("[Status] WiFi:%s RSSI:%d IP:%s RX:%lu TX:%lu Heap:%lu\n",
            WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED",
            WiFi.RSSI(), WiFi.localIP().toString().c_str(), rxCount, txCount, ESP.getFreeHeap());
        lastStatusPrint = now;
    }
    
    // WiFi reconnect
    if (now - lastWiFiCheck >= WIFI_RECONNECT_MS) {
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi lost, reconnecting...");
            digitalWrite(LED_PIN, LOW);
            WiFi.reconnect();
            delay(5000);
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println("Reconnected!");
                digitalWrite(LED_PIN, HIGH);
            }
        }
        lastWiFiCheck = now;
    }
}
