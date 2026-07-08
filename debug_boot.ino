/**
 * Debug boot — finds exactly where WROVER-E crashes
 * Prints a checkpoint before each operation
 * Watch serial to see the last checkpoint before reset
 */
#include <Arduino.h>
#include <WiFi.h>

#define SERVO_PIN  12
#define LED_PIN     2
#define SERVO_FREQ  50
#define SERVO_BITS  16

void setup() {
    Serial.begin(115200);
    delay(1000);   // give serial time to connect
    Serial.println("\n\n=== DEBUG BOOT ===");

    Serial.println("[1] pinMode LED");
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    delay(100);

    Serial.println("[2] ledcAttach servo");
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS);
    delay(100);

    Serial.println("[3] ledcWrite 1500us (stop)");
    uint32_t duty = (uint32_t)1500 * 65535UL / 20000UL;
    ledcWrite(SERVO_PIN, duty);
    delay(100);

    Serial.println("[4] WiFi.mode AP");
    WiFi.mode(WIFI_AP);
    delay(500);

    Serial.println("[5] WiFi.softAP");
    bool ok = WiFi.softAP("SmartLauncher", "");
    Serial.printf("[5] softAP result: %s\n", ok ? "OK" : "FAILED");
    delay(500);

    Serial.printf("[6] IP: %s\n", WiFi.softAPIP().toString().c_str());

    Serial.println("[7] All done — loop starting");
    Serial.println("If you see this, setup completed successfully");
}

void loop() {
    digitalWrite(LED_PIN, LOW);  delay(200);
    digitalWrite(LED_PIN, HIGH); delay(200);
    Serial.println("[LOOP] alive");
    delay(1000);
}
