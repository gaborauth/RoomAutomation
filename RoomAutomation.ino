/**
 * Build options: Genergic ESP 8266 Module; 160MHz; 1M (64k SPIFFS).
 */
extern "C" {
// Read the "reset" reason...
#include <user_interface.h>
}

#include "WiFiParameters.h"
#include "DeviceParameters.h"

#include <BME280_MOD-1022.h>
#include <DallasTemperature.h>

#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>

#include <TimeLib.h>
#include <OneWire.h>
#include <Wire.h>

#define VERSION        "0.1.18-4"
#define DEEPSLEEP      150000000

#define MAX_OW_DEVICES 10

#define PWM_MAX_VALUE  512
#define PWM_MAX_FREQ   256

#define D0             16

#define D1             5
#define D2             4
#define D3             0
#define D4             2

#define D5             14
#define D6             12
#define D7             13
#define D8             15

#define PIN_WAKE_UP    D0

/**
 * Read VCC on ADC.
 */
ADC_MODE(ADC_VCC);

/**
 * Pin layout.
 */
volatile int PIN_LAYOUT  = 0;  // 0 - old (default); 1 - new

volatile int PIN_RELAY   = D1;
volatile int PIN_SDA     = D2;
volatile int PIN_SCL     = D3; // 10k pull-up
volatile int PIN_SWITCH  = D4; // 10k pull-up

volatile int PIN_PIR     = D5;
volatile int PIN_DS20B18 = D6; // 10k pull-up
volatile int PIN_PWM     = D7;
volatile int PIN_POWER   = D8; // 10k pull-down

/**
 * Reset reason string and UUID of the node.
 */
String resetReason = "unknown";
String MAC_ADDRESS;
String DEVICE_ID;

/**
 * Deep sleep flag.
 */
volatile int deepSleepEnabled = 0;

/**
 * Update check.
 */
volatile unsigned long updateLastChecked = 0;
volatile unsigned long updateCheckDuration = 60000;

/**
 * State of the PWM.
 */
volatile int pwmEnabled = 0;

volatile int pwmBrightnessLookup[PWM_MAX_VALUE + 1];
volatile int pwmMaxPower = PWM_MAX_VALUE;
volatile int pwmCurrentPower = 0;
volatile int pwmTargetPower = -1;
volatile int pwmPrevTargetPower = -1;

/**
 * State of the PWM and the switch.
 */
volatile int switchEnabled = 0;

volatile int switchState = 1;
volatile int switchSavedState = 1;
volatile int switchNeedHandle = 0;
volatile int switchDirection = 0;

volatile unsigned long switchLastChanged = millis();
volatile unsigned long switchLastReceived = millis();

/**
 * State of environmental sensors.
 */
volatile unsigned long sensorLastSent = 0;
volatile unsigned long sensorSendDuration = 60000;

volatile double temp;
volatile double humidity;
volatile double pressure;

/**
 * State of PIR sensor.
 */
volatile int pirEnabled = 0;
 
volatile int pirState = 0;

volatile unsigned long pirLastChange = 0;
volatile unsigned long pirSwitchDuration = 300000;

/**
 * Heating state.
 */
int heatingEnabled = 0;

float heatingTargetTemp = 23.0;
float heatingHysteresis = 0.06;

/**
 * Ventilation state.
 */
int ventilationEnabled = 0;
volatile int ventilationState = LOW;

float ventilationHumidityAverage = 30.0;
float ventilationTargetHumidity = 40.0;
float ventilationHysteresis = 1.0;

/**
 * Relay state.
 */
int relayEnabled = 0;

volatile unsigned long relayState = LOW;

/**
 * State of the AC controller.
 */
volatile int acEnabled = 0;

int acState = 0;
float acTargetTemp = 23.0 + 1.0; // !!! Offset
float acHysteresis = 2.5;

volatile unsigned long acLastStateChange = -8 * 3600000;
volatile unsigned long acForcedCommandSendDuration = 8 * 3600000;

/**
 * PID contoller values.
 */
float acProportional = 20.0;
float acIntegral = 0.01;
float acDerivative = 10.0;

float acIntegralError = 0.0;
float acLastError = 0.0;

/**
 * Infrared sender and receiver states.
 */
volatile int irEnabled = 0;

volatile int irReceivedIndex;
volatile unsigned long irLastReceived;
volatile unsigned long irReceived[2048];
volatile unsigned long irLastWatchdog;

volatile int irSentCommand = 1;
String irReceivedCommand;

char irSendBuff[49];
uint32_t irTimerStart;
volatile unsigned long irHalfPeriodicTime = 500 / 38;

/**
 * AC command constants.
 */
char acCoolFMP4[]  = "101110100100010111011011001001000101001010101101";
char acCoolFMP3[]  = "101110100100010111011010001001010101001010101101";
char acCoolFMP2[]  = "101110100100010111011001001001100101001010101101";
char acCoolFMP1[]  = "101110100100010111011000001001110101001010101101";
char acCoolFM0[]   = "101110100100010111010111001010000101001010101101";
char acCoolFMM1[]  = "101110100100010111010110001010010101001010101101";
char acCoolFMM2[]  = "101110100100010111010101001010100101001010101101";
char acCoolFMM3[]  = "101110100100010111010100001010110101001010101101";
char acCoolFMM4[]  = "101110100100010111010011001011000101001010101101";

char acCoolFMOn[]  = "101110100100010101010111101010000101001010101101";
char acCool23On[]  = "101100100100110110111111010000000101000010101111";

char acOff[]       = "101100100100110101111011100001001110000000011111";

char acHeat23On[]  = "101100100100110110111111010000000101110010100011";
char acHeatFMOn[]  = "101110100100010101010111101010000101111010100001";

char acHeatFMM4[]  = "101110100100010111011011001001000101111010100001";
char acHeatFMM3[]  = "101110100100010111011010001001010101111010100001";
char acHeatFMM2[]  = "101110100100010111011001001001100101111010100001";
char acHeatFMM1[]  = "101110100100010111011000001001110101111010100001";
char acHeatFM0[]   = "101110100100010111010111001010000101111010100001";
char acHeatFMP1[]  = "101110100100010111010110001010010101111010100001";
char acHeatFMP2[]  = "101110100100010111010101001010100101111010100001";
char acHeatFMP3[]  = "101110100100010111010100001010110101111010100001";
char acHeatFMP4[]  = "101110100100010111010011001011000101111010100001";

/**
 * Start web server on the port 80 and start a client.
 */
ESP8266WebServer server(80);
volatile int staticIpEnabled = 0;
volatile int staticIp[4];
volatile int staticRoute[4];
volatile int staticMask[4];

/**
 * NTP synchronization.
 */
WiFiUDP udp;

long ntpTimestampOffset = 0;
volatile unsigned long ntpSendDuration = 600000;
volatile unsigned long ntpLastSent = millis();

/**
 * DS20B18 sensor
 */
OneWire oneWireBus(PIN_DS20B18);
DallasTemperature sensors;

/**
 * RTC data.
 */
struct {
  uint32_t crc32;
  char ssid[32];
  char passphrase[64];
  uint8_t channel;
  uint8_t bssid[6];
  uint8_t padding;
} rtcData;
bool rtcValid = false;

/**
 * Set up the module.
 */
void setup() {
    /**
     * Clearing the serial line.
     */
    Serial.begin(115200);
    Serial.setTimeout(2000);
    while (!Serial) {
    }

    Serial.println();
    for (int i = 0; i < 64; i++) {
        Serial.print(".");
        delay(1);
    }
    Serial.println();
    Serial.println();

    /**
     * Query the unique id from MAC address and check for update.
     */
    byte mac[6]; WiFi.macAddress(mac);
    char macString[13]; sprintf(macString, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    MAC_ADDRESS = String(macString);
    Serial.println("ESP8266 MAC = " + MAC_ADDRESS);
    Serial.println();

    /**
     * Query runtime parameters from the map.
     */
    for(int i = 0; i < NUMBER_OF_DEVICES; i++) {
        if (MAC_ADDRESS.equals(deviceParametersMap[i].name)) {
            DEVICE_ID = deviceParametersMap[i].deviceId;
            PIN_LAYOUT = deviceParametersMap[i].pinLayout;

            acEnabled = deviceParametersMap[i].acEnabled;
            deepSleepEnabled = deviceParametersMap[i].deepSleepEnabled;
            heatingEnabled = deviceParametersMap[i].heatingEnabled;
            pwmEnabled = deviceParametersMap[i].pwmEnabled;
            relayEnabled = deviceParametersMap[i].relayEnabled;
            ventilationEnabled = deviceParametersMap[i].ventilationEnabled;

            staticIpEnabled = deviceParametersMap[i].staticIpEnabled;
            staticIp[0] = deviceParametersMap[i].staticIp[0];
            staticIp[1] = deviceParametersMap[i].staticIp[1];
            staticIp[2] = deviceParametersMap[i].staticIp[2];
            staticIp[3] = deviceParametersMap[i].staticIp[3];
            
            staticRoute[0] = deviceParametersMap[i].staticRoute[0];
            staticRoute[1] = deviceParametersMap[i].staticRoute[1];
            staticRoute[2] = deviceParametersMap[i].staticRoute[2];
            staticRoute[3] = deviceParametersMap[i].staticRoute[3];

            staticMask[0] = deviceParametersMap[i].staticMask[0];
            staticMask[1] = deviceParametersMap[i].staticMask[1];
            staticMask[2] = deviceParametersMap[i].staticMask[2];
            staticMask[3] = deviceParametersMap[i].staticMask[3];
        }
    }

    /**
     * Connect to the WiFi and stand by if WiFi not available.
     */
    wifiConnect();
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot connect, entering deep sleep state...");
        delay(2500);
        ESP.deepSleep(DEEPSLEEP);
    }
    
    /**
     *  Prints the version number and the device parameters.
     */
    Serial.println();
    Serial.println("Version [" + String(PIN_LAYOUT) + "-" + String(VERSION) + "] is starting...");
    Serial.println();
    Serial.println("AC enabled [" + DEVICE_ID + "]: " + String(acEnabled));
    Serial.println("Deep sleep enabled [" + DEVICE_ID + "]: " + String(deepSleepEnabled));
    Serial.println("Heating enabled [" + DEVICE_ID + "]: " + String(heatingEnabled));
    Serial.println("PWM enabled [" + DEVICE_ID + "]: " + String(pwmEnabled));
    Serial.println("Relay enabled [" + DEVICE_ID + "]: " + String(relayEnabled));
    Serial.println("Ventilation enabled [" + DEVICE_ID + "]: " + String(ventilationEnabled));
    Serial.println();

    /**
     * Do HTTP update.
     */
    doHttpUpdate();

    /**
     * Examine reset reason.
     */
    int reason = ESP.getResetInfoPtr()->reason;
    switch (reason) {
        case 0: resetReason = "DEFAULT_RST"; break;
        case 1: resetReason = "WDT_RST"; break;
        case 2: resetReason = "EXCEPTION_RST"; break;
        case 3: resetReason = "SOFT_WDT_RST"; break;
        case 4: resetReason = "SOFT_RESTART"; break;
        case 5: resetReason = "DEEP_SLEEP_AWAKE"; break;
        case 6: resetReason = "EXT_SYS_RST"; break;
        default: resetReason = "UNKNOWN: " + String(reason);
    }
    Serial.println("Reset reason: " + resetReason);
    Serial.println();
    
    /**
     * Initialize the new layout of devices.
     */
    if (PIN_LAYOUT) {
        PIN_RELAY   = D1;
        PIN_SDA     = D2;
        PIN_SWITCH  = D3; // 10k pull-up
        PIN_DS20B18 = D4; // 10k pull-up
        PIN_PIR     = D5;
        PIN_SCL     = D6;
        PIN_POWER   = D7;
        PIN_PWM     = D8; // 10k pull-down
    }

    /**
     * Initialize DS20B18 sensors communication.
     */
    oneWireBus = OneWire(PIN_DS20B18);
    sensors = DallasTemperature(&oneWireBus);

    /**
     * AC unit settings.
     */
    irSendBuff[48] = '\0';

    /**
     * Set PIN modes and attach interrupts.
     */
    pinMode(PIN_RELAY, OUTPUT);  // Relay
    pinMode(PIN_SDA, INPUT);     // BME280 SCA
    pinMode(PIN_SWITCH, INPUT);  // Switch
    pinMode(PIN_PIR, INPUT);     // PIR
    pinMode(PIN_SCL, INPUT);     // BME280 SCL
    pinMode(PIN_POWER, OUTPUT);  // BME280 power

    digitalWrite(PIN_POWER, HIGH);

    if (deepSleepEnabled) {
        sensors.begin();

        return;
    }

    /**
     * Calculate the non-linear brightness of PWM output.
     */
    float R = (PWM_MAX_VALUE * log10(2)) / log10(PWM_MAX_VALUE);
    for (int i = 0; i <= PWM_MAX_VALUE; i++) {
        int brightness = pow(2, (i / R));
        pwmBrightnessLookup[i] = brightness - 1;
    }
    pwmBrightnessLookup[PWM_MAX_VALUE - 1] = PWM_MAX_VALUE;
    pwmBrightnessLookup[PWM_MAX_VALUE] = PWM_MAX_VALUE;

    /**
     * Starting UDP port for NTP.
     */
    Serial.print("Starting UDP for NTP... local port: ");
    udp.begin(2390);
    Serial.println(udp.localPort());

    /**
     * Send simple server status.
     */
    server.on("/", []() {
        Serial.println("Server status");
        server.send(200, "text/plain",
            "Server MAC is '"+ MAC_ADDRESS +"', last reset reason was '" + resetReason + "'.\n" +
            "Version [" + String(PIN_LAYOUT) + "-" + String(VERSION) + "].\n" +
            "Current uptime (timestamp): " + String(millis()) + " ms\n" +
            "Current NTP (timestamp): " + String(unixTimestamp()) + "\n" +
            "Local date and time: " + String(year()) + "-" + String(month()) + "-" + String(day()) + " " +
                String(hour()) + ":" + String(minute()) + ":" + String(second()) + "\n\n" +

            "AC enabled: " + String(acEnabled) + "\n" +
            "Deep sleep enabled: " + String(deepSleepEnabled) + "\n" +
            "Heating enabled: " + String(heatingEnabled) + "\n" +
            "PWM enabled: " + String(pwmEnabled) + "\n" +
            "Relay enabled: " + String(relayEnabled) + "\n" +
            "Ventilation enabled: " + String(ventilationEnabled) + "\n\n" +

            "PWM: " + String(pwmCurrentPower) + "\n" +
            "PWM target power: " + String(pwmTargetPower) + "\n" +
            "PWM previous target power: " + String(pwmPrevTargetPower) + "\n" +
            "PWM maximum power: " + String(pwmMaxPower) + "\n\n" +

            "Switch state: " + String(switchSavedState) + "\n" +
            "Switch direction: " + String(switchDirection) + "\n" +
            "Switch last timestamp: " + String(switchLastReceived) + "\n\n" +

            "Sensor last sent: " + String(sensorLastSent) + "\n" +
            "Temp: " + String(temp) + "\n" +
            "Humidity: " + String(humidity) + "\n" +
            "Pressure: " + String(pressure) + "\n\n" +

            "PIR state: " + String(pirState) + "\n" +
            "PIR last change: " + String(pirLastChange) + "\n\n" +

            "Relay state: " + String(relayState) + "\n" +
            "Ventilation state: " + String(ventilationState) + "\n\n" +
            "Ventilation humidity average: " + String(ventilationHumidityAverage) + "\n\n" +

            "AC proportional / integral / derivative: " + String(acProportional) + " / " + String(acIntegral) + " / " + String(acDerivative) + " / \n" +
            "AC integral error: " + String(acIntegralError) + "\n" +
            "AC last error: " + String(acLastError) + "\n" +

//volatile int irReceivedIndex;
//volatile unsigned long irLastReceived;
//volatile unsigned long irReceived[2048];
//volatile unsigned long irLastWatchdog;
//String irReceivedCommand;
//volatile int irSentCommand = 1;
            "");          
    });

    /**
     * Set the maximum PWM power.
     */
    server.on("/pwm", []() {
        String power = server.arg("power");
        int intPower = power.toInt();
        if (intPower >= 0 && intPower <= PWM_MAX_VALUE) {
            pwmMaxPower = intPower;
        }

        Serial.println("Maximum power of the PWM is " + String(power));
        server.send(200, "text/plain", "Maximum power of the PWM is " + String(pwmMaxPower));
    });

    /**
     * Set the AC target temperature.
     */
    server.on("/acTargetTemp", []() {
        String targetTemp = server.arg("temp");
        acTargetTemp = targetTemp.toInt();

        Serial.println("AC target temp is " + String(targetTemp));
        server.send(200, "text/plain", "AC target temp is " + String(targetTemp));
    });

    /**
     * Toggle the PWM.
     */
    server.on("/toggle", []() {
        if (pwmTargetPower <= pwmMaxPower) {
            pirLastChange = millis();
            pwmTargetPower = pwmMaxPower + 1;
            pwmPrevTargetPower = pwmMaxPower + 1;
            Serial.println("PWM ON");
        } else {
            pwmTargetPower = -1;
            pwmPrevTargetPower = -1;
            Serial.println("PWM OFF");
        }

        Serial.println("The target power of the PWM is " + String(pwmTargetPower));
        server.send(200, "text/plain", "The target power of the PWM is " + String(pwmTargetPower));
    });

    /**
     * Set the send duration of the sensors.
     */
    server.on("/sensor", []() {
        String duration = server.arg("duration");
        sensorSendDuration = duration.toInt();

        Serial.println("Send duration of sensor values is " + String(duration));
        server.send(200, "text/plain", "Send duration of sensor values is " + String(duration));
    });

    /**
     * Send an IR command via HTTP.
     */
    server.on("/ir", []() {
        String command = server.arg("command");
        command.toCharArray(irSendBuff, 48);
        irSendData();

        server.send(200, "text/plain", "IR data sent: " + command);
    });

    /**
     * Start the HTTP server on the port 80.
     */
    server.begin();
    Serial.println("HTTP server started...");
    Serial.println();

    if (pwmEnabled || ventilationEnabled) {
        pinMode(PIN_PWM, OUTPUT);
        analogWriteRange(PWM_MAX_VALUE);
        analogWriteFreq(PWM_MAX_FREQ);
    }

    attachInterrupt(PIN_PIR, handlePirPin, CHANGE);
    attachInterrupt(PIN_SWITCH, handleSwitchPin, CHANGE);

    noInterrupts();
    timer0_isr_init();
    timer0_attachInterrupt(updatePWM);
    timer0_write(ESP.getCycleCount() / 833200 * 833200 + 833200);
    interrupts();

    setSyncProvider(timeProvider);
    setSyncInterval(10);
  
    sensors.begin();
}

/**
 * Connect to the WiFi.
 */
void wifiConnect() {
    if (staticIpEnabled) {
        IPAddress ip(staticIp[0], staticIp[1], staticIp[2], staticIp[3]);
        IPAddress gateway(staticRoute[0], staticRoute[1], staticRoute[2], staticRoute[3]);
        IPAddress subnet(staticMask[0], staticMask[1], staticMask[2], staticMask[3]);
        WiFi.config(ip, gateway, subnet);
    }

    bool rtcValid = false;
    if(ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(rtcData))) {
        uint32_t crc = calculateCRC32(((uint8_t*)&rtcData)+4, sizeof(rtcData)-4);
        Serial.println("RTC CRC32:      " + String(crc));
        if(crc == rtcData.crc32) {
            rtcValid = true;
        }
    }

    if(rtcValid) {
        char macString[13]; sprintf(macString, "%02x%02x%02x%02x%02x%02x", rtcData.bssid[0], rtcData.bssid[1], rtcData.bssid[2], rtcData.bssid[3], rtcData.bssid[4], rtcData.bssid[5]);
        
        Serial.println("RTC SSID:       " + String(rtcData.ssid));
        Serial.println("RTC Passphrase: " + String(rtcData.passphrase));
        Serial.println("RTC Channel:    " + String(rtcData.channel));
        Serial.println("RTC BSSID:      " + String(macString));

        wifiConnect(AP0_SSID, AP0_PASSWORD, true);
    }
        
    if (WiFi.status() != WL_CONNECTED) {
        memcpy(rtcData.ssid, AP0_SSID, sizeof(AP0_SSID)+1);
        memcpy(rtcData.passphrase, AP0_PASSWORD, sizeof(AP0_PASSWORD)+1);
        wifiConnect(AP0_SSID, AP0_PASSWORD, false);
    }

    if (WiFi.status() != WL_CONNECTED) {
        memcpy(rtcData.ssid, AP1_SSID, sizeof(AP1_SSID)+1);
        memcpy(rtcData.passphrase, AP1_PASSWORD, sizeof(AP1_PASSWORD)+1);
        wifiConnect(AP1_SSID, AP1_PASSWORD, false);
    }

    if (WiFi.status() != WL_CONNECTED) {
        memcpy(rtcData.ssid, AP2_SSID, sizeof(AP2_SSID)+1);
        memcpy(rtcData.passphrase, AP2_PASSWORD, sizeof(AP2_PASSWORD)+1);
        wifiConnect(AP2_SSID, AP2_PASSWORD, false);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot connect to WiFi SSIDs...");
        return;
    }

    rtcData.channel = WiFi.channel();
    memcpy(rtcData.bssid, WiFi.BSSID(), 6);
    rtcData.crc32 = calculateCRC32(((uint8_t*)&rtcData)+4, sizeof(rtcData)-4);
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));

    Serial.print("Use this URL to connect: ");
    Serial.print("http://");
    Serial.print(WiFi.localIP());
    Serial.println("/");
    Serial.println();
    Serial.println();
}

/**
 * Connect to the WiFi.
 */
void wifiConnect(char* ssid, char* password, bool rtc) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoConnect(true); // Possible problem with Timer0?
    WiFi.setAutoReconnect(true); // Possible problem with Timer0?
    
    long start = micros();
    if (rtc) {
        Serial.print("Connecting (RTC) to WiFi [ssid=" + String(rtcData.ssid) + "]: ");
        WiFi.begin(rtcData.ssid, rtcData.passphrase, rtcData.channel, rtcData.bssid, true);
    } else {
        WiFi.begin(ssid, password);
        Serial.print("Connecting to WiFi [ssid=" + String(ssid) + "]: ");
    }

    /**
     * Wait 10 seconds to connect...
     */
    int counter = 10000;
    while (WiFi.status() != WL_CONNECTED && counter > 0) {
        delay(1); counter--;
        if (counter % 100 == 0) {
            Serial.print(".");
        }
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" connected (in " + String((micros()-start)/1000) + " ms)");
        return;
    }

    WiFi.disconnect();
    Serial.println(" cannot connect.");
}

/**
 * Try to update the firmware.
 */
void doHttpUpdate() {
    String updateUrl = String(IOT_BASE_URL) + "/firmware/check/" + IOT_USER_ID + "/" + IOT_PASSWORD + "/"  + DEVICE_ID + "/" + String(VERSION);
    Serial.println("Update URL: " + updateUrl);
    t_httpUpdate_return ret = ESPhttpUpdate.update(updateUrl, String(VERSION));
    switch(ret) {
        case HTTP_UPDATE_FAILED: {
            Serial.println("HTTP update: failed(" + String(ESPhttpUpdate.getLastError()) + "): " + ESPhttpUpdate.getLastErrorString());
            break;
        }
        case HTTP_UPDATE_NO_UPDATES: {
            Serial.println("HTTP update: no updates");
            break;
        }
        case HTTP_UPDATE_OK: {
            Serial.println("HTTP update: OK");
            ESP.restart();
            break;
        }
    }
    Serial.println();
}

/**
 * Fade the output to the specified power.
 */
void updatePWM() {
    timer0_write(ESP.getCycleCount() / 833200 * 833200 + 833200);

    if (pwmEnabled) {
        if (pwmTargetPower > pwmCurrentPower) {
            pwmCurrentPower += 1;
        }
        if (pwmTargetPower < pwmCurrentPower) {
            pwmCurrentPower -= 1;
        }

        if (pwmCurrentPower <= 0) {
            digitalWrite(PIN_PWM, LOW);
        } else if (pwmCurrentPower >= PWM_MAX_VALUE) {
            digitalWrite(PIN_PWM, HIGH);
        } else {
            analogWrite(PIN_PWM, pwmBrightnessLookup[pwmCurrentPower]);
        }

        if (pwmTargetPower != -1) {
            if (pirState == 1) {
              pwmTargetPower = pwmPrevTargetPower;
            } else {
                if (pirLastChange + pirSwitchDuration < millis() && pwmTargetPower > pwmPrevTargetPower / 4 * 3) {
                    pwmPrevTargetPower = pwmTargetPower;
                    pwmTargetPower = pwmPrevTargetPower / 4 * 3;
                } else if (pirLastChange + pirSwitchDuration * 2 < millis()) {
                  pwmTargetPower = -1;
                  pwmPrevTargetPower = -1;
                  switchDirection = 0;
                  digitalWrite(PIN_RELAY, LOW);
                }
            }
        }

        if (switchNeedHandle == 1 && switchLastReceived < millis()) {
            switchNeedHandle = 0;
            if (switchState == switchSavedState) {
                return;
            }
            switchSavedState = switchState;
            
            if (switchState == LOW) {
                switchLastChanged = millis();
            }

            if (switchState == HIGH) {
                if (switchLastReceived - switchLastChanged > 250) {
                    pirLastChange = millis();
                    pwmTargetPower = pwmCurrentPower;
                    pwmPrevTargetPower = pwmCurrentPower;
                    if (switchLastReceived - switchLastChanged > 5000) {
                        pirLastChange = millis();
                        if (relayEnabled) {
                            digitalWrite(PIN_RELAY, HIGH);
                        }
                    }
                }
                return;
            }
            
            if (switchDirection == 0) {
                pirLastChange = millis();
                pwmTargetPower = pwmMaxPower + 1;
                pwmPrevTargetPower = pwmMaxPower + 1;
                switchDirection = 1;
            } else {
                pirLastChange = millis();
                pwmTargetPower = -1;
                pwmPrevTargetPower = -1;
                switchDirection = 0;
                if (relayEnabled) {
                    digitalWrite(PIN_RELAY, LOW);
                }
            }
        }       
    } else if (acEnabled) {
        unsigned long delta = micros() - irLastReceived;

        // Save the result if nothing happened until 10ms ago.
        if (delta > 10000 && irReceivedIndex > 0) {
            // Dump bits of the received message.
            int bits = 0;
            char irBuffer[49];
            for (int i = 0; i < irReceivedIndex; i = i + 2) {
                if (bits > 47) continue;

                int bitWidth = irReceived[i] + irReceived[i + 1];
                if (bitWidth > 900 && bitWidth < 1200) {
                    irBuffer[bits++]  = '0';
                } else if (bitWidth > 2000 && bitWidth < 2300) {
                    irBuffer[bits++]  = '1';
                }
            }
            irBuffer[48] = '\0';

            irReceivedCommand = String(irBuffer);
            Serial.println("Received command: " + irReceivedCommand);
            irSentCommand = 0;

            irReceivedIndex = 0;
        }

        if (micros() - irLastWatchdog > 30000000) {
            irLastWatchdog = micros();
            Serial.println("tick");
        }
    }
}

/**
 * Handle the PIR sensor output.
 */
void handlePirPin() {
    if (pwmEnabled) {
        int pin = digitalRead(PIN_PIR);

        if (pin == HIGH && pirState == LOW) {
            pirLastChange = millis();
            pirState = HIGH;
        } else if (pin == LOW && pirState == HIGH) {
            pirLastChange = millis();
            pirState = LOW;
        }
    } else if (acEnabled) {
        unsigned long us = micros();
        unsigned long delta = us - irLastReceived;
        irLastReceived = us;

        // Save edge time when the delta less than 10 ms.
        if (delta < 10000) {
            irReceived[irReceivedIndex] = delta;
            irReceivedIndex++;
        }
    }
}

/**
 * Handle the input of the switch port.
 */
void handleSwitchPin() {
    switchState = digitalRead(PIN_SWITCH);
    switchNeedHandle = 1;
    switchLastReceived = millis() + 50;
}

/**
 * Main loop.
 */
void loop()
{
    handleSensor();

    if (deepSleepEnabled) {
        digitalWrite(PIN_POWER, LOW);
        Serial.println("Entering deep sleep state...");
        ESP.deepSleep(deepSleepEnabled);
    }
    
    handleNtpPacket();
    checkForUpdate();
    server.handleClient();
}

/**
 * Check for update on the specified server.
 */
void checkForUpdate() {
    if (updateLastChecked == 0 || updateLastChecked + updateCheckDuration < millis()) {
        updateLastChecked = millis();
    } else {
        return;
    }

    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    http.begin(String(IOT_BASE_URL) + "/firmware/check/" + IOT_USER_ID + "/" + IOT_PASSWORD + "/" + DEVICE_ID + "/" + String(VERSION) + "/" + WiFi.localIP().toString());
    int code = http.GET();
    Serial.println("Checking for new firmware: " + String(code));
    http.end();

    if (code == 200) {
        ESP.restart();
    }
}

/**
 * Handle the sensor.
 */
void handleSensor() {
    if (sensorLastSent == 0 || sensorLastSent + sensorSendDuration < millis()) {
        sensorLastSent = millis();
    } else {
        return;
    }

    String vcc = String(ESP.getVcc());

    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    /**
     * TODO: report the VCC via DEVICE_ID.
     */
    String NODE_KEY = nodeKeyByDeviceId();
    if (String("not-found-node-key") != NODE_KEY) {
        http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/vcc/" + vcc);
        Serial.println("Sent 'vcc': " + String(http.GET()));
        http.end();
    }

    /**
     * Initiate the DS20B18 sensors.
     */
    DeviceAddress deviceAddresses[MAX_OW_DEVICES];
    int devices = discoverOneWireDevices(deviceAddresses, MAX_OW_DEVICES);

    sensors.requestTemperatures();
    for (int i = 0; i < devices; i++) {
        sendTemperature(deviceAddresses[i], vcc);
    }

    /**
     * Initiate the BME280 sensor.
     */
    digitalWrite(PIN_POWER, HIGH);
    delay(5);

    Wire.begin(PIN_SDA, PIN_SCL);
    uint8_t chipID = BME280.readChipId();
    Serial.print("ChipID = 0x");
    Serial.println(chipID, HEX);

    if (chipID != 0x60) {
        digitalWrite(PIN_POWER, LOW);

        heatController();
        acController();
        ventilationController();

        return;
    }

    BME280.readCompensationParams();
    BME280.writeOversamplingPressure(os16x);
    BME280.writeOversamplingTemperature(os16x);
    BME280.writeOversamplingHumidity(os16x);

    BME280.writeMode(smForced);

    int result = repeatedBMERead();

    BME280.writeMode(smNormal);

    digitalWrite(PIN_POWER, LOW);

    if (result == 0) {
        Serial.println("Cannot read valid measurements...");
        return;
    }

    Serial.println("Temperature  " + String(temp));
    Serial.println("Humidity     " + String(humidity));
    Serial.println("Pressure     " + String(pressure));

    http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/temperature/" + String(temp));
    Serial.println("Sent 'temperature': " + String(http.GET()));
    http.end();

    http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/pressure/" + String(pressure));
    Serial.println("Sent 'pressure': " + String(http.GET()));
    http.end();

    http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/humidity/" + String(humidity));
    Serial.println("Sent 'humidity': " + String(http.GET()));
    http.end();

    heatController();
    acController();
    ventilationController();
}

/**
 * Repeated BME read.
 */
int repeatedBMERead() {
    for (int i = 0; i < 10; i++) {
        Serial.println("Measuring...");
        while (BME280.isMeasuring()) {
        }

        noInterrupts();
        BME280.readMeasurements();
        interrupts();

        temp = BME280.getTemperature();
        humidity = BME280.getHumidity();
        pressure = BME280.getPressure();

        BME280.writeStandbyTime(tsb_0p5ms);
        BME280.writeFilterCoefficient(fc_16);

        if (pressure > 800) {
            return 1;
        }

        delay(10);
    }

    return 0;
}

/**
 * Send temperature of the specified DS18B20 device.
 */
void sendTemperature(DeviceAddress address, String vcc) {
    temp = getTemperature(address);

    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    String NODE_KEY = nodeKeyByAddress(addressToString(address));
    http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/vcc/" + vcc);
    Serial.println("Sent 'vcc': " + String(http.GET()));
    http.end();

    http.begin(String(IOT_BASE_URL) + "/measurement/create/" + NODE_KEY + "/temperature/" + String(temp));
    Serial.println("Sent 'temperature': " + String(http.GET()));
    http.end();
}

/**
 * Control ventilation through PWM output.
 */
void ventilationController() {
    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    /**
     * Ventilation logic (simple on/off).
     */
    if(ventilationEnabled) {
        Serial.println("PWM based ventilation controller");

        http.begin(String(IOT_BASE_URL) + "/measurement/loadLastFloatValue/15c28f10-00cc-11e7-832f-2b6139351b1b/humidity");
        Serial.println("Load humidity of '5ccf7fd89d76': " + String(http.GET()));
        String humidityStr = http.getString();
        http.end();

        if (humidityStr.length() == 0) {
           ventilationHumidityAverage = humidity;
        } else {
           ventilationHumidityAverage = humidityStr.toFloat();
        }

        if (ventilationHumidityAverage > ventilationTargetHumidity + ventilationHysteresis) {
            ventilationState = HIGH;
        } else if (ventilationHumidityAverage < ventilationTargetHumidity - ventilationHysteresis) {
            ventilationState = LOW;
        }

        /**
         * 10 minutes ventilation on every hour.
         */
        if (minute() > 50) {
            ventilationState = HIGH;
        }

        /**
         * Night silent mode... :)
         */
        if (hour() < 8 || hour() > 21) {
            ventilationState = LOW;
        }
        
        digitalWrite(PIN_PWM, ventilationState);
        Serial.println("Humidity average: " + String(ventilationHumidityAverage) + ", relay: " + String(ventilationState));
    }
}

/**
 * Control heating units through relay output.
 */
void heatController() {
    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);

    /**
     * Send the received IR command.
     */
    if (irSentCommand == 0) {
        http.begin(String(IOT_BASE_URL) + "/irCommand/create/" + DEVICE_ID + "/" + irReceivedCommand);
        Serial.println("Sent 'ir command': " + String(http.GET()));
        http.end();

        irSentCommand = 1;
    }

    /**
     * Relay (on/off) controller.
     */
    if(heatingEnabled) {
        Serial.println("Relay based heat controller");

        http.begin(String(IOT_BASE_URL) + "/measurement/loadLastFloatValue/14164380-997b-11e7-85ac-9591c57c2076/temperature");
        Serial.println("Load temperature of '5ccf7fd93c7b': " + String(http.GET()));
        String tempStr = http.getString();
        http.end();

        float tempAvg;
        if (tempStr.length() == 0) {
           tempAvg = temp;
        } else {
           tempAvg = tempStr.toFloat() - 1.0;
        }
        
        if (tempAvg > heatingTargetTemp + heatingHysteresis) {
            relayState = LOW;
            digitalWrite(PIN_RELAY, LOW);
        } else if (tempAvg < heatingTargetTemp - heatingHysteresis) {
            relayState = HIGH;
            digitalWrite(PIN_RELAY, HIGH);
        }
        Serial.println("Temperature average: " + String(tempAvg) + ", relay: " + String(relayState));
    }
}


/**
 * Control AC units.
 */
void acController() {
    HTTPClient http;
    http.useHTTP10(true);
    http.setTimeout(8000);
    
    /**
     * PID controller via IR.
     */
    if(acEnabled) {
        Serial.print("AC controller enabled...");

        int acNewState = 0;
        if (temp < acTargetTemp && acState != -1) {
            Serial.println("Heating with PID controller...");
            acNewState = -1;
        } else if (temp > acTargetTemp + acHysteresis && acState != 1) {
            Serial.println("Cooling with PID controller...");
            acNewState = 1;
        }

        if (acNewState == 0 && acState == 0 && acLastStateChange + acForcedCommandSendDuration < millis()) {
            acLastStateChange = millis();
            acState = acNewState;

            for (int i = 0; i < 48; i++) {
                irSendBuff[i] = acOff[i];
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" +DEVICE_ID + "/" + String(irSendBuff) + "/off");
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            long now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");
        }

        if (acNewState == -1 || acState == -1 && acLastStateChange + acForcedCommandSendDuration < millis()) {
            acLastStateChange = millis();

            if (acNewState == -1) {
                acState = acNewState;
                acIntegralError = 0.0;
                acLastError = 0.0;
            }

            for (int i = 0; i < 48; i++) {
                irSendBuff[i] = acHeat23On[i];
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/h23/on");
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            long now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");

            for (int i = 0; i < 48; i++) {
                irSendBuff[i] = acHeatFMOn[i];
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/h23/fm");
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");
        }
        if (acNewState == 1 || acState == 1 && acLastStateChange + acForcedCommandSendDuration < millis()) {
            acLastStateChange = millis();

            if (acNewState == 1) {
                acState = acNewState;
                acIntegralError = 0.0;
                acLastError = 0.0;
            }

            for (int i = 0; i < 48; i++) {
                irSendBuff[i] = acCool23On[i];
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/c23/on");
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            long now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");

            for (int i = 0; i < 48; i++) {
                irSendBuff[i] = acCoolFMOn[i];
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/c23/fm");
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");
        }

        if (acState == -1) {
            float error = acTargetTemp - temp;
            float acDerivativeError = error - acLastError;
            acIntegralError = acIntegralError + error;
            acLastError = error;

            float followMeState = acProportional * error + acIntegral * acIntegralError + acDerivative * acDerivativeError;

            if (followMeState > 4) {
                Serial.println("Turbo heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMP4[i];
                }
            } else if (followMeState > 3) {
                Serial.println("Maximum heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMP3[i];
                }
            } else if (followMeState > 2) {
                Serial.println("Medium heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMP2[i];
                }
            } else if (followMeState > 1) {
                Serial.println("Low heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMP1[i];
                }
            } else if (followMeState > 0) {
                Serial.println("Fan heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFM0[i];
                }
            } else if (followMeState > -1) {
                Serial.println("Fan heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMM1[i];
                }
            } else if (followMeState > -2) {
                Serial.println("Fan heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMM2[i];
                }
            } else if (followMeState > -3) {
                Serial.println("Fan heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMM3[i];
                }
            } else {
                Serial.println("Fan heating...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acHeatFMM4[i];
                }
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/hfm/" + followMeState + "/" + error + "/" + acIntegralError + "/" + acDerivativeError);
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            long now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");
        }

        if (acState == 1) {
            float error = acTargetTemp - temp;
            float acDerivativeError = error - acLastError;
            acIntegralError = acIntegralError + error;
            acLastError = error;

            float followMeState = acProportional * error + acIntegral * acIntegralError + acDerivative * acDerivativeError;

            if (followMeState > 4) {
                Serial.println("Turbo cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMP4[i];
                }
            } else if (followMeState > 3) {
                Serial.println("Maximum cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMP3[i];
                }
            } else if (followMeState > 2) {
                Serial.println("Medium cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMP2[i];
                }
            } else if (followMeState > 1) {
                Serial.println("Low cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMP1[i];
                }
            } else if (followMeState > 0) {
                Serial.println("Fan cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFM0[i];
                }
            } else if (followMeState > -1) {
                Serial.println("Fan cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMM1[i];
                }
            } else if (followMeState > -2) {
                Serial.println("Fan cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMM2[i];
                }
            } else if (followMeState > -3) {
                Serial.println("Fan cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMM3[i];
                }
            } else {
                Serial.println("Fan cooling...");
                for (int i = 0; i < 48; i++) {
                    irSendBuff[i] = acCoolFMM4[i];
                }
            }

            http.begin(String(IOT_BASE_URL) + "/acCommand/create/" + DEVICE_ID + "/" + String(irSendBuff) + "/cfm/" + followMeState + "/" + error + "/" + acIntegralError + "/" + acDerivativeError);
            Serial.println("Sent 'ir command': " + String(http.GET()));
            http.end();

            long now = micros();
            irSendData();
            Serial.println("Total time: " + String(micros() - now) + " us");
        }
    }
}

/**
 * Returns with the temperature of the Dallas unit.
 *
 * @param address the address
 * @return the temperature
 */
float getTemperature(DeviceAddress address) {
    int counter = 0;
    do {
        temp = sensors.getTempC(address);
        delay((counter++) * 10);
    } while ((temp == 85.0 || temp == (-127.0)) && counter < 10);
    Serial.println("Temperature of '" + addressToString(address) + "': " + String(temp));

    return temp;
}

/**
 * Discover DS18B20 devices.
 */
int discoverOneWireDevices(DeviceAddress deviceAddresses[], int maxDevices) {
    Serial.println("Looking for 1-Wire devices...");

    byte addr[8];
    int count = 0;
    while (count < maxDevices && oneWireBus.search(addr)) {
        for(int i = 0; i < 8; i++) {
            deviceAddresses[count][i] = addr[i];
        }
        if (OneWire::crc8(addr, 7) != addr[7]) {
            Serial.println("CRC is not valid, aborting...");
            return 0;
        }

        Serial.println("Found 1-Wire device with address: " + addressToString(addr));
        count++;
    }

    Serial.println("Found " + String(count) + " devices.");
    Serial.println("");

    oneWireBus.reset_search();

    return count;
}

/**
 * Convert DS1820's address to String.
 */
String addressToString(DeviceAddress address) {
    String result = "";
    for (int i = 0; i < 8; i++) {
        if (address[i] < 16) {
            result += "0";
        }

        result += String(address[i], HEX);
    }

    return result;
}

/**
 * Send IR data on IR channel.
 */
void irSendData() {
    irMark(4445);
    irSpace(4385);

    for (uint16_t i = 0; i < 48; i++) {
        if (irSendBuff[i] == '0') {
            irMark(585);
            irSpace(495);
        } else {
            irMark(585);
            irSpace(1570);
        }
    }

    irMark(585);
    irSpace(5170);

    irMark(4445);
    irSpace(4385);

    for (uint16_t i = 0; i < 48; i++) {
        if (irSendBuff[i] == '0') {
            irMark(585);
            irSpace(495);
        } else {
            irMark(585);
            irSpace(1570);
        }
    }

    irMark(585);
    irSpace(5170);
}

/**
 * Write 'mark' on IR channel.
 */
void irMark(unsigned int usec) {
  irTimerReset();
  while (irTimerElapsed() < usec) {
    digitalWrite(PIN_PWM, HIGH);
    delayMicroseconds(irHalfPeriodicTime);
    digitalWrite(PIN_PWM, LOW);
    delayMicroseconds(irHalfPeriodicTime);
  }
}

/**
 * Write 'space' on IR channel.
 */
void irSpace(unsigned int usec) {
    digitalWrite(PIN_PWM, LOW);
    delayMicroseconds(usec);
}

/**
 * Reset the timer.
 */
void irTimerReset() {
    irTimerStart = micros();
}

/**
 * Elapsed microseconds.
 * 
 * @return elapsed microseconds.
 */
uint32_t irTimerElapsed() {
    uint32_t now = micros();

    if (irTimerStart <= now) {
        return (now - irTimerStart);
    }

    return (0xffffffff - irTimerStart + now);
}

/**
 * Calculate CRC32 of the data.
 * 
 * @param data the data
 * @param length the length of data
 * @return the crc32 value
 */
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xffffffff;

    while(length--) {
        uint8_t c = *data++;
        for(uint32_t i = 0x80; i > 0; i >>= 1) {
            bool bit = crc & 0x80000000;
            if(c & i) {
                bit = !bit;
            }

            crc <<= 1;
            if(bit) {
                crc ^= 0x04c11db7;
            }
        }
    }

    return crc;
}

/**
 * Return with UNIX timestamp.
 */
long unixTimestamp() {
    if (ntpTimestampOffset == 0) {
        return 0;
    }

    return millis() / 1000 + ntpTimestampOffset;
}

/**
 * Time provider of Time library.
 */
time_t timeProvider() {
    return unixTimestamp();
}

/**
 * .Handle the NTP packet.
 */
void handleNtpPacket() {
    byte packetBuffer[48];
    unsigned long sendDuration = (ntpTimestampOffset == 0) ? 1000 : ntpSendDuration;
    if (ntpLastSent + sendDuration < millis()) {
        ntpLastSent = millis();

        const char* ntpServerName = "0.hu.pool.ntp.org";
        IPAddress timeServerIP;
        WiFi.hostByName(ntpServerName, timeServerIP);

        memset(packetBuffer, 0, 48);
        packetBuffer[0] = 0b11100011;
        packetBuffer[1] = 0;
        packetBuffer[2] = 6;
        packetBuffer[3] = 0xEC;
        packetBuffer[12]  = 49;
        packetBuffer[13]  = 0x4E;
        packetBuffer[14]  = 49;
        packetBuffer[15]  = 52;

        udp.beginPacket(timeServerIP, 123);
        udp.write(packetBuffer, 48);
        udp.endPacket();
        Serial.println("NTP request sent");

        return;
    }

    if (udp.parsePacket() != 48) {
        return;
    }

    udp.read(packetBuffer, 48);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    unsigned long milliseconds = packetBuffer[39] * 1000L / 256;

    ntpTimestampOffset = secsSince1900 - 2208988800UL - millis() / 1000;
    Serial.println("NTP response received, the timestamp is " + String(millis() / 1000 + ntpTimestampOffset));
}

/**
 * Returns nodeKey by deviceId.
 */
String nodeKeyByDeviceId() {
    for(int i = 0; i < NUMBER_OF_NODES; i++) {
        if (MAC_ADDRESS.equals(nodeParametersMap[i].name)) {
          return nodeParametersMap[i].nodeKey;
        }
    }

    return "not-found-node-key";
}

/**
 * Returns nodeKey by address.
 */
String nodeKeyByAddress(String address) {
    for(int i = 0; i < NUMBER_OF_NODES; i++) {
        if (address.equals(nodeParametersMap[i].name)) {
          return nodeParametersMap[i].nodeKey;
        }
    }

    return "not-found-node-key";
}

