/*
  AnTester (LoRa Antenna Efficacy Tester)
  by Jon Silver, http://jfdi.info
  "Standing on the shoulders of giants"
*/

#include "main.h"
#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RHEncryptedDriver.h>
#include <RH_RF95.h>
#include <CryptoLW.h>
#include <Speck.h>
#include <ElapsedMillis.h>
#ifdef USEDISPLAY
    #include <Wire.h>
    #include "SSD1306Wire.h"
    #include "lato-regular-12.h"
#endif
#include <inttypes.h>
#include "ClickButton.h"
#include <algorithm>

// #define DEBUG_LEVEL DEBUG_TRACE // now defined in platformio.ini
// #define DEBUG_LEVEL DEBUG_NONE // now defined in platformio.ini
#include <Debug.h>

const uint8_t encryptionKey[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // Secret encryption key

// Singleton instance of the radio radio
RH_RF95 radio(RADIO_CS, RADIO_INT);
#ifdef USEENCRYPTION
    Speck cipher;
    RHEncryptedDriver driver(radio, cipher);
    // Class to manage message delivery and receipt, using the radio declared above
    RHReliableDatagram manager(driver, DEFAULTSERVERADDRESS);
#else
    RHReliableDatagram manager(radio, config.address);
#endif

#ifdef USEDISPLAY
    SSD1306Wire display(0x3c, SDA, SCL);
    bool displayOn = true;
    elapsedMillis displayTimer;
#endif
ClickButton userButton(USERBUTTON, LOW, CLICKBTN_PULLUP);
elapsedMillis ledTimer;
// elapsedMillis timeout;

const char firmwareTitle[] PROGMEM = {FIRMWARETITLE};

#define INITMAXRSSI -130
#define INITMINSNR 9999
#define INITMINFERR 9999

int16_t lastRSSI, maxRSSI = INITMAXRSSI, minRSSI, meanRSSI;
int16_t lastSNR, maxSNR, minSNR = INITMINSNR, meanSNR;
int16_t lastFErr, maxFErr, minFErr = INITMINFERR, meanFErr;
uint8_t msgCount;
uint8_t ackCount;
uint32_t ledTimeout;
String mode = "RX";


void setup()
{
    resetDisplay();

    #if DEBUG_LEVEL > DEBUG_NONE
        esp_log_level_set("*", ESP_LOG_WARN);
        Serial.begin(115200);
        Serial.setDebugOutput(false);
        while (!Serial);
    #endif

    DEBUG5_PRINT(FIRMWARETITLE);
    DEBUG5_VALUELN(" SDK version:", ESP.getSdkVersion());
    
    setPins();
    userButton.debounceTime   = 20;   // Debounce timer in ms
    userButton.multiclickTime = 250;  // Time limit for multi clicks
    userButton.longClickTime  = 1500; // time until "held-down clicks" register
    
    initRadio();
    display.init();
    display.displayOn();
    displayConnecting("Waiting...");
}


void loop()
{
    handleButton();
    yield();

    receiveMessage();
    timeoutLED();

    // if (timeout > 5000) {
    // }
}


void receiveMessage() {
    if (manager.available())
    {
        // timeout = 0;
        mode = "RX";

        DEBUG5_PRINT("Message waiting... ");
        // Wait for a message addressed to us from the client
        uint8_t buf[50];
        uint8_t len = sizeof(buf);
        uint8_t from, to, msgId;

        DEBUG5_PRINT("Receiving message... ");
        if (manager.recvfromAck(buf, &len, &from, &to, &msgId)) {
            DEBUG5_PRINTLN("Message complete.");

            ackCount++;
            updateStats();
            DEBUG5_VALUE("msg from ", from);
            DEBUG5_VALUE(" to ", to);
            DEBUG5_VALUE(" | bytes: ", len);
            DEBUG5_VALUELN(" | rssi: ", lastRSSI);

            handleMessage(buf, len, from, msgId, lastRSSI, false);
        } else {
            DEBUG5_PRINTLN("failed.");
        }
    }
}


void handleMessage(uint8_t *buf, uint8_t len, uint8_t from, uint8_t messageId, int16_t rssi, bool nearby)
{
    flashLEDOnce(100);

    char msg[len+1] = {0};
    memcpy(&msg, buf, len);
    DEBUG5_VALUE("msg: ", msg);
    DEBUG5_VALUE(" (", len);
    DEBUG5_PRINTLN(")");
    updateDisplay(String(msg));
}


void sendTestMessages() {
    mode = "TX";

    for (uint8_t i = 1; i <= 10; i++) {
        flashLEDOnce(100);

        msgCount++;
        String msg = "Message " + String(msgCount);
        uint8_t buf[msg.length()];
        memcpy(&buf, msg.c_str(), sizeof(buf));
        DEBUG5_VALUE("Sending '", msg);
        DEBUG5_PRINT("'... ");
        if (manager.sendtoWait(buf, sizeof(buf), 7)) {
            ackCount++;
            updateStats();
            // timeout = 0;
            DEBUG5_PRINTLN("Ack'd.");
        } else {
            DEBUG5_PRINTLN("not Ack'd.");
        }
        updateDisplay(msg);
    }
}


void updateStats() {
    int16_t rssi = radio.lastRssi();
    int16_t snr = radio.lastSNR();
    int16_t ferr = radio.frequencyError();

    minRSSI = std::min(rssi, minRSSI);
    maxRSSI = std::max(rssi, maxRSSI);
    
    meanRSSI = (meanRSSI + rssi) / 2;

    minSNR = std::min(snr, minSNR);
    maxSNR = std::max(snr, maxSNR);
    meanSNR = (meanSNR + snr) / 2;

    minFErr = std::min(ferr, minFErr);
    maxFErr = std::max(ferr, maxFErr);
    meanFErr = (meanFErr + ferr) / 2;

    lastRSSI = rssi;
    lastSNR = snr;
    lastFErr = ferr;
}


void updateDisplay(String msg) {
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, mode + ": " + msg + " (" + String(ackCount) + " ok)");
    display.drawString(0, 12, "Last rssi " + String(lastRSSI) + " snr " + String(lastSNR) + " ferr " + String(lastFErr));
    display.drawString(0, 24, "Max rssi " + String(maxRSSI) + " snr " + String(maxSNR) + " ferr " + String(maxFErr));
    display.drawString(0, 36, "Min rssi " + String(minRSSI) + " snr " + String(minSNR) + " ferr " + String(minFErr));
    display.drawString(0, 48, "Avg rssi " + String(meanRSSI) + " snr " + String(meanSNR) + " ferr " + String(meanFErr));
    display.display();
}


void setPins() {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
}


void handleButton() {
    userButton.Update();

    if (userButton.clicks == -1) {
        DEBUG5_PRINTLN("LONG PRESS");
        // timeout = 0;
        msgCount = 0;
        ackCount = 0;
        minFErr = INITMINFERR;
        maxFErr = 0;
        meanFErr = 0;
        minRSSI = 0;
        maxRSSI = INITMAXRSSI;
        meanRSSI = 0;
        minSNR = INITMINSNR;
        maxSNR = 0;
        meanSNR = 0;
        lastRSSI = 0;
        lastSNR = 0;
        lastFErr = 0;
        updateDisplay("WAITING");
    } else if (userButton.clicks == 1) {
        DEBUG5_PRINTLN("CLICK");
        updateDisplay("Transmitting...");
        sendTestMessages();
    }
}


void flashLEDOnce(uint32_t duration) {
    switchLEDOn();
    ledTimeout = duration;
    ledTimer = 0;
}


void switchLED(uint8_t state) {
    digitalWrite(LED, state);
}


void switchLEDOn() {
    switchLED(HIGH);
}


void switchLEDOff() {
    switchLED(LOW);
}


void toggleLED()
{
    digitalWrite(LED, !(digitalRead(LED))); //Invert Current State of LED
}


void initRadio() {
    if (!manager.init())
    {
        DEBUG_ERR("Radio init failed");
        DEBUG_ERR_STATE(DEBUG_ERR_UNINIT);
    } else {
        DEBUG5_PRINTLN("Radio initialised");
    }

    if (radio.setFrequency(RADIO_FREQ)) {
        DEBUG5_VALUELN("Set Freq: ", RADIO_FREQ);
    } else {
        DEBUG5_VALUELN("Freq Fail: ", RADIO_FREQ);
    }
    
    // Set transmitter power using PA_BOOST.
    radio.setTxPower(TXPOWER, false);
    // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
    radio.setModemConfig(MODEMCONFIG);
    // wait 1s until Channel Activity Detection shows no activity on the channel before transmitting
    radio.setCADTimeout(CADTIMEOUT);

    manager.setThisAddress(DEFAULTSERVERADDRESS);
    #ifdef USEENCRYPTION
        cipher.setKey(encryptionKey, sizeof(encryptionKey));
    #endif
}


void resetRadio() {
    // manual reset
    digitalWrite(RADIO_RST, HIGH);
    delay(10);
    digitalWrite(RADIO_RST, LOW);
    delay(1);
    digitalWrite(RADIO_RST, HIGH);
    delay(1);
    DEBUG5_PRINTLN("Radio reset");
}


void resetDisplay()
{
    pinMode(OLED_RESET, OUTPUT);
    digitalWrite(OLED_RESET, LOW); // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(OLED_RESET, HIGH); // while OLED is running, must set GPIO16 in high、
}


void displayTitle() {
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(63, 0, firmwareTitle);
}


void displayConnecting(String msg) {
    display.clear();
    display.setFont(Lato_Regular_12);
    displayTitle();

    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_CENTER_BOTH);
    display.drawString(63, 31, msg);
    display.display();
}


void timeoutLED() {
    if (ledTimer > ledTimeout && digitalRead(LED) == HIGH) {
        switchLEDOff();
    }
}