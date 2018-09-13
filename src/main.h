#ifndef __main_h
#define __main_h
#endif

#include <inttypes.h>
#include <RH_RF95.h>


#define FIRMWARENAME "AnTester" 
#define FIRMWARETITLE FIRMWARENAME " " FIRMWAREVER

// Pin assignments for the Heltec/TTGO ESP32 LoRa OLED board
#define RADIO_CS 18
#define RADIO_RST 14
#define RADIO_INT 26

#define OLED_RESET 16
#define SDA 4
#define SCL 15

#define DEFAULTSERVERADDRESS 7
#define RADIO_FREQ 433.5    // bang in the middle of the 433MHz band
#define CHANNELBANDWIDTH 125.0
#define MODEMCONFIG RH_RF95::Bw125Cr45Sf128
#define TXPOWER 23 // 13 for a reasonable TX power, 23 to blast the hell out of everything
#define CADTIMEOUT 1000
#define LED 2
#define USERBUTTON 0

#define STRICT_CONTENT_LEN


void receiveMessage();
void handleMessage(uint8_t *buf, uint8_t len, uint8_t from, uint8_t messageId);
void handleButton();
void flashLEDOnce(uint32_t duration);
void switchLED(uint8_t state);
void switchLEDOn();
void switchLEDOff();
void toggleLED();
void initRadio();
void resetRadio();
void setPins();
void resetDisplay();
void displayTitle();
void displayLogo();
void displayWait(String msg);
void timeoutLED();
void sendTestMessages();
void updateDisplay(String msg);
void updateStats();
