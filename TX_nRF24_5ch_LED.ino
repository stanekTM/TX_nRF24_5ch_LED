
//*********************************************************************************************************************
// RC transmitter
//****************
// Simple 5 channel RC transmitter from my repository https://github.com/stanekTM/TX_nRF24_5ch_LED
//
// This RC transmitter works with RC receiver from my repository https://github.com/stanekTM/RX_nRF24_Motor_Servo
//
// Thank you to "Phil_G" http://www.singlechannel.co.uk for the calibration and reverse routine I used in the code.
//*********************************************************************************************************************

#include <RF24.h>   // https://github.com/nRF24/RF24
#include <SPI.h>    // Arduino standard library
#include <EEPROM.h> // Arduino standard library


// Setting a unique address (5 bytes number or character)
const byte address[] = "jirka";

// RF communication channel settings (0-125, 2.4Ghz + 76 = 2.476Ghz)
#define RADIO_CHANNEL         76

// TX alarm voltage setting
#define TX_BATTERY_VOLTAGE    4.2
#define TX_MONITORED_VOLTAGE  3.3

// RX alarm voltage setting
#define RX_BATTERY_VOLTAGE    4.2
#define RX_MONITORED_VOLTAGE  3.49

// Setting the control range value
#define MIN_CONTROL_VAL       1000
#define MID_CONTROL_VAL       1500
#define MAX_CONTROL_VAL       2000
#define EPA_POSITIVE          500
#define EPA_NEGATIVE         -500

// Free pins
// Pin                    0
// Pin                    1
// Pin                    2
// Pin                    3
// Pin                    5
// Pin                    7
// Pin                    8
// Pin                    A5
// Pin                    A6

// Pins for pots, joysticks
// Pot1                   A0
// Pot2                   A1
// Pot3                   A2
// Pot4                   A3
// Pot5                   A4

// LED alarm
#define PIN_LED           6

// Calibration button
#define PIN_BUTTON_CALIB  4

// Input battery
#define PIN_BATTERY       A7

// Pins for nRF24L01
#define PIN_CE            9
#define PIN_CSN           10

// Hardware SPI
//----- MOSI              11
//----- MISO              12
//----- SCK               13

// Setting of CE and CSN pins
RF24 radio(PIN_CE, PIN_CSN);

//*********************************************************************************************************************
// This structure defines the data sent (max 32 bytes)
//*********************************************************************************************************************
struct rc_packet_size
{
  unsigned int ch1;
  unsigned int ch2;
  unsigned int ch3;
  unsigned int ch4;
  unsigned int ch5;
};
rc_packet_size rc_packet;

//*********************************************************************************************************************
// This structure defines the received ACK payload data
//*********************************************************************************************************************
struct telemetry_packet_size
{
  byte rssi;    // Not used yet
  byte batt_A1 = 255;
  byte batt_A2; // Not used yet
};
telemetry_packet_size telemetry_packet;

//*********************************************************************************************************************
// Read pots, joysticks
//*********************************************************************************************************************
int ch, raw_pots;
int pot_calib_min[] = {0, 0, 0, 0, 0};
int pot_calib_mid[] = {512, 512, 512, 512, 512};
int pot_calib_max[] = {1023, 1023, 1023, 1023, 1023};
int pots_value[] = {1500, 1500, 1500, 1500, 1500};
byte reverse[] = {0, 0, 0, 0, 0};

void read_pots()
{
  for (ch = 0; ch < 5; ch++)
  {
    raw_pots = analogRead(ch);

    if (raw_pots > pot_calib_mid[ch])
    pots_value[ch] = map(raw_pots, pot_calib_mid[ch], pot_calib_min[ch], 0, EPA_POSITIVE);
    else
    pots_value[ch] = map(raw_pots, pot_calib_max[ch], pot_calib_mid[ch], EPA_NEGATIVE, 0);
  }
  
  // Format the frame
  for (ch = 0; ch < 5; ch++)
  {
    pots_value[ch] += MID_CONTROL_VAL;
    pots_value[ch] = constrain(pots_value[ch], MIN_CONTROL_VAL, MAX_CONTROL_VAL);

    if (reverse[ch] == 1) pots_value[ch] = 3000 - pots_value[ch];
  }
  
  rc_packet.ch1 = pots_value[0]; // A0
  rc_packet.ch2 = pots_value[1]; // A1
  rc_packet.ch3 = pots_value[2]; // A2
  rc_packet.ch4 = pots_value[3]; // A3
  rc_packet.ch5 = pots_value[4]; // A4
  
  //Serial.println(rc_packet.ch1);
}

//*********************************************************************************************************************
// Calibrate pots, joysticks
//*********************************************************************************************************************
int calibrated = 1;

void calibrate_pots()
{
  while (digitalRead(PIN_BUTTON_CALIB) == 0)
  {
    calibrated = 0;

    for (int pot = 0; pot < 5; ++pot)
    {
      raw_pots = analogRead(pot);
      if (raw_pots > pot_calib_min[pot]) pot_calib_min[pot] = raw_pots;
      if (raw_pots < pot_calib_max[pot]) pot_calib_max[pot] = raw_pots;
      pot_calib_mid[pot] = raw_pots; // Save neutral pots, joysticks as button is released
    }
  } // Calibrate button released
  
  if (calibrated == 0)
  {
    for (ch = 0; ch < 5; ch++)
    {
      EEPROMWriteInt(ch * 6,     pot_calib_max[ch]); // EEPROM locations  0,  6, 12, 18 (decimal)
      EEPROMWriteInt(ch * 6 + 2, pot_calib_mid[ch]); // EEPROM locations  2,  8, 14, 20 (decimal)
      EEPROMWriteInt(ch * 6 + 4, pot_calib_min[ch]); // EEPROM locations  4, 10, 16, 22 (decimal)
    }
    calibrated = 1;
  }
  
  for (ch = 0; ch < 5; ch++)
  {
    pot_calib_max[ch] = EEPROMReadInt(ch * 6);     // EEPROM locations  0,  6, 12, 18 (decimal)
    pot_calib_mid[ch] = EEPROMReadInt(ch * 6 + 2); // EEPROM locations  2,  8, 14, 20 (decimal)
    pot_calib_min[ch] = EEPROMReadInt(ch * 6 + 4); // EEPROM locations  4, 10, 16, 22 (decimal)
    reverse[ch] = EEPROM.read(ch + 30) & 1;        // EEPROM locations 30, 31, 32, 33 (decimal), ch * 6 = 30
  }
  
  // Check for reversing, stick over on power-up
  for (ch = 0; ch < 5; ch++)
  {
    pots_value[ch] = map(analogRead(ch), pot_calib_max[ch], pot_calib_min[ch], EPA_NEGATIVE, EPA_POSITIVE);
    if (pots_value[ch] > EPA_POSITIVE - 50 || pots_value[ch] < EPA_NEGATIVE + 50)
    {
      reverse[ch] ^= B00000001;
      EEPROM.write(30 + ch, reverse[ch]); // ch * 6 = 30
    }
  }
}

//*********************************************************************************************************************
// This function will write a 2 byte integer to the eeprom at the specified address and address + 1
//*********************************************************************************************************************
void EEPROMWriteInt(int p_address, int p_value)
{
  byte lowByte = p_value % 256;
  byte highByte = p_value / 256;
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

//*********************************************************************************************************************
// This function will read a 2 byte integer from the eeprom at the specified address and address + 1
//*********************************************************************************************************************
unsigned int EEPROMReadInt(int p_address)
{
  byte lowByte = EEPROM.read(p_address);
  byte highByte = EEPROM.read(p_address + 1);
  return lowByte + highByte * 256;
}

//*********************************************************************************************************************
// Program setup
//*********************************************************************************************************************
void setup()
{
  //Serial.begin(9600);
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BATTERY, INPUT);
  pinMode(PIN_BUTTON_CALIB, INPUT_PULLUP);
  
  calibrate_pots();
  
  // Define the radio communication
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setRetries(5, 0);
  radio.setChannel(RADIO_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MIN); // RF24_PA_MIN (-18dBm), RF24_PA_LOW (-12dBm), RF24_PA_HIGH (-6dbm), RF24_PA_MAX (0dBm)
  radio.stopListening();
  radio.openWritingPipe(address);
}

//*********************************************************************************************************************
// Program loop
//*********************************************************************************************************************
void loop()
{
  read_pots();
  send_and_receive_data();
  TX_batt_monitoring();
  RX_batt_monitoring();
  LED_mode();
}

//*********************************************************************************************************************
// Send and receive data
//*********************************************************************************************************************
unsigned long rf_timeout = 0;

void send_and_receive_data()
{
  if (radio.write(&rc_packet, sizeof(rc_packet_size)))
  {
    if (radio.available())
    {
      radio.read(&telemetry_packet, sizeof(telemetry_packet_size));
      
      rf_timeout = millis();
    }
  }
}

//*********************************************************************************************************************
// TX battery voltage monitoring
//*********************************************************************************************************************
bool tx_low_batt = 0;

void TX_batt_monitoring()
{
  tx_low_batt = analogRead(PIN_BATTERY) <= (1023 / TX_BATTERY_VOLTAGE) * TX_MONITORED_VOLTAGE;
  
  //Serial.println(tx_low_batt);
}

//*********************************************************************************************************************
// RX battery voltage monitoring
//*********************************************************************************************************************
bool rx_low_batt = 0;
bool previous_state_batt;

void RX_batt_monitoring()
{
  rx_low_batt = telemetry_packet.batt_A1 <= (255 / RX_BATTERY_VOLTAGE) * RX_MONITORED_VOLTAGE;
  
  // Battery alarm lock
  if (rx_low_batt)
  {
    previous_state_batt = 1;
  }
  rx_low_batt = previous_state_batt;
  
  //Serial.println(telemetry_packet.batt_A1);
}

//*********************************************************************************************************************
// LED blink mode
//*********************************************************************************************************************
void LED_mode()
{
  if (millis() - rf_timeout > 1000) // If we lose RF data for 1 second, the LED blink at 0.1s interval
  {
    blink(PIN_LED, 100);
  }
  else if (rx_low_batt) // If the RX battery is low, the LED blink at 0.3s interval
  {
    blink(PIN_LED, 300);
  }
  else if (tx_low_batt) // If the TX battery is low, the LED blink at 0.5s interval
  {
    blink(PIN_LED, 500);
  }
  else
  {
    digitalWrite(PIN_LED, HIGH); // Normal mode, LED is lit
  }
}

//*********************************************************************************************************************
// LED blink function
//*********************************************************************************************************************
unsigned long led_time = 0;
bool led_state;

void blink(uint8_t pin, uint16_t interval)
{
  if (millis() - led_time > interval)
  {
    led_time = millis();
    
    led_state = !led_state;
    
    digitalWrite(pin, led_state);
  }
}
 
