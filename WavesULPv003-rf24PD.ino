#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

//Forbruget flader fra 70mA til 20mA når DHT fjernes
//tilføjer RF24 lib for powerDown() - bemærk ændrede pins!!

//importerer libraries
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "SPI.h"  // SPI in Arduino Uno/Nano: MOSI pin 11, MISO pin 12, SCK pin 13
#include "DHT.h"
//definerer pins
#define PIN_CE 7 //chipenabler - kan den kaldes on the fly og disables under sleep??
#define PIN_CSN 8 //chipselector - kan hele SPI disables og initialiseres under metodekald til send når data er indlæst fra DHT?
#define DHTPIN 4 //måske dårligt kald at bruge 4 - 10K resistor fra DHTPIN til PIN_VCC_DHT
//definer VCC pins - tjek om der er problemer med impedansen på de loadede pins
#define PIN_VCC_DHT 5
#define PIN_VCC_NRF 6
//definer typer
//#define DHTTYPE DHT22
//definer mac - gemmes som byte i flash - bør ikke bruge strøm
#define MAC_0 0x11
#define MAC_1 0x11
#define MAC_2 0x11
#define MAC_3 0x11
#define MAC_4 0x11
#define MAC_5 0x11
//gem sensortype som variabel
DHT dht(DHTPIN, DHT22,3);
//bufferstørrelse, CRC pacering af bits
uint8_t buf[32];
static const uint8_t chRf[] = {2, 26, 80};
static const uint8_t chLe[] = {37, 38, 39};
uint8_t ch = 0;
//caster navn
char n1 = 'n';

//laver metoder før setup - skal bare finde måde at omgå sensor library ikke har powerdown()
int readTemp(int tempVar) {
  //dht.begin();
  pinMode(DHTPIN,INPUT);
  pinMode(PIN_VCC_DHT, OUTPUT);
  digitalWrite(PIN_VCC_DHT, HIGH);
  delay(100);
  tempVar = (int)dht.readTemperature();
  //dht.stop();
  digitalWrite(PIN_VCC_DHT, LOW);
  pinMode(PIN_VCC_DHT, INPUT_PULLUP);
  pinMode(DHTPIN, INPUT_PULLUP);
  return tempVar;
}
int readHumi(int humiVar) {
  //dht.begin();
  pinMode(DHTPIN,INPUT);
  pinMode(PIN_VCC_DHT, OUTPUT);
  digitalWrite(PIN_VCC_DHT, HIGH);
  delay(100);
  humiVar = (int)dht.readHumidity();;
  //dht.stop();
  digitalWrite(PIN_VCC_DHT, LOW);
  pinMode(PIN_VCC_DHT, INPUT_PULLUP);
  pinMode(DHTPIN, INPUT_PULLUP);
  return humiVar;
}

//konstruer CRC for BLE - skriv de enkelte bits ind på fastlagte adresser
void btLeCrc(const uint8_t* data, uint8_t len, uint8_t* dst) {
  uint8_t v, t, d;

  while (len--) {
    d = *data++;
    for (v = 0; v < 8; v++, d >>= 1) {
      t = dst[0] >> 7;
      dst[0] <<= 1;
      if (dst[1] & 0x80) dst[0] |= 1;
      dst[1] <<= 1;
      if (dst[2] & 0x80) dst[1] |= 1;
      dst[2] <<= 1;

      if (t != (d & 1)) {
        dst[2] ^= 0x5B;
        dst[1] ^= 0x06;
      }
    }
  }
}

//metode til at swappe bits - læses baglæns, skrives forlæns
uint8_t  swapbits(uint8_t a) {
  uint8_t v = 0;
  if (a & 0x80) v |= 0x01;
  if (a & 0x40) v |= 0x02;
  if (a & 0x20) v |= 0x04;
  if (a & 0x10) v |= 0x08;
  if (a & 0x08) v |= 0x10;
  if (a & 0x04) v |= 0x20;
  if (a & 0x02) v |= 0x40;
  if (a & 0x01) v |= 0x80;
  return v;
}

void btLeWhiten(uint8_t* data, uint8_t len, uint8_t whitenCoeff) {
  // Implementing whitening with LFSR
  uint8_t  m;
  while (len--) {
    for (m = 1; m; m <<= 1) {
      if (whitenCoeff & 0x80) {
        whitenCoeff ^= 0x11;
        (*data) ^= m;
      }
      whitenCoeff <<= 1;
    }
    data++;
  }
}

static inline uint8_t btLeWhitenStart(uint8_t chan) {
  //the value we actually use is what BT'd use left shifted one...makes our life easier
  return swapbits(chan) | 2;
}

void btLePacketEncode(uint8_t* packet, uint8_t len, uint8_t chan) {
  // Assemble the packet to be transmitted
  // Length is of packet, including crc. pre-populate crc in packet with initial crc value!
  uint8_t i, dataLen = len - 3;
  btLeCrc(packet, dataLen, packet + dataLen);
  for (i = 0; i < 3; i++, dataLen++)
    packet[dataLen] = swapbits(packet[dataLen]);
  btLeWhiten(packet, len, btLeWhitenStart(chan));
  for (i = 0; i < len; i++)
    packet[i] = swapbits(packet[i]); // the byte order of the packet should be reversed as well

}

//skriver via SPI - caster fejl hvis impedans ikke stemmer overens på pins
uint8_t spi_byte(uint8_t byte) {
  // using Arduino's SPI library; clock out one byte
  SPI.transfer(byte);
  return byte;
}

//åbner register på chip
void nrf_cmd(uint8_t cmd, uint8_t data) {
  // Write to nRF24's register
  digitalWrite(PIN_CSN, LOW);
  spi_byte(cmd);
  spi_byte(data);
  digitalWrite(PIN_CSN, HIGH);
}


void nrf_simplebyte(uint8_t cmd) {
  // transfer only one byte
  digitalWrite(PIN_CSN, LOW);
  spi_byte(cmd);
  digitalWrite(PIN_CSN, HIGH);
}

void nrf_manybytes(uint8_t* data, uint8_t len) {
  // transfer several bytes in a row
  digitalWrite(PIN_CSN, LOW);
  do {
    spi_byte(*data++);
  } while (--len);
  digitalWrite(PIN_CSN, HIGH);
}

void initialiserNRF () {
  pinMode(PIN_CSN, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PIN_CSN, HIGH);
  digitalWrite(PIN_CE, LOW);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  //fastlægger parametre - copy/paste fra producentens hjemmeside
  // Now initialize nRF24L01+, setting general parameters
  nrf_cmd(0x20, 0x12);  //on, no crc, int on RX/TX done
  nrf_cmd(0x21, 0x00);  //no auto-acknowledge
  nrf_cmd(0x22, 0x00);  //no RX
  nrf_cmd(0x23, 0x02);  //4-byte address
  nrf_cmd(0x24, 0x00);  //no auto-retransmit
  nrf_cmd(0x26, 0x06);  //1MBps at 0dBm
  nrf_cmd(0x27, 0x3E);  //clear various flags
  nrf_cmd(0x3C, 0x00);  //no dynamic payloads
  nrf_cmd(0x3D, 0x00);  //no features
  nrf_cmd(0x31, 32);          //always RX 32 bytes
  nrf_cmd(0x22, 0x01);  //RX on pipe 0
  // Set access addresses (TX address in nRF24L01) to BLE advertising 0x8E89BED6
  // Remember that both bit and byte orders are reversed for BLE packet format
  buf[0] = 0x30;
  buf[1] = swapbits(0x8E);
  buf[2] = swapbits(0x89);
  buf[3] = swapbits(0xBE);
  buf[4] = swapbits(0xD6);
  nrf_manybytes(buf, 5);
  buf[0] = 0x2A;    // set RX address in nRF24L01, doesn't matter because RX is ignored in this case
  nrf_manybytes(buf, 5);

}

// watchdog interrupt
ISR (WDT_vect)
{
  wdt_disable();  // disable watchdog
}  // end of WDT_vect

void setup() {
  initialiserNRF();
  dht.begin();

}

void loop() {
  RF24 radio(7, 8);
  //pins to HIGH
  digitalWrite(PIN_VCC_NRF, HIGH);
  radio.powerUp();
  //SPI.begin();
  //SPI.setBitOrder(MSBFIRST);
  delay(100);


  int h = 0;
  int t = 0;

  // Channel hopping
  for (ch = 0; ch < sizeof(chRf); ch++)
  {
    uint8_t i, L = 0;

    buf[L++] = 0x42;  //PDU type, given address is random; 0x42 for Android and 0x40 for iPhone
    buf[L++] = 16 + 5; // length of payload

    buf[L++] = MAC_0;
    buf[L++] = MAC_1;
    buf[L++] = MAC_2;
    buf[L++] = MAC_3;
    buf[L++] = MAC_4;
    buf[L++] = MAC_5;

    buf[L++] = 2;   //flags (LE-only, general discoverable mode)
    buf[L++] = 0x01;
    buf[L++] = 0x06;

    buf[L++] = 6;   // length of the name, including type byte
    buf[L++] = 0x08;
    buf[L++] = n1;  //caster garateret fejl TODO
    buf[L++] = 'R';
    buf[L++] = 'F';
    buf[L++] = '2';
    buf[L++] = '5';

    buf[L++] = 4;   // length of custom data, including type byte
    buf[L++] = readHumi(h);
    buf[L++] = 0xff;
    buf[L++] = readTemp(t); //TODO metode i byteloop ....
    buf[L++] = 0xff;// some test data

    buf[L++] = 0x55;  //CRC start value: 0x555555
    buf[L++] = 0x55;
    buf[L++] = 0x55;

    nrf_cmd(0x25, chRf[ch]);
    nrf_cmd(0x27, 0x6E);  // Clear flags

    btLePacketEncode(buf, L, chLe[ch]);
    nrf_simplebyte(0xE2); //Clear RX Fifo
    nrf_simplebyte(0xE1); //Clear TX Fifo

    digitalWrite(PIN_CSN, LOW);
    spi_byte(0xA0);
    for (i = 0 ; i < L ; i++) spi_byte(buf[i]);
    digitalWrite(PIN_CSN, HIGH);

    nrf_cmd(0x20, 0x12);  // TX on
    digitalWrite(PIN_CE, HIGH); // Enable Chip
    delay(2);        //
    digitalWrite(PIN_CE, LOW);   // (in preparation of switching to RX quickly)
  }
  delay(200);    // Broadcasting interval

  //sleep NRF
  radio.powerDown();
  //  avr_enter_sleep_mode();

  //pins to Low
  digitalWrite(PIN_VCC_NRF, LOW);

  // SPI.end();
  // disable ADC
  ADCSRA = 0;

  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval
  WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog


  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
  noInterrupts ();           // timed sequence follows
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();

  // cancel sleep as a precaution
  sleep_disable();
}
