#include <HardwareSerial.h>

// Lidar 2 (to uart1)
#define U0_RX_PIN 2
#define U0_TX_PIN 15
#define BAUD_RATE_LIDAR 921600

// Lidar 1 (to uart5)
#define U1_RX_PIN 5
#define U1_TX_PIN 4

// GNSS (to uart4)
#define U2_RX_PIN 16
#define U2_TX_PIN 17
#define BAUD_RATE_GNSS 115200

const int timeOutLidar = 10; // in ms
const int timeOutGnss = 50; 

HardwareSerial SerialUART0(0);
HardwareSerial SerialUART1(1);
HardwareSerial SerialUART2(2);

enum LidarState {IDLE, SENT_VERSION, START_SCAN};
LidarState lidars = IDLE;

uint8_t readMiss = 0;

// Lidar fake message
const uint8_t versionMes[] = {0xa5, 0x03, 0x20, 0x0a, 0x00, 0x00, 0x0E, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf6, 0x06, 0x42, 0x00, 0x74, 0x26, 0x01, 0x00,
  0x0b, 0x44}; // Checksum
const uint8_t lidarMes[] = {
  0xa5, 0x03, 0x20, 0x01, 0x00, 0x00, // Start command
  0x0E, // Data length
  0xff, 0xff, 0xff, 0xff, 0xff, 0xf6, 0x06, 0x42, 0x00, 0x74, 0x26, 0x01, 0x00, // Actual data
  0x0b, 0x44 // Checksum
};

// GNSS fake messages
/* GNGGA format:
  0 (Header) 	  --> $GNGGA 
  1 (UTC time) 	--> 072446.00
  2 (Lat) 	    --> 3130.5226316
  3 (N/S) 	    --> N
  4 (Long) 	    --> 12024.0937010
  5 (E/W)		    --> E
  6 (Quality)	  --> 4
  7 (num sats)	--> 27
  8 (HDOP)	    --> 0.5
  9 (Alt.)	    --> 31.924
  10 (Units)	  --> M
  11 (Geiod. sep.)--> 0.000
  12 (Units)	  --> M
  13 (DGPS ID)	--> 2.0
  14 (Checksum)	--> *44
*/
const char gngga[] = "$GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44";
const char gngga2[] = "$GNGGA,072446.00,3130.5526316,N,12024.1237010,E,4,27,0.5,41.924,M,0.000,M,2.0,*44";

hw_timer_t *timerLidar = NULL;
portMUX_TYPE timeLidarMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timerGnss = NULL;
portMUX_TYPE timeGnssMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool sendFlagLidar = false;
volatile bool sendFlagGnss = false;
bool even = false;

void IRAM_ATTR onTimerLidar() {
  portENTER_CRITICAL_ISR(&timeLidarMux);
  sendFlagLidar = true;
  portEXIT_CRITICAL_ISR(&timeLidarMux);
}

void IRAM_ATTR onTimerGnss() {
  portENTER_CRITICAL_ISR(&timeGnssMux);
  sendFlagGnss = true;
  portEXIT_CRITICAL_ISR(&timeGnssMux);
}

void handleLidarCommand() {
  uint8_t rxBuffer[64];
  int len = SerialUART0.available();

  if(len > 0) {
    int read = SerialUART0.readBytes(rxBuffer, len);
    if(read >= 6) { 
      int i = 0;
      while((rxBuffer[i] != 0xa5 && rxBuffer[i+1] != 0x03 && rxBuffer[i+2] != 0x20) && (i+4) < read) i++;
      uint8_t cmd = rxBuffer[i+3];
      if (cmd == 0x0a || lidars == IDLE) {
        SerialUART0.write(versionMes, sizeof(versionMes));
        SerialUART1.write(versionMes, sizeof(versionMes));
        lidars = SENT_VERSION;
      }
      else if (cmd == 0x01 || lidars == SENT_VERSION) {
        SerialUART0.write(lidarMes, sizeof(lidarMes));
        SerialUART1.write(lidarMes, sizeof(lidarMes));
        lidars = START_SCAN;
      }
    }
  } else if(lidars == START_SCAN || readMiss > 20) {
      SerialUART0.write(lidarMes, sizeof(lidarMes));
      SerialUART1.write(lidarMes, sizeof(lidarMes));
  } else {
    readMiss++;
  }
}

void setup() {
  SerialUART0.begin(BAUD_RATE_LIDAR, SERIAL_8N1, U0_RX_PIN, U0_TX_PIN);
  SerialUART1.begin(BAUD_RATE_LIDAR, SERIAL_8N1, U1_RX_PIN, U1_TX_PIN);
  SerialUART2.begin(BAUD_RATE_GNSS, SERIAL_8N1, U2_RX_PIN, U2_TX_PIN);

  timerGnss = timerBegin(1000000);
  timerAttachInterrupt(timerGnss, &onTimerGnss);
  timerAlarm(timerGnss, timeOutGnss*1000, true, 0);

  timerLidar = timerBegin(1000000);
  timerAttachInterrupt(timerLidar, &onTimerLidar);
  timerAlarm(timerLidar, timeOutLidar*1000, true, 0);
}

void loop() {
  if (sendFlagGnss) {
    portENTER_CRITICAL(&timeGnssMux);
    sendFlagGnss = false;
    portEXIT_CRITICAL(&timeGnssMux);
    even = !even;
    const uint8_t* message = even? (const uint8_t*)gngga : (const uint8_t*)gngga2;
    SerialUART2.write(message, strlen(gngga));
  }

  if (sendFlagLidar) {
    portENTER_CRITICAL(&timeLidarMux);
    sendFlagLidar = false;
    portEXIT_CRITICAL(&timeLidarMux);

    handleLidarCommand();

    SerialUART0.flush();
    SerialUART1.flush();
  }
}
