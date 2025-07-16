#include <HardwareSerial.h>

#define RX_PIN 16
#define TX_PIN 17

const int timeOut = 50; // in ms

HardwareSerial SerialUART(2);

const char gngga[] = "$GNGGA,072446.00,3130.5226316,N,12024.0937010,E,4,27,0.5,31.924,M,0.000,M,2.0,*44";
const char gngga2[] = "$GNGGA,072446.00,3130.5526316,N,12024.1237010,E,4,27,0.5,41.924,M,0.000,M,2.0,*44";

hw_timer_t *timer = NULL;
portMUX_TYPE timeMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool sendFlag = false;
bool even = false;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timeMux);
  sendFlag = true;
  portEXIT_CRITICAL_ISR(&timeMux);
}

void setup() {
  Serial.begin(9600);
  SerialUART.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, timeOut*1000, true, 0);
}

void loop() {
  if (sendFlag) {
    portENTER_CRITICAL(&timeMux);
    sendFlag = false;
    portEXIT_CRITICAL(&timeMux);
    even = !even;
    const uint8_t* message = even? (const uint8_t*)gngga : (const uint8_t*)gngga2;
    SerialUART.write(message, 82);
  }
}
