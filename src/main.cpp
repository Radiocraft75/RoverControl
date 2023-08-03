//  Used HardwareSerial
//  UART Port   Rx        Tx      Useable
//  UART0       GPIO3     GPIO1   Yes
//  UART1       GPIO9     GPIO10  Yes but requires the reassignment of pins
//  UART2       GPIO16    GPIO17  Yes

#include <Arduino.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         9600        // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           50        // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // [-] Maximum speed for testing
#define SPEED_STEP          10          // [-] Speed step
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define LED_BUILTIN 2

#define SSID "KVANT_24"
#define WifiPass "75367536"

IPAddress local_IP(192, 168, 0, 40);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

BluetoothSerial SerialBT;
HardwareSerial SerialPort1(1);
#define HoverSerial_1 SerialPort1
HardwareSerial SerialPort2(2);
#define HoverSerial_2 SerialPort2

char vcmd;
int maxspeed = 200;
int lSpeed = 0;
int rSpeed = 0;
bool demo = false;

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// Вперед
void vforward() {
  lSpeed = maxspeed* -1;
  rSpeed = maxspeed;
}

// Назад
void vbackward() {
  lSpeed = maxspeed;
  rSpeed = maxspeed * -1;
}

// Влево
void vleft() {
  lSpeed = maxspeed * -1;
  rSpeed = maxspeed * -1;
}

// Вправо
void vright() {
  lSpeed = maxspeed;
  rSpeed = maxspeed;
}

// Стоп
void vrelease() {
  lSpeed = 0;
  rSpeed = 0;
}

void vspeed(int spd) {
  maxspeed = spd;
}

// ########################## SEND ##########################
void Send_1(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial_1.write((uint8_t *) &Command, sizeof(Command));
}

void Send_2(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial_2.write((uint8_t *) &Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive_1()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial_1.available()) {
    incomingByte    = HoverSerial_1.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p       = (byte *)&NewFeedback;
    *p++    = incomingBytePrev;
    *p++    = incomingByte;
    idx     = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++    = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}


void Receive_2()
{
  // Check for new data availability in the Serial buffer
  if (HoverSerial_2.available()) {
    incomingByte     = HoverSerial_2.read();                                   // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
  }
  else {
    return;
  }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME) {                     // Initialize if new data is detected
    p       = (byte *)&NewFeedback;
    *p++    = incomingBytePrev;
    *p++    = incomingByte;
    idx     = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
    *p++    = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback)) {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                          ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

      // Print data to built-in Serial
      Serial.print("1: ");   Serial.print(Feedback.cmd1);
      Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
      Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
      Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
      Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
      Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
      Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
    } else {
      Serial.println("Non-valid data skipped");
    }
    idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  SerialBT.begin("RoverControl"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  HoverSerial_1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 4, 2);
  HoverSerial_2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 16, 17); 
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  Serial.print("Connecting to ");
  Serial.println(SSID);

  WiFi.begin(SSID, WifiPass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("ESP Mac Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Subnet Mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS: ");
  Serial.println(WiFi.dnsIP());

  // Port defaults to 3232
  ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Rover");
  ArduinoOTA.begin();

  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  digitalWrite(26, HIGH);
  digitalWrite(27, HIGH);
}

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void loop(void) {
  ArduinoOTA.handle();
  // Если есть данные
  if (SerialBT.available()) {
    // Читаем команды и заносим их в переменную. char преобразует код символа команды в символ
    vcmd = (char)SerialBT.read();
    // Отправляем команду в порт, чтобы можно было их проверить в «Мониторе порта»
    Serial.println(vcmd);

    // Вперед
    if (vcmd == 'F') {
      vforward();
    }
    // Назад
    if (vcmd == 'B') {
      vbackward();
    }
    // Влево
    if (vcmd == 'L')
    {
      vleft();
    }
    // Вправо
    if (vcmd == 'R')
    {
      vright();
    }
    // Стоп
    if (vcmd == 'S')
    {
      vrelease();
    }

    // Включение драйверов
    if (vcmd == 'W') {
      digitalWrite(26, LOW);
      digitalWrite(27, LOW);
      delay(50);
      digitalWrite(26, HIGH);
      digitalWrite(27, HIGH);
    }
    // Выключение драйверов
    if (vcmd == 'w') {
      digitalWrite(26, LOW);
      digitalWrite(27, LOW);
      delay(300);
      digitalWrite(26, HIGH);
      digitalWrite(27, HIGH);
    }

    // Подсветка 12
    if (vcmd == 'U') {}
    // Подсветка 12
    if (vcmd == 'u') {}

    // Активация демо режима
    if (vcmd == 'X') {
      demo = true;
    }
    // Деактивация демо режима
    if (vcmd == 'x')
    {
      demo = false;
    }
    // Скорость 0%
    if (vcmd == '0')
    {
      vspeed(0);
    }
    // Скорость 10%
    if (vcmd == '1')
    {
      vspeed(25);
    }
    // Скорость 20%
    if (vcmd == '2')
    {
      vspeed(50);
    }
    // Скорость 30%
    if (vcmd == '3')
    {
      vspeed(75);
    }
    // Скорость 40%
    if (vcmd == '4')
    {
      vspeed(100);
    }
    // Скорость 50%
    if (vcmd == '5')
    {
      vspeed(125);
    }
    // Скорость 60%
    if (vcmd == '6')
    {
      vspeed(150);
    }
    // Скорость 70%
    if (vcmd == '7')
    {
      vspeed(175);
    }
    // Скорость 80%
    if (vcmd == '8')
    {
      vspeed(200);
    }
    // Скорость 90%
    if (vcmd == '9')
    {
      vspeed(250);
    }
    // Скорость 100%
    if (vcmd == 'q')
    {
      vspeed(300);
    }
  }
  unsigned long timeNow = millis();

  // Check for new received data
  Receive_1();
  Receive_2();

  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  iTest = 10;
  if (demo) {
    Send_1(0, 10);
    Send_2(0, 10);
  }
  else {
    Send_1(0, lSpeed);
    Send_2(0, rSpeed);
  }

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);
}
