/*********************************************************************
 *  ROSArduinoBridge - DFPlayer Mini ses sistemi ile güncellenmiş versiyon
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   #define ARDUINO_ENC_COUNTER
   #define BTS7960_MOTOR_DRIVER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

/* Buzzer pin definition */
#define BUZZER_PIN     8   // Buzzer bağlı olduğu pin

/* Buzzer pattern configuration */
#define BEEP_ON_TIME   200   // Beep süresi (ms)
#define BEEP_OFF_TIME  300   // Beep arası bekleme süresi (ms)

/* DFPlayer Mini configuration */
#define USE_DFPLAYER   // DFPlayer Mini kullanımını etkinleştir
#define DFPLAYER_RX_PIN 12  // Arduino pin 12 -> DFPlayer TX
#define DFPLAYER_TX_PIN 13  // Arduino pin 13 -> DFPlayer RX

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* DFPlayer Mini kütüphaneleri */
#ifdef USE_DFPLAYER
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
SoftwareSerial dfSerial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
DFRobotDFPlayerMini dfplayer;
bool dfplayer_ready = false;
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* Variable initialization */
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Buzzer control variables */
bool buzzer_enabled = false;      // Buzzer etkin mi?
bool buzzer_state = false;        // Anlık buzzer durumu (HIGH/LOW)
unsigned long last_beep_time = 0; // Son beep zamanı

/* DFPlayer fonksiyonları */
#ifdef USE_DFPLAYER
void initDFPlayer() {
  dfSerial.begin(9600);
  Serial.println(F("DFPlayer başlatılıyor..."));
  
  if (!dfplayer.begin(dfSerial)) {
    Serial.println(F("DFPlayer başlatılamadı! Bağlantıları kontrol et."));
    dfplayer_ready = false;
    return;
  }
  
  Serial.println(F("DFPlayer hazır."));
  dfplayer.volume(10); // 0-30 arası ses seviyesi
  delay(200);
  dfplayer_ready = true;
}

void playSound(int sound_number) {
  if (!dfplayer_ready) {
    Serial.println(F("DFPlayer hazır değil!"));
    return;
  }
  
  if (sound_number >= 1 && sound_number <= 2) {
    dfplayer.play(sound_number);
    Serial.print(F("Ses çalınıyor: "));
    Serial.println(sound_number);
  } else {
    Serial.print(F("Geçersiz ses numarası: "));
    Serial.println(sound_number);
  }
}

void checkDFPlayerStatus() {
  if (!dfplayer_ready) return;
  
  if (dfplayer.available()) {
    int type = dfplayer.readType();
    int value = dfplayer.read();
    
    if (type == DFPlayerPlayFinished) {
      Serial.print(F("Ses çalma tamamlandı: "));
      Serial.println(value);
    }
    else if (type == DFPlayerError) {
      Serial.print(F("DFPlayer hatası: "));
      Serial.println(value);
    }
  }
}
#endif

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Buzzer pattern kontrolü */
void updateBuzzer() {
  if (!buzzer_enabled) {
    // Buzzer devre dışı - kapalı tut
    if (buzzer_state) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
    }
    return;
  }
  
  // Buzzer etkin - pattern çal
  unsigned long current_time = millis();
  
  if (buzzer_state) {
    // Buzzer çalıyor - bekleme süresini kontrol et
    if (current_time - last_beep_time >= BEEP_ON_TIME) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
      last_beep_time = current_time;
    }
  } else {
    // Buzzer kapalı - yeni beep zamanı mı?
    if (current_time - last_beep_time >= BEEP_OFF_TIME) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzer_state = true;
      last_beep_time = current_time;
    }
  }
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case BUZZER_CONTROL:
    // Buzzer kontrolü: "b 1" etkinleştir, "b 0" devre dışı bırak
    buzzer_enabled = (arg1 == 1);
    if (!buzzer_enabled) {
      // Buzzer devre dışı bırakılırsa hemen kapat
      digitalWrite(BUZZER_PIN, LOW);
      buzzer_state = false;
    }
    Serial.println("OK");
    break;
  case SOUND_CONTROL:
    // Ses kontrolü: "s 1" ses 1'i çal, "s 2" ses 2'yi çal
    #ifdef USE_DFPLAYER
    playSound(arg1);
    #else
    Serial.println("DFPlayer devre dışı");
    #endif
    Serial.println("OK");
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      resetPID();
      moving = 0;
    }
    else moving = 1;
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.println("OK"); 
    break;
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    resetPID();
    moving = 0; // Sneaky way to temporarily disable the PID
    setMotorSpeeds(arg1, arg2);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);

  // Buzzer pin'ini output olarak ayarla ve başlangıçta kapat
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzer_enabled = false;
  buzzer_state = false;
  last_beep_time = millis();

  // DFPlayer Mini'yi başlat
  #ifdef USE_DFPLAYER
  initDFPlayer();
  #endif

// Motor controller ve encoder başlatma
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    // Arduino Mega 2560 için encoder pin konfigürasyonu
    pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);  // Pin 2
    pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);  // Pin 3
    pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP); // Pin 18
    pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP); // Pin 19
    
    // Interrupt'ları bağla
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderAInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderBInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderAInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderBInterrupt, CHANGE);
  #endif
  
  initMotorController();
  resetPID();
#endif

/* Servo bağlantıları */
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(
        servoPins[i],
        stepDelay[i],
        servoInitPosition[i]);
  }
#endif

  Serial.println("Arduino ROSBridge hazır - DFPlayer destekli versiyon");
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  // Buzzer pattern'ini güncelle
  updateBuzzer();
  
  // DFPlayer durumunu kontrol et
  #ifdef USE_DFPLAYER
  checkDFPlayerStatus();
  #endif
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
