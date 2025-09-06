#ifndef SERVOS_H
#define SERVOS_H

// Servo sayısı (ihtiyacınıza göre ayarlayın)
#define N_SERVOS 1

// Servo hareket hızı ayarı (ms cinsinden step gecikmesi)
// 0 = tam hız, büyük değerler = yavaş hareket
int stepDelay [N_SERVOS] = { 20 }; // 20ms = hızlı hareket

// Servo pin tanımları
byte servoPins [N_SERVOS] = { 4 }; // Pin 4'e bağlı servo

// Başlangıç pozisyonları (derece cinsinden)
byte servoInitPosition [N_SERVOS] = { 0 }; // Başlangıçta 0 derece

class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif
