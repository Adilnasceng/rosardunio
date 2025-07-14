/* *************************************************************
   Arduino Mega 2560 için Encoder definitions
   
   Harici interrupt pinleri kullanılarak quadrature encoder okuma
   ************************************************************ */
   
#ifdef USE_BASE

#ifdef ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "MegaEncoderCounter.h"

  /* Create the encoder shield object */
  MegaEncoderCounter encoders = MegaEncoderCounter(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return encoders.YAxisGetCount();
    else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#elif defined(ARDUINO_ENC_COUNTER)
  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;
  
  // Encoder durumları için lookup table
  static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
  
  // Sol encoder için durum değişkenleri
  volatile uint8_t left_enc_last = 0;
  
  // Sağ encoder için durum değişkenleri  
  volatile uint8_t right_enc_last = 0;
    
  /* Sol encoder A pini için interrupt (INT0 - Pin 2) */
  void leftEncoderAInterrupt() {
    left_enc_last <<= 2; // Önceki durumu 2 bit sola kaydır
    left_enc_last |= (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
    left_enc_pos += ENC_STATES[(left_enc_last & 0x0f)];
  }
  
  /* Sol encoder B pini için interrupt (INT1 - Pin 3) */
  void leftEncoderBInterrupt() {
    left_enc_last <<= 2; // Önceki durumu 2 bit sola kaydır
    left_enc_last |= (digitalRead(LEFT_ENC_PIN_A) << 1) | digitalRead(LEFT_ENC_PIN_B);
    left_enc_pos += ENC_STATES[(left_enc_last & 0x0f)];
  }
  
  /* Sağ encoder A pini için interrupt (INT5 - Pin 18) */
  void rightEncoderAInterrupt() {
    right_enc_last <<= 2; // Önceki durumu 2 bit sola kaydır
    right_enc_last |= (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
    right_enc_pos += ENC_STATES[(right_enc_last & 0x0f)];
  }
  
  /* Sağ encoder B pini için interrupt (INT4 - Pin 19) */
  void rightEncoderBInterrupt() {
    right_enc_last <<= 2; // Önceki durumu 2 bit sola kaydır
    right_enc_last |= (digitalRead(RIGHT_ENC_PIN_A) << 1) | digitalRead(RIGHT_ENC_PIN_B);
    right_enc_pos += ENC_STATES[(right_enc_last & 0x0f)];
  }
  
  /* Encoder okuma fonksiyonu */
  long readEncoder(int i) {
    if (i == LEFT) return left_enc_pos;
    else return right_enc_pos;
  }

  /* Encoder sıfırlama fonksiyonu */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos = 0L;
      return;
    } else { 
      right_enc_pos = 0L;
      return;
    }
  }
#else
  #error A encoder driver must be selected!
#endif

/* Her iki encoder'ı sıfırlama fonksiyonu */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif
