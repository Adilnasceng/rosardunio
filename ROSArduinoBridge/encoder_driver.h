/* *************************************************************
   Encoder driver function definitions - Arduino Mega 2560 için
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  // Arduino Mega 2560 için pin tanımları
  // Sol encoder - Digital pin 2 ve 3 (INT0 ve INT1)
  #define LEFT_ENC_PIN_A 2   // Pin 2 - INT0
  #define LEFT_ENC_PIN_B 3   // Pin 3 - INT1
  
  // Sağ encoder - Digital pin 18 ve 19 (INT5 ve INT4) 
  #define RIGHT_ENC_PIN_A 18  // Pin 18 - INT5
  #define RIGHT_ENC_PIN_B 19  // Pin 19 - INT4
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
