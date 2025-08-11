/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define BUZZER_CONTROL 'b'  // Buzzer kontrolü
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SOUND_CONTROL  's'  // DFPlayer ses kontrolü (eski SERVO_WRITE değiştirildi)
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define GET_BAUDRATE   'z'  // Baudrate komutunu 'z' ye değiştirdik

#define LEFT            0
#define RIGHT           1

#endif
