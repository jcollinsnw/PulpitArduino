#include <SoftwareSerial.h>  
#include <SerialCommand.h>
#include <AFMotor.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

/*
MOTOR34_64KHZ
MOTOR34_8KHZ
MOTOR34_1KHZ
*/

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR12_64KHZ);
AF_DCMotor motor4(4, MOTOR12_64KHZ);

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

SerialCommand SCmd;   // The demo SerialCommand object

String lcdLine1 = "                ";
String lcdLine2 = "                ";

void setup()
{
  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight

  Serial.begin(9600); 

  // Setup callbacks for SerialCommand commands 
  SCmd.addCommand("M",Motor); // M {<id>} MOTOR{1-4} SPEED{0-255} DIRECTION{1|0|-1}
  SCmd.addCommand("MF",MotorFreq); // MF {<id>} MOTOR{1-4} PWM{64|8|1}
  SCmd.addCommand("LW", LCD_Write); // LW {<id>} HPOS{0-15} LINE{0|1} MESSAGE{<string_no_spaces>}
  SCmd.addCommand("LC", LCD_Clear); // LC {<id>}
  SCmd.addCommand("LB", LCD_Backlight); // LB {<id>} STATE{0|1}
  SCmd.addCommand("AR", Analog_Read); // AR {<id>} PIN{#}
  SCmd.addCommand("AW", Analog_Write); // AW {<id>} PIN{#} VAL{0-255}
  SCmd.addCommand("DR", Digital_Read); // DR {<id>} PIN{#}
  SCmd.addCommand("DW", Digital_Write); // DW {<id>} PIN{#} VAL{0|1}
  SCmd.addCommand("PM", Pin_Mode); // PM {<id>} PIN{#} MODE{0|1}
  SCmd.addCommand("E", Echo); // E {<id>} MESSAGE{<string_no_spaces>}
  
  SCmd.setDefaultHandler(unrecognized);
  
  Serial.println("R 0 READY"); 
  lcd.setCursor(0,0);
  lcd.print("Pulpit v1.0");
  lcd.setCursor(0,1);
  lcd.print("READY");
}

void loop()
{  
  SCmd.readSerial();     // We don't do much, just process serial commands
}

void Echo() {
  String id;
  char* arg;
  String response = "R E ";
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    response += arg;
  }
  
  //RETURN
  Serial.println(response);
}

void Motor()
{
  // M {<id>} {1-4} {0-255} {1|0|-1}
  String id;
  int motorNumber;
  int spd;
  int dir;
  char *arg;
  String response = "R M ";

  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    motorNumber = atoi(arg);
    response += motorNumber;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    spd = atoi(arg);
    response += spd;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    dir = atoi(arg);
    response += dir;
    response += " ";
  }
  if( dir == 1 ) {
    dir = FORWARD;
  }
  if( dir == 0 ) {
    dir = RELEASE;
  }
  if( dir == 2 ) {
    dir == BACKWARD;
  }
  
  if( motorNumber == 1 ) {
    motor1.setSpeed(spd);
    motor1.run(dir);
  }
  if( motorNumber == 2 ) {
    motor2.setSpeed(spd);
    motor2.run(dir);
  }
  if( motorNumber == 3 ) {
    motor3.setSpeed(spd);
    motor3.run(dir);
  }
  if( motorNumber == 4 ) {
    motor4.setSpeed(spd);
    motor4.run(dir);
  }
  
  //RETURN
  Serial.println(response);
}

void MotorFreq()
{
  // MF {<id>} {1-4} {64|8|1}
  String id;
  int motorNumber;
  int freq;
  char *arg;
  String response = "R MF ";

  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    motorNumber = atoi(arg);
    response += motorNumber;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    freq = atoi(arg);
    response += freq;
    response += " ";
  }
  
  if( motorNumber == 1 || motorNumber == 2 ) {
    if( freq == 64 ) {
      freq = MOTOR12_64KHZ;
    }
    if( freq == 8 ) {
      freq = MOTOR12_8KHZ;
    }
    if( freq == 1 ) {
      freq = MOTOR12_1KHZ;
    }
  }
  if( motorNumber == 2 || motorNumber == 3 ) {
    if( freq == 64 ) {
      freq = MOTOR34_64KHZ;
    }
    if( freq == 8 ) {
      freq = MOTOR34_8KHZ;
    }
    if( freq == 1 ) {
      freq = MOTOR34_1KHZ;
    }
  }
  
  if( motorNumber == 1 ) {
    motor1.initPWM(freq); 
  }
  if( motorNumber == 2 ) {
    motor2.initPWM(freq); 
  }
  if( motorNumber == 3 ) {
    motor3.initPWM(freq); 
  }
  if( motorNumber == 4 ) {
    motor4.initPWM(freq); 
  }
  
  //RETURN
  Serial.println(response);
}

void LCD_Write() {
  // LW {<id>} {0-15} {0|1} {<string_no_spaces>}
  String id;
  int hpos;
  int line;
  String message;
  char *arg;
  String response = "R LW ";
  
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    hpos = atoi(arg);
    response += hpos;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    line = atoi(arg);
    response += line;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    message = String(arg);
    response += message;
    response += " ";
    message.replace("_", " ");
  }
  
  lcd.setCursor(hpos, line);
  lcd.print(message);
  
  //RETURN
  Serial.println(response);
}

void LCD_Read() {
}

void LCD_Clear() {
  // LC {<id>}
  String id;
  char *arg;
  String response = "R LC ";
  
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
  }
  
  lcd.clear();
  
  //RETURN
  Serial.println(response);
}

void LCD_Backlight() {
  // LB {<id>} {0|1}
  String id;
  int state;
  char *arg;
  String response = "R LB ";
  
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += (" ");
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    state = atoi(arg);
  }
  
  if( state == 1 ) {
    lcd.backlight();
  }
  if( state == 0 ) {
    lcd.noBacklight();
  }
  response += arg;
  
  //RETURN
  Serial.println(response);
}

void Analog_Read() {
  // AR {<id>} PIN{#}
  String id;
  int pin;
  char *arg;
  String response = "R AR ";
  
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    pin = atoi(arg);
  }
  int val = analogRead(pin);
  response += val;
  
  //RETURN
  Serial.println(response);
}

void Analog_Write() {
  // AW {<id>} PIN{#} VAL{0-255}
  String id;
  int pin;
  int val;
  char *arg;
  String response = "R AW ";
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    pin = atoi(arg);
    response += pin;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    val = atoi(arg);
    response += val;
    response += " ";
  }
  
  analogWrite(pin, val);
  
  //RETURN
  Serial.println(response);
}

void Digital_Read() {
  // DR {<id>} {0-5}
  String id;
  int pin;
  char *arg;
  String response = "R DR ";
  
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    pin = atoi(arg);
  }
  int val = digitalRead(pin);
  response += val;
  
  //RETURN
  Serial.println(response);
}

void Digital_Write() {
  String id;
  int pin;
  int val;
  char *arg;
  String response = "R DW ";
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    pin = atoi(arg);
    response += pin;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    val = atoi(arg);
    response += val;
  }
  
  if( val == 0 ) {
    digitalWrite(pin, LOW);
  }
  if( val == 1 ) {
    digitalWrite(pin, HIGH);
  }
  
  //RETURN
  Serial.println(response);
}

void Pin_Mode() {
  String id;
  int pin;
  int  mode;
  char *arg;
  String response = "R PM ";
  
  arg = SCmd.next();
  if( arg != NULL ) {
    id = String(arg);
    response += id;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    pin = atoi(arg);
    response += pin;
    response += " ";
  }
  arg = SCmd.next();
  if( arg != NULL ) {
    mode = atoi(arg);
    response += mode;
  }
  
  if( mode == 0 ) {
    pinMode(pin, INPUT);
  }
  if( mode == 1 ) {
    pinMode(pin, OUTPUT);
  }
  
  //RETURN
  Serial.println(response);
}

void unrecognized(const char *command) {
  Serial.println(command);
}
