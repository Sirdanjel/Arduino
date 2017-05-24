#include <EEPROM.h>
#include <avr/wdt.h>  //watchdog library
//library for IR remote decoding
#include <IRLibRecv.h>
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLibCombo.h>
IRrecv MyReceiver(10);
IRdecode MyDecoder;
//Remote control codes
long code[24] = {
  0x4F9E01F,
  0x4F928D7,
  0x4F9A05F,
  83488783,
  83429623,
  83452063,
  83439823,
  83462263,
  83447983,
  83472463,
  83445943,
  83480623,
  83456143,
  83478583,
  83464303,
  83470423,
  83454103,
  83486743,
  83433703,
  83466343,
  83450023,
  83482663,
  83441863,
  83474503
};
int pos[2][24]; //remote control array with save positions
int rbut = 0;
boolean StorePostoRBut = false;

//common variables
#define INTERUPTPIN 2
#define STEP 2
#define DATA 11
#define clock 13
#define BUTT 7
#define SENSOR1 18
#define SENSOR2 12
#define RELE1 16
#define RELE2 17
#define RELE3 8
#define RELE4 9
#define BACKSTOP1 15
#define BACKSTOP2 14
#define W 0
#define E 1
#define LED 6

boolean rot = false;
boolean Err = false;
byte sensor[2] = {SENSOR1, SENSOR2};
byte backstop[2] = {BACKSTOP1, BACKSTOP2};
byte rele1[2] = {RELE1, RELE3};
byte rele2[2] = {RELE2, RELE4};
byte direct;
int positions[2] = {0, 0};
int setposition[2] = {0, 0};
byte channel = 0;
unsigned long lastDebounceTime = 0;
unsigned long lastDebounceTime1 = 0;
unsigned long debounceDelay = 1500;


//define display variables
#define MIDDLE 6 // G              _A_
#define UPPER_L 5 // F            |   |B
#define LOWER_L 4 // E           E|_G_|
#define BOTTOM 3 // D             |   |C
#define LOWER_R 2 // C           D|_ _|
#define UPPER_R 1 // B              C
#define TOP 0 // A

int n = 0;
int ShReg_position = 0; //shift bit position in shift register
int digitPins[4] = {5, 4, 3, 19}; //pins for 7-segment multiplexing
int ON = HIGH;
int OFF = LOW;
int number[14][7]; // holds information about which segments are needed for each of the 10 numbers
int digit[4]; // holds values for each of the 4 display digits

//sensor variable
byte sensor_change = 0;
byte stepp;
byte sensor_state;
byte step_size = 0;
byte setstep_size[2] = {1, 1};

// button variable
volatile byte con = 0;
byte step1 = 0;

enum {stSensor, stStopMotorBut, stSerial, stStopMotor, stRemote, stError, stchannel, stStartMotor} //states of state machine
state = stStartMotor; //firts state

//used function
void setupTimer1(); // set register for TIMER1
void initNumber(); //set each segment to numbers
void setDigit(int n); //devide number to units and set special characters
void Sensor(); //reading information about motoro rotation from magnetic contact sensor
void Remote(); //controling device by remote control
void flash(); //save position to EEPROM after power lost
void EEPROMload(); // laoding information saved in EEPROM memory
void Error(); //determine type of error
void channell(); //changing actual channel to next one

void setup() {
  wdt_enable(WDTO_4S); //watchdog -4sek period
  //setup power loose interupt
  attachInterrupt(digitalPinToInterrupt(INTERUPTPIN), flash, RISING);
  //timer1 interrupt setup to 300ms
  setupTimer1();
  //load values save in EEPROM
  EEPROMload();
  MyReceiver.enableIRIn();//enable IR receiver
  Serial.begin(9600); // start serial comunication
  //display initialization
  shiftOut(DATA, clock, MSBFIRST, B1111111); //starting positons of shift register
  initNumber(); //set which segments are needed for each of the 10 numbers
  //setup all pins mode
  pinMode(clock, OUTPUT);
  pinMode(DATA, OUTPUT);
  //set up transistor controls 7-segments panels
  for (int i = 0; i < 4; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], LOW);
  }
  //set mode of others pins
  pinMode(BUTT, INPUT_PULLUP);
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);
  pinMode(RELE1, OUTPUT);
  pinMode(RELE2, OUTPUT);
  pinMode(RELE3, OUTPUT);
  pinMode(RELE4, OUTPUT);
  pinMode(BACKSTOP1, INPUT);
  pinMode(BACKSTOP2, INPUT);
  pinMode(LED, OUTPUT);
  //turn LED diode to green color
  digitalWrite(LED, LOW);
}

//timer1 interrupt service routine
ISR(TIMER1_COMPA_vect)
{
  interrupts(); //allow intterupt during timer1 interrupt function
  //setDigit(n);
  for (int i = 0; i < 4; i++) { //whole display off
    digitalWrite(digitPins[i], LOW);
  }
  //incremenet position of zero in shift register to 7
  ShReg_position++;
  if (ShReg_position > 6) {
    ShReg_position = 0;
  }
  shiftOut(DATA, clock, MSBFIRST, ~(1 << ShReg_position)); // shift one bit and set output of shreg

  for (int i = 0; i < 4; i++) {
    if (digit[i] < 0) {//if digit[i]=-1 off all segments
      digitalWrite(digitPins[i], LOW);
      continue;
    }
    digitalWrite(digitPins[i], number[digit[i]][ShReg_position]);   //set transistor controling dispaly on or off for specific number and specific position of active segment
  }

  //read buttoms status
  if ((ShReg_position == 0) && (digitalRead(BUTT) == LOW)) //button one is pressed W->
  {
    step1++;
    if (step1 > 4 && (ShReg_position == 0) && (digitalRead(BUTT) == LOW) && con != 2) //debaucing
    {
      con = 1;
      step1 = 0;
      direct = W;
    }

  }
  else if ((ShReg_position == 1) && (digitalRead(BUTT) == LOW))//button two is pressed <-E
  {
    step1++;
    if (step1 > 7 && (ShReg_position == 1) && (digitalRead(BUTT) == LOW) && con != 1) //debaucing
    {
      con = 2;
      step1 = 0;
      direct = E;
    }

  }
  else if ((ShReg_position == 2) && (digitalRead(BUTT) == LOW))//button CH/RESET id pressed
  {
    step1++;
    if (Err == true && step1 > 4 && (ShReg_position == 2) && (digitalRead(BUTT) == LOW)) //when error state is active, button is used to reset error
    {
      step1 = 0;
      con = 8;
    }
    else if (Err == false && step1 > 4 && (ShReg_position == 2) && (digitalRead(BUTT) == LOW))//when error state is inactive, button is used to changing channel
    {
      lastDebounceTime = millis();
      step1 = 0;
      con = 3;
    }

  }
  //set variable to 5 , if no button is used
  else if ((ShReg_position == 0 || ShReg_position == 1) && (digitalRead(BUTT) == HIGH) && con != 0 && con != 3 && con != 5 && con != 6 && con != 7 )
  {
    step1++;
    if (step1 > 5 && digitalRead(BUTT) == HIGH && (ShReg_position == 0 || ShReg_position == 1)) //debaucing
    {
      //if(con != 5)
      con = 5;
    }
  }

}

void loop() {

  wdt_reset(); //upload watchdog status
  setDigit(n); //devide number to units and set special characters
  //state amachine
  switch (state)
  {
    case stStartMotor:
      StartMotor();
      if (rot == true)
        state = stSensor;
      else if (rot == false)
        state = stRemote;
      break;

    case stSensor:
      Sensor();
      if (con == 1 || con == 2 || con == 5 || con == 6 || con == 7) //set next state when one of button was pressed
        state = stStopMotorBut;
      else if (con == 0) // set next state when motor is controled by computer
        state = stStopMotor;
      if (Err == true) //set nex state if same error ocucred
        state = stError;
      break;
    case stStopMotorBut:
      StopMotorBut();
      if (rot == true)
        state = stSensor;
      else if (rot == false)
        state = stRemote;
      break;

    case stStopMotor:
      StopMotor();
      if (rot == true) //set next state due to motor rotation
        state = stSensor;
      else if (rot == false)
        state = stRemote;

      break;
    case stRemote:
      Remote();
      if (con == 3) //if button CH/RESET is pressed next state will be stchannel
        state = stchannel;
      else
        state = stSerial;
      break;

    case stSerial:
      if (Serial.available()) //if there is some bit in serial buffer doperform the function
      { Seriall();
      }
      state = stStartMotor;
      break;

    case stError:
      Error();
      if (Err == true) //stay in state Error until the button CH/RESET is pressed
        state = stError;
      else if (Err == false)
        state = stRemote;
      break;

    case stchannel:
      channell();
      state = stSerial;
      break;
  }
  //end of state machine

}

//individuall function of state machine
//reading information about motoro rotation from magnetic contact sensor
void Sensor()
{
  byte lastsensor_state;
  lastsensor_state = sensor_state; //save last sensor state
  if ((millis() - lastDebounceTime1) > (setstep_size[channel] * 100)) //control predicted time for next change, if time is longer, some error must occured
    Err = true; //set auxiliary variable to activate Error state

  //reading changes of magnetic sensor
  if (sensor_state == 1 && digitalRead(sensor[channel]) == LOW) //magnetic contact are disconected
  {
    stepp++;
    if (stepp > 10) //debaucing
    {
      stepp = 0;
      sensor_state = 0;
    }
  }
  if (sensor_state == 0 && digitalRead(sensor[channel]) == HIGH)//magnetic contact are conected
  {
    stepp++;
    if (stepp > 10) //debaucing
    {
      stepp = 0;
      sensor_state = 1;
    }
  }
  if (sensor_state != lastsensor_state) //make action , when state was changed
  {
    digitalWrite(LED, digitalRead(LED) ^ 1); //front panel LED blinking
    step_size++;
    if (step_size == setstep_size[channel]) //evaluated setting step (sofftnes of rotation)
    {
      lastDebounceTime1 = millis(); //start time for error chacking
      step_size = 0;
      sensor_change = 1;

      if (direct == W)
      {
        positions[channel]--; //decrease position
        n = positions[channel]; //set actual position on display
        Serial.println(positions[channel]); //sent actual position via serial port
      }
      if (direct == E)
      {
        positions[channel]++; //increase position
        n = positions[channel];//set actual position on display
        Serial.println(positions[channel]);//sent actual position via serial port
      }
    }

  }
  else
  {
    sensor_change = 0;
  }
}

// stop motor controled by buttons
void StopMotorBut()
{
  if ( (con == 5 || con == 6 || con == 7) && (sensor_change == 1))
  {
    digitalWrite(rele1[channel], LOW); //disconect relay
    digitalWrite(rele2[channel], LOW);//disconect relay
    setposition[channel] = positions[channel];
    rot = false;
    con = 0;
    digitalWrite(LED, LOW); // set LED color to green
  }
}

//reading serial comunincation, proccesesing recived string and performing functions
void Seriall()
{
  byte input_length;
  byte i = 0;
  int number = 0;
  char Buffer[14] = {0}; //serial input buffer
  char command[11] = {0};// array for commands

  input_length = Serial.readBytesUntil('\n', Buffer, 14); //cauting number of received characters to specific character

  for (; Buffer[i] != '_' && i < 14; i++) //this part find text part of command
  {
    command[i] = Buffer[i];
    //Serial.println(command[i]);

  }
  //Serial.println(i);
  //Serial.println(Buffer[i]);
  i++;
  for (; i < input_length; i++) //this part change char of array to int number in number part of command
  {
    //if(i<10)
    //command[i]='a';
    number = number * 10;
    number = number + (Buffer[i] - '0');
    //Serial.println(number);
  }
  i = 0;
  //Serial.println(command);
  //Serial.println(number);

  if (strcmp(command, "setstep") == 0) //set size of one step
  {
    if (number > 0 && number < 11) //it checks the validity of the scope
    {
      setstep_size[channel] = number; //set received value to competent variable
      // according used channel, decide where received number will be saved
      // this approach is used for other commands too
      if (channel == 0)
        EEPROM.update(STEP, number);
      else if (channel == 1)
        EEPROM.update(STEP + 30, number);
      Serial.print(F("Krok je nastaveny na: ")); //send answer to serial port
      Serial.println(number);
    }
    else
      Serial.println(F("Nespravna hodnota")); //if validity if the scope is wrong , this answerr will be send

  }
  //logic used in this commnad is very simmilar for other commands...so they wont be commented again

  else if (strcmp(command, "getstep") == 0)
  {
    //Serial.print("Krok je nastaveny na: ");
    Serial.println(setstep_size[channel]);
  }
  else if (strcmp(command, "W") == 0)
  {
    if (number >= 0 && number < 256)
    {
      direct = W;
      setposition[channel] = ((-1) * number);
      direct = E;
      Serial.print(F("Idem na poziciu W: "));
      Serial.println(number);
    }
    else
      Serial.println(F("Nespravna hodnota"));

  }
  else if (strcmp(command, "E") == 0)
  {
    if (number >= 0 && number < 256)
    {
      direct = E;
      setposition[channel] = number;
      direct = W;
      Serial.print(F("Idem na poziciu E: "));
      Serial.println(number);
    }
    else
      Serial.println(F("Nespravna hodnota"));

  }
  else if (strcmp(command, "setzero") == 0)
  {
    positions[channel] = 0;
    setposition[channel] = positions[channel];
    n = 0;
    Serial.print(F("Pozicia je: "));
    Serial.println(number);
  }
  else if (strcmp(command, "setrbut") == 0)
  {
    if (number > 0 && number < 25)
    {
      rbut = (number - 1);
      Serial.print(F("Zvolene je tlacidlo c.:"));
      Serial.println(rbut + 1);
    }
    else
      Serial.println(F("Nespravna hodnota"));

  }
  else if (strcmp(command, "setch") == 0)
  {
    if (number > 0 && number < 3)
    {
      if ( channel != (number - 1))
      {
        con = 3;
        //channel();
        lastDebounceTime = millis();
      }

      Serial.print(F("Zvoleny je kanal c.:"));
      Serial.println(number);
    }
    else
      Serial.println(F("Nespravna hodnota"));

  }
  else if (strcmp(command, "setrbutW") == 0)
  {
    if (number >= 0 && number < 256)
    {
      pos[channel][rbut] = (number * -1);
      if (channel == 0)
      {
        EEPROM.update((rbut + 3), number);
        EEPROM.update((rbut + 33), 0);
      }
      else if (channel == 1)
      {
        EEPROM.update((rbut + 63), number);
        EEPROM.update((rbut + 93), 0);
      }
      //EEPROM.update(1,positions);
      Serial.print(F("Na kanaly.: "));
      Serial.print(channel + 1);
      Serial.print(F("tlacidle c: "));
      Serial.print(rbut + 1);
      Serial.print(F("je ulozena pozicia: "));
      if (channel == 0)
        Serial.println((EEPROM.read(rbut + 3) * -1));
      else if (channel == 1)
        Serial.println((EEPROM.read(rbut + 63) * -1));
    }

  }
  else if (strcmp(command, "setrbutE") == 0)
  {
    if (number >= 0 && number < 256)
    {
      pos[channel][rbut] = number;
      if (channel == 0)
      {
        EEPROM.update((rbut + 33), pos[channel][rbut]);
        EEPROM.update((rbut + 3), 0);
      }
      else if (channel == 1)
      {
        EEPROM.update((rbut + 93), pos[channel][rbut]);
        EEPROM.update((rbut + 63), 0);
      }

      Serial.print(F("Na kanaly.: "));
      Serial.print(channel + 1);
      Serial.print(F("tlacidle c: "));
      Serial.print(rbut + 1);
      Serial.print(F("je ulozena pozicia: "));
      if (channel == 0)
        Serial.println(EEPROM.read(rbut + 33));
      else if (channel == 1)
        Serial.println(EEPROM.read(rbut + 93));

    }
    else
      Serial.println(F("Nespravna hodnota"));

  }
  else if (strcmp(command, "getpos") == 0)
    Serial.println(positions[channel]);
  else if (strcmp(command, "reset") == 0)
    con = 8;
  else
    Serial.println(F("Nespravnyprikaz"));

}

//stop motor controled  by computer
void StopMotor()
{
  //stop motor if set and actual position are equal
  if (positions[channel] == setposition[channel])
  {
    digitalWrite(rele1[channel], LOW);
    rot = false;
    digitalWrite(LED, LOW);
  }


  if (positions[channel] == setposition[channel])
  {
    digitalWrite(rele2[channel], LOW);
    rot = false;
    digitalWrite(LED, LOW);
  }


}
//starting motor rotation
void StartMotor()
{ //when motor is controled by computer
  if (direct == W && positions[channel] != setposition[channel]) //new  position was set
  {
    //logic decided about direction of motoro rotation
    if (positions[channel] < setposition[channel])
    {
      digitalWrite(rele2[channel], HIGH); // activate competetn relay
      direct = E; //set direction
      lastDebounceTime1 = millis(); //start time for error chacking
      rot = true; //inform program that motor is rotating

    }
    else
      digitalWrite(rele1[channel], HIGH);
    lastDebounceTime1 = millis();
    rot = true;

  }


  else if (direct == E && positions[channel] != setposition[channel])
  {
    if (positions[channel] > setposition[channel])
    {
      digitalWrite(rele1[channel], HIGH);
      direct = W;
      lastDebounceTime1 = millis();
      rot = true;
    }

    else
      digitalWrite(rele2[channel], HIGH);
    lastDebounceTime1 = millis();
    rot = true;

  }
  else {
    //when motor is controled by buttons
    switch (con) //
    {
      //controling by gadget buttons
      case 1:
        digitalWrite(rele1[channel], HIGH);
        lastDebounceTime1 = millis();
        rot = true;
        break;

      case 2:
        digitalWrite(rele2[channel], HIGH);
        lastDebounceTime1 = millis();
        rot = true;
        break;
      // contoling by remote control buttons
      case 6:
        digitalWrite(rele1[channel], HIGH);
        lastDebounceTime1 = millis();
        rot = true;
        break;
      case 7:
        digitalWrite(rele2[channel], HIGH);
        lastDebounceTime1 = millis();
        rot = true;
        break;
    }
  }
}

//devide number to units and set special characters
void setDigit(int n)
{
  if (Err == false) //dispaly position in normal condition
  {
    if (n < 0)
      n = n * (-1);
    digit[0] = n % 10;
    digit[1] = (n / 10) % 10;
    if ((digit[1] == 0) && (n < 10)) {
      digit[1] = -1;
    }
    digit[2] = (n / 100) % 10;
    if ((digit[2] == 0) && (n < 100)) {
      digit[2] = -1;
    }

    if (con == 3) //dispaly C during channel changing
      digit[3] = 10; //C
    else if (StorePostoRBut == true) //display "S" at first position when store state of remoto control is active
      digit[3] = 5; //S
    else if (positions[channel] > 0) //for direction E, display symbol E at first position of segments display
      digit[3] = 11; //E
    else
      digit[3] = -1; //all segment off
  }
  else if (Err == true) // when error occured dispaly "Err" and number of specific error
  {
    digit[3] = 11; //E
    digit[2] = 13; //r
    digit[1] = 13; //r
    digit[0] = n % 10;
  }
}

//set each segment to numbers
void initNumber()
{
  number[0][MIDDLE]  = OFF;
  number[0][UPPER_L] = ON;
  number[0][LOWER_L] = ON;
  number[0][BOTTOM]  = ON;
  number[0][LOWER_R] = ON;
  number[0][UPPER_R] = ON;
  number[0][TOP]     = ON;

  number[1][MIDDLE]  = OFF;
  number[1][UPPER_L] = OFF;
  number[1][LOWER_L] = OFF;
  number[1][BOTTOM]  = OFF;
  number[1][LOWER_R] = ON;
  number[1][UPPER_R] = ON;
  number[1][TOP]     = OFF;

  number[2][MIDDLE]  = ON;
  number[2][UPPER_L] = OFF;
  number[2][LOWER_L] = ON;
  number[2][BOTTOM]  = ON;
  number[2][LOWER_R] = OFF;
  number[2][UPPER_R] = ON;
  number[2][TOP]     = ON;

  number[3][MIDDLE]  = ON;
  number[3][UPPER_L] = OFF;
  number[3][LOWER_L] = OFF;
  number[3][BOTTOM]  = ON;
  number[3][LOWER_R] = ON;
  number[3][UPPER_R] = ON;
  number[3][TOP]     = ON;

  number[4][MIDDLE]  = ON;
  number[4][UPPER_L] = ON;
  number[4][LOWER_L] = OFF;
  number[4][BOTTOM]  = OFF;
  number[4][LOWER_R] = ON;
  number[4][UPPER_R] = ON;
  number[4][TOP]     = OFF;

  number[5][MIDDLE]  = ON;
  number[5][UPPER_L] = ON;
  number[5][LOWER_L] = OFF;
  number[5][BOTTOM]  = ON;
  number[5][LOWER_R] = ON;
  number[5][UPPER_R] = OFF;
  number[5][TOP]     = ON;

  number[6][MIDDLE]  = ON;
  number[6][UPPER_L] = ON;
  number[6][LOWER_L] = ON;
  number[6][BOTTOM]  = ON;
  number[6][LOWER_R] = ON;
  number[6][UPPER_R] = OFF;
  number[6][TOP]     = ON;

  number[7][MIDDLE]  = OFF;
  number[7][UPPER_L] = OFF;
  number[7][LOWER_L] = OFF;
  number[7][BOTTOM]  = OFF;
  number[7][LOWER_R] = ON;
  number[7][UPPER_R] = ON;
  number[7][TOP]     = ON;

  number[8][MIDDLE]  = ON;
  number[8][UPPER_L] = ON;
  number[8][LOWER_L] = ON;
  number[8][BOTTOM]  = ON;
  number[8][LOWER_R] = ON;
  number[8][UPPER_R] = ON;
  number[8][TOP]     = ON;

  number[9][MIDDLE]  = ON;
  number[9][UPPER_L] = ON;
  number[9][LOWER_L] = OFF;
  number[9][BOTTOM]  = ON;
  number[9][LOWER_R] = ON;
  number[9][UPPER_R] = ON;
  number[9][TOP]     = ON;
  // specific signs
  //sign C
  number[10][MIDDLE]  = OFF;
  number[10][UPPER_L] = ON;
  number[10][LOWER_L] = ON;
  number[10][BOTTOM]  = ON;
  number[10][LOWER_R] = OFF;
  number[10][UPPER_R] = OFF;
  number[10][TOP]     = ON;
  //sign E
  number[11][MIDDLE]  = ON;
  number[11][UPPER_L] = ON;
  number[11][LOWER_L] = ON;
  number[11][BOTTOM]  = ON;
  number[11][LOWER_R] = OFF;
  number[11][UPPER_R] = OFF;
  number[11][TOP]     = ON;
  //sign S
  number[12][MIDDLE]  = ON;
  number[12][UPPER_L] = ON;
  number[12][LOWER_L] = OFF;
  number[12][BOTTOM]  = ON;
  number[12][LOWER_R] = ON;
  number[12][UPPER_R] = OFF;
  number[12][TOP]     = ON;
  //sign r
  number[13][MIDDLE]  = ON;
  number[13][UPPER_L] = OFF;
  number[13][LOWER_L] = ON;
  number[13][BOTTOM]  = OFF;
  number[13][LOWER_R] = OFF;
  number[13][UPPER_R] = OFF;
  number[13][TOP]     = OFF;
}

// set register for TIMER1
void setupTimer1() {
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 300.0018750117188 Hz (16000000/((53332+1)*1))
  OCR1A = 53332;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 1
  TCCR1B |= (1 << CS10);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

//save position to EEPROM after power lost
void flash()
{ //for channel 1
  if (positions[0] <= 0) //if actual position is zero, save value zeto to both position in EEPROM
  {
    EEPROM.write(0, ((-1)*positions[0])); //save actual position to EEPROM[0] position, when antena is located in west (W) area
  }
  if (positions[0] > 0)
  {
    EEPROM.write(1, positions[0]); //save actual position to EEPROM[1] position, when antena is located in east (E) area
    EEPROM.write(0, 0);
  }
  //for channel 2
  if (positions[1] <= 0)
  {
    EEPROM.write(30, ((-1)*positions[1]));
    EEPROM.write(31, 0);
  }
  if (positions[1] > 0)
  {
    EEPROM.write(31, positions[1]);
    EEPROM.write(30, 0);
  }
}

//controling device by remote control
void Remote()
{
  if ( MyReceiver.getResults()) // when signal was received do...
  {
    MyDecoder.decode(); //try to decode signal
    if (MyDecoder.value == 0) //if signal was decodes incorrectly, throw away it
    {
      MyReceiver.enableIRIn(); //enable receiving new signal
    }
    else if (MyDecoder.value == 0x4F9807F) //decoded signal is button ->
    {
      con = 6;
      direct = W;
      MyReceiver.enableIRIn();
    }
    else if (MyDecoder.value == 0x4F910EF) //decoded signal is button <-
    {
      con = 7;
      direct = E;
      MyReceiver.enableIRIn();
    }
    else if (MyDecoder.value == 0x4F9C03F) //decoded signal is button STORE
    {
      if (StorePostoRBut == false)  // set auxiliary variable to active saveing mod
        StorePostoRBut = true;
      MyReceiver.enableIRIn();
    }
    else
    {
      for (int i = 0; i < 25; i++)
      {
        //saving mod
        if (StorePostoRBut == true)
        {
          if (MyDecoder.value == code[i]) //compare received code with codes safed in memmory a find out , which button was pressed
          {
            if (positions[channel] <= 0) //according location of antenna (E or W), decide where position will be saved
            {
              pos[channel][i] = (positions[channel] * (-1));
              if (channel == 0) //according channel,determine where position will be saved
              {
                EEPROM.update((i + 3), pos[channel][i]); //save position in W area
                EEPROM.update((i + 33), 0); //save complemetary position to zero for future reading
              }
              else if (channel == 1)
              {
                EEPROM.update((i + 63), pos[channel][i]);
                EEPROM.update((i + 93), 0);
              }
              StorePostoRBut = false;
              MyReceiver.enableIRIn();
            }
            else
            {
              pos[channel][i] = positions[channel]; //saving position in W area
              if (channel == 0)
              {
                EEPROM.update((i + 33), pos[channel][i]);
                EEPROM.update((i + 3), 0);
              }
              else if (channel == 1)
              {
                EEPROM.update((i + 93), pos[channel][i]);
                EEPROM.update((i + 63), 0);
              }
              StorePostoRBut = false;
              MyReceiver.enableIRIn();
            }
          }
          //reading mod
          else
          {
            //find out pressed button and read position from array, array is loading fromm EEPROM in initialization
            if (MyDecoder.value == code[i])
            {
              if (pos[channel][i] < 0)
              {
                setposition[channel] = pos[channel][i];
                direct = W;
              }

              else {
                setposition[channel] = pos[channel][i];
                direct = E;
              }

            }
          }
        }
        MyReceiver.enableIRIn();
      }
    }
  }

  else if ((con == 6 || con == 7) && MyDecoder.value != 0xFFFFFFF)
  {
    //Serial.println("tu zastavujem");
    con = 5;
    MyReceiver.enableIRIn();
  }

}

//changing actual channel to next one
void channell()
{
  if (channel == 0)//if actual channel is 1 , change channel to 2
  {
    n = 2; //set 2 on display
    channel = 1; //change channel
    setDigit(n);
    while ((millis() - lastDebounceTime) < 1500) //diplay information about new channel for a certain time
    {
    }
    n = positions[channel]; //then display actual position of new channel
    con = 0;
  }

  else if (channel == 1)//same logic for this channel
  {

    n = 1;
    channel = 0;
    setDigit(n);
    while ((millis() - lastDebounceTime) < 1500)
    {
    }
    n = positions[channel];
    con = 0;
  }

}

//determine type of error
void Error()
{
  if (rot == 1 && digitalRead(backstop[channel]) == LOW) // antenna has reached backstop
  { digitalWrite(rele1[channel], LOW); //realy off
    digitalWrite(rele2[channel], LOW);  //realy off
    digitalWrite(LED, HIGH); // LED is red
    Serial.println(991); //send special number for error 1
    rot = 0; //inform program that motor stop
    n = 1;
  }

  else if (rot == 1   && digitalRead(backstop[channel]) == HIGH)// magnetic sensor broke up
  {
    digitalWrite(rele1[channel], LOW);
    digitalWrite(rele2[channel], LOW);
    digitalWrite(LED, HIGH);
    Serial.println(992);//send special number for error 1
    rot = 0;
    n = 2;


  }

  if (con == 8) //user have to press buton CH/RESET to end this state
  {
    Err = false;
    setposition[channel] = positions[channel];
    n = positions[channel];
    digitalWrite(LED, LOW);// LED go green
    con = 0;
  }
}

// laoding information saved in EEPROM memmory
void EEPROMload()
{
  //working with EEPROM use same logic
  //for one variable there si two EEPROM positions, one for W area and one for E area
  //if actual value is zero , zero is set to both positons
  //if actual position lay in W area, EEPROM position for W is set and complemetary E position is set to zero
  // for E arey is used reverse logic
  //so during loading, the non-zero value is read from EEPROM and set to specific variable
  if (EEPROM.read(30) == 0) //W area position at channel 2
  {
    positions[1] = EEPROM.read(31);
    n = positions[channel];
    setposition[1] = positions[1];
  }
  else if (EEPROM.read(31) == 0)//E area position at channel 2
  {
    positions[1] = (EEPROM.read(30)) * (-1);
    n = positions[channel];
    setposition[1] = positions[1];
  }

  if (EEPROM.read(0) == 0) //W area position at channel 1
  {
    positions[0] = EEPROM.read(1);
    n = positions[channel];
    setposition[0] = positions[0];
  }
  else if (EEPROM.read(1) == 0)//E area position at channel 1
  {
    positions[0] = (EEPROM.read(0)) * (-1);
    n = positions[channel];
    setposition[0] = positions[0];
  }

  if (EEPROM.read(STEP) > 0) //step at channal 1
    setstep_size[0] = EEPROM.read(STEP);

  if (EEPROM.read(STEP + 30) > 0) //step at channal 2
    setstep_size[1] = EEPROM.read(STEP + 30);

  for (int i = 0; i < 24; i++) //load code and position of all remote control buttons
  {
    if (EEPROM.read(i + 3) == 0)//at channel 1
    {
      pos[0][i] = EEPROM.read(i + 33);

    }
    else if (EEPROM.read(i + 33) == 0)
    {
      pos[0][i] = (EEPROM.read(i + 3) * -1);
    }

  }
  for (int i = 0; i < 24; i++) //at channel 1
  {
    if (EEPROM.read(i + 63) == 0)
    {
      pos[1][i] = EEPROM.read(i + 93);

    }
    else if (EEPROM.read(i + 93) == 0)
    {
      pos[1][i] = (EEPROM.read(i + 63) * -1);
    }

  }
}
