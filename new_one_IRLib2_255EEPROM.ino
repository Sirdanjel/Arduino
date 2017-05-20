#include <EEPROM.h>
#include <avr/wdt.h>
//library for IR remote decoding
#include <IRLibRecv.h>
#include <IRLibDecodeBase.h>
#include <IRLib_P01_NEC.h>
#include <IRLibCombo.h>
IRrecv MyReceiver(10);
IRdecode MyDecoder;
long code[24]={0x4F9E01F,0x4F928D7,0x4F9A05F,83488783,83429623,83452063,83439823, 83462263, 83447983, 83472463, 83445943, 83480623, 83456143, 83478583, 83464303, 83470423, 83454103, 83486743, 83433703, 83466343, 83450023, 83482663, 83441863, 83474503};
int pos[2][24];
int rbut=0;
boolean StorePostoRBut=false;


#define INTERUPTPIN 2
#define STEP 2
#define DATA 11
#define clock 13
#define BUTT 7
#define SENSOR1 18
#define SENSOR2 12
#define TLAC 7
#define RELE1 16
#define RELE2 17
#define RELE3 8
#define RELE4 9
#define DORAZ1 15
#define DORAZ2 14
#define W 0
#define E 1
#define LED 6


//common variables
boolean rot=false;
boolean Err=false;
byte sensor[2]={SENSOR1,SENSOR2};
byte doraz[2]={DORAZ1,DORAZ2};
byte rele1[2]={RELE1,RELE3};
byte rele2[2]={RELE2,RELE4};
byte direct;
int positions[2]={0,0};
int setposition[2]={0,0};
byte channal=0;
unsigned long lastDebounceTime = 0; 
unsigned long lastDebounceTime1 = 0;
unsigned long debounceDelay = 1500;


//define display and buttoms
#define MIDDLE 6 // G
#define UPPER_L 5 // F
#define LOWER_L 4 // E
#define BOTTOM 3 // D
#define LOWER_R 2 // C
#define UPPER_R 1 // B
#define TOP 0 // A

void setupTimer1();
void initNumber();
void setDigit(int n);

int n = 0;
int ShReg_position=0; //shift bit position in shift register
//zmena pinu  na 3 pozicii z 3 na 10
int digitPins[4] = {5, 4, 3, 19}; //pins for 7-segment multiplexing
int ON = HIGH; // HIGH for CA, LOW for CC
int OFF = LOW; // LOW for CA, HIGH for CC
int number[14][7]; // holds information about which segments are needed for each of the 10 numbers
int digit[4]; // holds values for each of the 4 display digits

//sensor variable
byte sensor_change = 0;
byte stepp;
byte sensor_state;
byte step_size = 0;
byte setstep_size[2] = {1,1};

// button variable
volatile byte button = 0;
byte step1=0;

enum {stSensor, stStopMotorBut, stSerial, stStopMotor, stRemote, stError, stChannal,stStartMotor}
state = stStartMotor;
void setupTimer2();
void Remote();
void flash();
void Motor();
void EEPROMload();
boolean a;

void setup() {
  wdt_enable(WDTO_4S);
  //setup power loose interupt
attachInterrupt(digitalPinToInterrupt(INTERUPTPIN),flash,RISING);
  //timer1 interrupt setup to 300ms
  setupTimer2();
  //load values save in EEPROM
   EEPROMload();
  MyReceiver.enableIRIn();//enable IR receiver
  Serial.begin(9600); // start serial comunication
  //setup for display and buttom
  shiftOut(DATA, clock, MSBFIRST, B1111111);
  initNumber();
  pinMode(clock, OUTPUT);
  pinMode(DATA, OUTPUT);
   for(int i=0; i < 4; i++) {
    pinMode(digitPins[i], OUTPUT);
    digitalWrite(digitPins[i], LOW); // LOW for CA, HIGH for CC
  } 
  pinMode(TLAC, INPUT_PULLUP);
  pinMode(SENSOR1, INPUT_PULLUP);
  pinMode(SENSOR2, INPUT_PULLUP);
  pinMode(RELE1, OUTPUT);
  pinMode(RELE2, OUTPUT);
  pinMode(RELE3, OUTPUT);
  pinMode(RELE4, OUTPUT);
  pinMode(DORAZ1,INPUT);
  pinMode(DORAZ2,INPUT);
  pinMode(LED, OUTPUT);
   //turn LED diode to green color
  digitalWrite(LED,LOW);
}

//timer1 interrupt service routine 
//void displej()       // interrupt service routine 
ISR(TIMER1_COMPA_vect)
{ interrupts();

 
  //TCNT1 = timer1_counter;   // preload timer
  //digitalWrite(13, digitalRead(13) ^ 1);
  //if(a==true)
  {
  //setDigit(n);
    for(int i=0; i < 4; i++) {
       digitalWrite(digitPins[i], LOW);
    }
    ShReg_position++;
    if(ShReg_position>6){
      ShReg_position=0;
    }
    shiftOut(DATA, clock, MSBFIRST,~(1<<ShReg_position)); // LOW for CA, HIGH for CC
    for(int i=0; i < 4; i++) {
      if (digit[i] < 0) {
       digitalWrite(digitPins[i], LOW);
       continue;
      }
      digitalWrite(digitPins[i], number[digit[i]][ShReg_position]);
       
    }
    
   if((ShReg_position==0)&&(digitalRead(TLAC)==LOW))
   {// Serial.println(button);
     step1++;
    if(step1>4 && (ShReg_position==0)&&(digitalRead(TLAC)==LOW) && button!=2)
    {
      button=1;
      step1=0;
      direct=W;
      //Serial.println(button); 
   }
     
   }
   else if((ShReg_position==1)&&(digitalRead(TLAC)==LOW))
   {//  Serial.println(button);
    step1++;
      if(step1>7 && (ShReg_position==1)&&(digitalRead(TLAC)==LOW) && button!=1)
      {
        button=2;
        step1=0;
        direct=E;
        //Serial.println(button); 
     }
   }
   else if((ShReg_position==2)&&(digitalRead(TLAC)==LOW))
   {//  Serial.println(button);
    step1++;
      if(Err==true && step1>4 && (ShReg_position==2)&&(digitalRead(TLAC)==LOW))
      {  
        //lastDebounceTime = millis();
        step1=0;
        button=8; 
        //Serial.println(button);             
     }
     else if(Err==false && step1>4 && (ShReg_position==2)&&(digitalRead(TLAC)==LOW))
      { //Serial.println(button); 
        lastDebounceTime = millis();
        step1=0;
        button=3;
      }
   }
   else if((ShReg_position==0||ShReg_position==1)&&(digitalRead(TLAC)==HIGH)&& button!=0 && button!=3 && button!=6 && button!=7 )
   {
    step1++;
    if(step1>5 && digitalRead(TLAC)==HIGH &&(ShReg_position==0||ShReg_position==1))
    {button=5;
    //step1=0;
    //Serial.println(button); 
   }
   }
   
    
}
}

void loop() {
  //if (irrecv.decode(&results))
  //Remote();
  //Serial.println(rot);
  wdt_reset();
  setDigit(n);
  switch (state)
  {
    case stStartMotor:
    //Serial.println("startM");
      StartMotor();
      if(rot==true)
      state=stSensor;
      else if(rot==false)
      state = stRemote;
      break;

    case stSensor:
     //Serial.println("Sensor");
     //Serial.println(state);
      Sensor();
      if(button==1 || button==2 || button==5 || button==6 ||button==7)
      state = stStopMotorBut;
      //treba to riesit v inom kroku
      //else if(button==3)
      //state=stChannal;
      else if(button==0)
      state = stStopMotor;
      if(Err==true)
      state = stError;
      break;
    case stStopMotorBut:
   // Serial.println("stStopMotorBut");
    // Serial.println(state);
      StopMotorBut();
      if(rot==true)
      state= stSensor;
      else if(rot==false)
      state = stRemote;
      break;

    case stStopMotor:
   // Serial.println("stStopMotor");
   // Serial.println(state);
      StopMotor();
      if(rot==true)
      state= stSensor;
      else if(rot==false)
      state = stRemote;

      break;
    case stRemote:
    // Serial.println("stRemote");
   // Serial.println(state);
      Remote();
      if(button==3)
      state = stChannal;
      else
      state = stSerial;
      break;
    
    case stSerial:
     //Serial.println("stSerial");
   // Serial.println(state);
      if (Serial.available())
      {Seriall();
      }   
      state = stStartMotor;
      break;

    case stError:
      Error();
      if(Err==true)
      state = stError;
      else if(Err==false)
      state = stRemote;
      break;

    case stChannal:
    // Serial.println("stChannal");
   // Serial.println(state);
      Channal();
      state = stSerial;
      break;


  }
  // put your main code here, to run repeatedly:

}
void Sensor()
{


  byte lastsensor_state;
  lastsensor_state = sensor_state;
  if ((millis() - lastDebounceTime1) > (setstep_size[channal]*100))
  Err=true;
  if (sensor_state == 1 && digitalRead(sensor[channal]) == LOW)
  {
    stepp++;
    if (stepp > 10)
    {
      stepp = 0;
      sensor_state = 0;
    }
  }
  if (sensor_state == 0 && digitalRead(sensor[channal]) == HIGH)
  {
    stepp++;
    if (stepp > 10)
    {
      stepp = 0;
      sensor_state = 1;
    }
  }
  if (sensor_state != lastsensor_state)
  {// Serial.println(step_size);
    digitalWrite(LED, digitalRead(LED) ^ 1);
    step_size++;
    if (step_size == setstep_size[channal])
    { 
      lastDebounceTime1=millis();
      step_size = 0;
      sensor_change = 1;
      
      if(direct==W)
      {
        positions[channal]--;
        n=positions[channal];
        Serial.println(positions[channal]);
      }
      if(direct==E)
      {
        positions[channal]++;
        n=positions[channal];
        Serial.println(positions[channal]);
      }   
    }

  }
  else
  {
    sensor_change = 0;
  }
}


void StopMotorBut()
{
  /*switch (button)
  {
    case 1:
      digitalWrite(RELE1, HIGH);
      
      break;
      
    case 2:
      digitalWrite(RELE2, HIGH);
      
      break;
    case 6:
      digitalWrite(RELE1, HIGH);
      
      break;
    case 7:
      digitalWrite(RELE2, HIGH);
      
      break;
  }*/
  


  if( (button==5 || button==6 || button==7)&& (sensor_change == 1))
  {
    digitalWrite(rele1[channal], LOW);
    digitalWrite(rele2[channal], LOW);
    setposition[channal]=positions[channal];
    rot=false;
    button=0;
    digitalWrite(LED, LOW);
  }
}

void Seriall()
{ 
   {
   byte j;
   byte i=0;
   int number=0;
   char Buffer[14];
   char command[11];
   char command1[11]={"aaaaaaaaa"};
   //
  //Serial.println(freeRam());
    //freeRam();

 
     
      j=Serial.readBytesUntil('\n',Buffer,14); //pocita pocet prijatych znakov po charakter znak + plny definovany buffer
      //Serial.println(j);
      while(Buffer[i]!='_') //this part find text part of command
      {
        command1[i]=Buffer[i];
        //Serial.println(command[i]);
        i++;
        
      }
        
        i++;
        
         //command[8]='a';
         //command[9]='a';
         //command[10]='a';
         //command[11]='a';
         strncpy(command, command1, 11);
         //command[12]='a';
         //command[13]='a';
        // command[14]='a';
      for( i;i<j;i++) //this part change char of array to int number in number part of command
      {
        //if(i<10)
        //command[i]='a';
        number=number*10;
        number=number+(Buffer[i]-'0');     
        //Serial.println(otoc);
      }
      i=0;
     // Serial.println(command);
     // Serial.println(number);
    
      if(strcmp(command, "setstepaa") == 0)
      { 
        if (number>0 && number<11)
        {
          setstep_size[channal] = number;
          if(channal==0)
          EEPROM.update(STEP,number);
          else if(channal==1)
          EEPROM.update(STEP+30,number);
          Serial.print(F("Krok je nastaveny na: "));
          Serial.println(number);
        }
        else
          Serial.println(F("Nespravna hodnota"));          
      }
      
      else if(strcmp(command, "getstepaa") == 0)
      {
        //Serial.print("Krok je nastaveny na: ");
        Serial.println(setstep_size[channal]); 
      }
      
      else if(strcmp(command, "Waaaaaaaa") == 0)
      {
        if (number>=0 && number<256)
        {
           direct=W;
          setposition[channal]=((-1)*number);
          //if(positions[channal]<setposition[channal])
          direct=E;
         // Motor1();
          Serial.print(F("Idem na poziciu W: "));
          Serial.println(number); 
        }
        else
          Serial.println(F("Nespravna hodnota"));       
      }
      
      else if(strcmp(command, "Eaaaaaaaa") == 0)
      { 
        if (number>=0 && number<256)
        {
          direct=E;
          setposition[channal]=number;
          //if(positions[channal]>setposition[channal])
          direct=W;
          //Motor1();
          Serial.print(F("Idem na poziciu E: "));
          Serial.println(number); 
        }
         else
          Serial.println(F("Nespravna hodnota"));
        
      }
      
      else if(strcmp(command, "setzeroaa") == 0)
      { 
        positions[channal]=0;
        setposition[channal]=positions[channal];
        n=0;
        //EEPROM.update(0,positions);
        //EEPROM.update(1,positions);
        Serial.print(F("Pozicia je: "));
        Serial.println(number);
      }
      
      else if(strcmp(command, "setrbutaa") == 0)
      { 
        if (number>0 && number<25)
        {
           rbut=(number-1);
          //EEPROM.update(0,positions);
          //EEPROM.update(1,positions);
          Serial.print(F("Zvolene je tlacidlo c.:"));
          Serial.println(rbut+1); 
        }
        else
          Serial.println(F("Nespravna hodnota"));
      }
      
      else if(strcmp(command, "setchaaaa") == 0)
      { 
        if (number>0 && number<3)
        {
           if( channal!=(number-1))
          {   button=3;
            //Channal();
            
           lastDebounceTime = millis();
          }
          Serial.print(F("Zvoleny je kanal c.:"));
          Serial.println(number);
       }
        else
          Serial.println(F("Nespravna hodnota"));
      }
      
      else if(strcmp(command, "setrbutWa") == 0)
      { 
        if (number>=0 && number<256)
        {
          pos[channal][rbut]=(number*-1);
          if(channal==0)
          {
            EEPROM.update((rbut+3),number);
            EEPROM.update((rbut+33),0);
          }
          
          else if(channal==1)
          {
            EEPROM.update((rbut+63),number);
            EEPROM.update((rbut+93),0);
          }
          
          
          //EEPROM.update(1,positions);
          Serial.print(F("Na kanaly.: "));
          Serial.print(channal+1);
          Serial.print(F("tlacidle c: "));
          Serial.print(rbut+1);
          Serial.print(F("je ulozena pozicia: "));
           if(channal==0)
          Serial.println((EEPROM.read(rbut+3)*-1));
           else if(channal==1)
          Serial.println((EEPROM.read(rbut+63)*-1));        
        }
        
       
      }
      else if(strcmp(command, "setrbutEa") == 0)
      { 
        if (number>=0 && number<256)
        {
           pos[channal][rbut]=number;
          if(channal==0)
          {
            EEPROM.update((rbut+33),pos[channal][rbut]);
            EEPROM.update((rbut+3),0);
          }
          
          else if(channal==1)
          {
            EEPROM.update((rbut+93),pos[channal][rbut]);
            EEPROM.update((rbut+63),0);
          }
          
          //EEPROM.update(1,positions);
          Serial.print(F("Na kanaly.: "));
          Serial.print(channal+1);
          Serial.print(F("tlacidle c: "));
          Serial.print(rbut+1);
          Serial.print(F("je ulozena pozicia: "));
           if(channal==0)
          Serial.println(EEPROM.read(rbut+33));
           else if(channal==1)
          Serial.println(EEPROM.read(rbut+93));
        }
        else
          Serial.println(F("Nespravna hodnota"));
                       
      }
      else if(strcmp(command, "getposaaa") == 0)
      { 
        Serial.println(positions[channal]);
      }
      else if(strcmp(command, "resetaaaa") == 0)
      { 
        button=8; 
      }
      else
      {
        Serial.println(F("Nespravnyprikaz"));
      }
  }

}

void StopMotor()
{
 
  if(positions[channal]==setposition[channal])
  {
    digitalWrite(rele1[channal],LOW);
    rot=false;
    digitalWrite(LED, LOW);
  }

  
  if(positions[channal]==setposition[channal])
  {
    digitalWrite(rele2[channal],LOW);
    rot=false;
    digitalWrite(LED, LOW);
  }
  
  
}

void StartMotor()
{ //Serial.println("StartMotor");
  if(direct==W && positions[channal]!=setposition[channal])
  {
    if(positions[channal]<setposition[channal])
    {
    digitalWrite(rele2[channal], HIGH);
    direct=E;
    lastDebounceTime1 = millis();
    rot=true;
    
    }
    else
    digitalWrite(rele1[channal], HIGH);
    lastDebounceTime1 = millis();
    rot=true;
    
  }
  

  else if(direct==E && positions[channal]!=setposition[channal])
  { 
    if(positions[channal]>setposition[channal])
    {
      digitalWrite(rele1[channal], HIGH);
      direct=W;
      lastDebounceTime1 = millis();
      rot=true;
    }
    
    else
    digitalWrite(rele2[channal], HIGH);
    lastDebounceTime1 = millis();
    rot=true;
    
  }
  else{
  switch (button)
  {
    case 1:
      digitalWrite(rele1[channal], HIGH);
      lastDebounceTime1 = millis();
      rot=true;
      break;
      
    case 2:
      digitalWrite(rele2[channal], HIGH);
      lastDebounceTime1 = millis();
      rot=true;
      break;
    case 6:
      digitalWrite(rele1[channal], HIGH);
      lastDebounceTime1 = millis();
      rot=true;
      break;
    case 7:
      digitalWrite(rele2[channal], HIGH);
      lastDebounceTime1 = millis();
      rot=true;
      break;
  }
  }  
}
void setDigit(int n) //devide number to units
{
// rozdelenie cisla na samostatne cislice, ale v takom pripade sa zobrazuju vsetky
  if(Err==false)
  {
    if(n<0)
      n=n*(-1);
    digit[0] = n % 10;
    digit[1] = (n / 10) % 10;
    if ((digit[1] == 0) && (n < 10)) {
      digit[1] = -1;
    }
    digit[2] = (n / 100) % 10;
    if ((digit[2] == 0) && (n < 100)) {
      digit[2] = -1;
    }
   
    if(button==3)
     digit[3]=10;
    else if(StorePostoRBut==true)
     digit[3]=5;
    else if(positions[channal]>0) //for direction E, display symbol E at first position of segments display
     digit[3]=11;
    else
     digit[3]=-1;
    //digit[3] = (n / 1000) % 10;
    //if (digit[3] == 0) {
    //  digit[3] = -1;
    //digit[3]=10;
    //}
  }
  else if(Err==true)
  {
    digit[3]=11;
    digit[2]=13;
    digit[1]=13;
    digit[0] = n % 10;
  }
}

void initNumber() //set each segment to numbers
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

void setupTimer2() {
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

void flash() //save position to EEPROM after power lost
{  
  if(positions[0] <=0) //ked je pozicia nula ulozi sa nula do oboch pozicii EEPROM
  {
    EEPROM.write(0,((-1)*positions[0])); //ulozi aktualnu poziciu na poziciu EEPROM[0], je to pre pripad ze satelit je v oblasti W, co je specifikovane znamiekom -
    EEPROM.write(1,0);
  }
  if(positions[0]>0)
  {
    EEPROM.write(1,positions[0]);//ulozi aktualnu poziciu do EEPROM ak je satelit v oblasti E
    EEPROM.write(0,0);
  }
  if(positions[1] <=0) //ked je pozicia nula ulozi sa nula do oboch pozicii EEPROM
  {
    EEPROM.write(30,((-1)*positions[1])); //ulozi aktualnu poziciu na poziciu EEPROM[0], je to pre pripad ze satelit je v oblasti W, co je specifikovane znamiekom -
    EEPROM.write(31,0);
  }
  if(positions[1]>0)
  {
    EEPROM.write(31,positions[1]);//ulozi aktualnu poziciu do EEPROM ak je satelit v oblasti E
    EEPROM.write(30,0);
  }
   
    //Serial.println("zapisanie vo flash");
    // positions[channal] = EEPROM.read(0);
   // Serial.println(positions[channal]);
    
}
void Remote()
{ //Serial.println("ano");
  if( MyReceiver.getResults())
  {
    MyDecoder.decode();
    //Serial.println(MyDecoder.value);
    if(MyDecoder.value==0)
    {
      MyReceiver.enableIRIn();
    }
    else if(MyDecoder.value==0x4F9807F)
    {
      button=6;
      direct=W;
      MyReceiver.enableIRIn();
    }
    else if(MyDecoder.value==0x4F910EF)
    {
      button=7;
      direct=E;
      MyReceiver.enableIRIn();
    }
    else if(MyDecoder.value==0x4F9C03F)
    {
      if(StorePostoRBut==false)
      StorePostoRBut=true;
      //Serial.println(StorePostoRBut);
      MyReceiver.enableIRIn();
    }
    else 
    {
      for(int i=0;i<25;i++)
      { 
        if(StorePostoRBut==true)
        {
          if(MyDecoder.value==code[i])
         {
          if(positions[channal]<=0)
          { 
            pos[channal][i]=(positions[channal]*(-1));
            //Serial.println("Save");
            //Serial.println(pos[channal][i]);
            if(channal==0)
            {
              EEPROM.update((i+3),pos[channal][i]);
              EEPROM.update((i+33),0);
            }
            else if(channal==1)
            {
              EEPROM.update((i+63),pos[channal][i]);
              EEPROM.update((i+93),0);
            }  
            StorePostoRBut=false;
            MyReceiver.enableIRIn();
          }
          else 
          {
            pos[channal][i]=positions[channal];
           //  Serial.println("Save");
           // Serial.println(pos[channal][i]);
            if(channal==0)
            {
              EEPROM.update((i+33),pos[channal][i]);
              EEPROM.update((i+3),0);
            }
            else if(channal==1)
            {
              EEPROM.update((i+93),pos[channal][i]);
              EEPROM.update((i+63),0);
            }
          }//for testing, use position -> show resault on diplay, but normally we need here setpostion
         // Serial.println("to je ono");
         // Serial.println(i);
          StorePostoRBut=false;
          MyReceiver.enableIRIn();
        }
        }
        else
        {
        if(MyDecoder.value==code[i])
        {
          if(pos[channal][i]<0)
          {
            setposition[channal]=pos[channal][i];
            direct=W;
            //Motor1();
            //n=((pos[channal][i]-128)*(-1));
            //Serial.println("chod na");
           // Serial.println(setposition[channal]);
          }
        
          else{
            setposition[channal]=pos[channal][i];
            direct=E;
           // Motor1();
           // Serial.println("chod na");
           // Serial.println(setposition[channal]);
          //n=pos[channal][i];
          }//for testing, use position -> show resault on diplay, but normally we need here setpostion
          
        }
       }
      }
      //MyDecoder.dumpResults();
      MyReceiver.enableIRIn();
      }
    }
  
  else if((button==6 || button==7)&& MyDecoder.value!=0xFFFFFFF)
  {
    //Serial.println("tu zastavujem");
     button=5;
      MyReceiver.enableIRIn();
  }
  
}

void Channal()
{
  if(channal==0)
        {
          
          n=2;
          channal=1;
          setDigit(n);
          while ((millis() - lastDebounceTime) < 1500)
          { 
          }
          
            //Serial.println(channal);
            //Serial.println("zobrazenie");
            n=positions[channal];
            button=0;
        }
        
        else if(channal==1)
        {
          
          n=1;
          channal=0;
          setDigit(n);
          while ((millis() - lastDebounceTime) < 1500)
          { 
          }
          
            //Serial.println(channal);
            //Serial.println("zobrazenie");
            n=positions[channal];
            button=0;
        }
      
}

void Error()
{ 
  
  //digitalWrite(LED, HIGH);
  //Serial.println("Doraz-LOW!");
    
  if(rot==1 && digitalRead(doraz[channal])==LOW)
  { digitalWrite(rele1[channal], LOW);
    digitalWrite(rele2[channal], LOW);
    digitalWrite(LED, HIGH);
    Serial.println(991);
    //setposition[channal]=positions[channal];
    rot=0;
    n=1;
  }
  
  else if(rot==1   && digitalRead(doraz[channal])==HIGH)
  { 
      digitalWrite(rele1[channal], LOW);
      digitalWrite(rele2[channal], LOW);
      digitalWrite(LED, HIGH);
      Serial.println(992);
     // Serial.println("HIGH!");    
     rot=0;
     n=2;
    
    
  }
  
  if(button==8)
  {
    Err=false;
    setposition[channal]=positions[channal];
    n=positions[channal];
     digitalWrite(LED, LOW);
     button=0;
  }
  
  
}
void EEPROMload() 
{ if(EEPROM.read(30)==0)
  {
    positions[1]=EEPROM.read(31); 
    n=positions[channal];
    setposition[1]=positions[1];
  }
  else if(EEPROM.read(31)==0)
  { 
    positions[1]=(EEPROM.read(30))*(-1); 
    n=positions[channal];
    setposition[1]=positions[1];
  }
  if(EEPROM.read(0)==0)
  {
    positions[0]=EEPROM.read(1); 
    n=positions[channal];
    setposition[0]=positions[0];
  }
  else if(EEPROM.read(1)==0)
  { 
    positions[0]=(EEPROM.read(0))*(-1); 
    n=positions[channal];
    setposition[0]=positions[0];
  }
  
  if(EEPROM.read(STEP)>0)
  setstep_size[0]=EEPROM.read(STEP);

  if(EEPROM.read(STEP+30)>0)
  setstep_size[1]=EEPROM.read(STEP+30);
  
  for(int i=0;i<24;i++)
  {
   if(EEPROM.read(i+3)==0)
   {
    pos[0][i]=EEPROM.read(i+33);
    
  }
    else if(EEPROM.read(i+33)==0)
  { 
    pos[0][i]=(EEPROM.read(i+3)*-1);
  }
 } 
 for(int i=0;i<24;i++)
  {
   if(EEPROM.read(i+63)==0)
   {
    pos[1][i]=EEPROM.read(i+93);
    
  }
    else if(EEPROM.read(i+93)==0)
  { 
    pos[1][i]=(EEPROM.read(i+63)*-1);
  }
 } 
}
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
