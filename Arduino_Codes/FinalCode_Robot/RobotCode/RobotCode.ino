// Encoder pins
#include <PinChangeInterrupt.h>  

//right side motor
#define MC_ENB 11
#define MC_IN3 12
#define MC_IN4 13 

//Left side motor
#define MC_ENA 10
#define MC_IN1 8
#define MC_IN2 9 


const int encoderAr = 6;
const int encoderBr = 7;

const int encoderAl = A1;
const int encoderBl = A0;

volatile float Kp = 1.0;
volatile float Ki = 0.25;
volatile float Kd = 0.1;

// Filter settings
const float ALPHA = 0.1;  // Smoothing factor
float smoothedRPMr = 0;    
float smoothedRPMl = 0;    

// Motor right control variables
volatile long encoderCountr = 0;
volatile float currentRPMr = 0.0;
int motorOutputr = 0;

// Motor left control variables
volatile long encoderCountl = 0;
volatile float currentRPMl = 0.0;
int motorOutputl = 0;

// right PID variables
float errorr = 0;
float lastErrorr = 0;
float integralr = 0;
float derivativer = 0;
float referenceRPMr = 0;

// left PID variables
float errorl = 0;
float lastErrorl = 0;
float integrall = 0;
float derivativel = 0;
float referenceRPMl = 0;

// Encoder state
volatile int lastEncAStater = LOW;
volatile int lastEncAStatel = LOW;

// Timing constants
const long SPEED_CALC_INTERVAL = 20000; // 20ms in microseconds
bool New_Current_Speed = false;

void InitPins();
void InitBluetooth();
void ParseBluetooth();
void DecodeBluetoothData();

void CalculateCurrentRPM();
void CalculateReferenceRPM();
void computePID();

short pitch = 0;
short roll = 0;

byte datapacket[13] = {0} ;  //array for storing the incoming bytes from the bluetooth

void setup() {
  // put your setup code here, to run once:
  InitPins();
  InitBluetooth();

  // Attach Pin Change Interrupt to encoderA (D6)
  attachPCINT(digitalPinToPCINT(encoderAr), encoderISRr, CHANGE);
  // Enable Pin Change Interrupt on A0 (PCINT8)
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);  // A0 is PCINT8
  PCICR   |= (1 << PCIE1);     // Enable PCINT1 group (A0-A5)
  pinMode(encoderAl, INPUT);
  pinMode(encoderBl, INPUT);
  // Disable ADC before digital read
  ADCSRA &= ~(1 << ADEN);  // Disable ADC
  
  lastEncAStater = digitalRead(encoderAr);
  lastEncAStatel = digitalRead(encoderAl);

 // Timer for speed calculation
  uint8_t oldSREG = SREG;
  TCCR0A = 0;
  TCCR0B = (1 << CS02) | (1 << CS00);

  // Calculate compare value for 20ms (16MHz clock)
  // 16,000,000 / 1024 = 15,625 Hz
  // 15,625 * 0.02s = 312.5 → OCR0A = 312 (20.032ms)
  OCR0A = 312;
  TIMSK0 |= (1 << OCIE0A);
  SREG = oldSREG;
 
  delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
  ParseBluetooth();
  DecodeBluetoothData();
  CalculateReferenceRPM();
  
   if(New_Current_Speed)
  {
    computePID();
    New_Current_Speed=false;
  }

  motorOutputr = constrain(motorOutputr, -255, 255);
  motorOutputl = constrain(motorOutputl, -255, 255);

  if(motorOutputr>=0)
  {
    digitalWrite(MC_IN4,HIGH);
    digitalWrite(MC_IN3,LOW);
    analogWrite(MC_ENB, motorOutputr);
  }
  
   if(motorOutputr<0)
  {
    digitalWrite(MC_IN4,LOW);
    digitalWrite(MC_IN3,HIGH);
    analogWrite(MC_ENB, -motorOutputr);
  }

   if(motorOutputl>=0)
  {
    digitalWrite(MC_IN2,HIGH);
    digitalWrite(MC_IN1,LOW);
    analogWrite(MC_ENA, motorOutputl);
  }
  
   if(motorOutputl<0)
  {
    digitalWrite(MC_IN2,LOW);
    digitalWrite(MC_IN1,HIGH);
    analogWrite(MC_ENA, -motorOutputl);
  }

  sendDataToVisualizer();
}

void InitPins()
{
  //Motor controller Pins Setup
  pinMode(MC_ENA,OUTPUT);  //ENA
  pinMode(MC_ENB,OUTPUT);  //ENB

  pinMode(MC_IN1,OUTPUT);  //IN1
  pinMode(MC_IN2,OUTPUT);  //IN2
  pinMode(MC_IN3,OUTPUT);  //IN3
  pinMode(MC_IN4,OUTPUT);  //IN4

  pinMode(encoderAr, INPUT);
  pinMode(encoderBr, INPUT);
  pinMode(encoderAl, INPUT);
  pinMode(encoderBl, INPUT);


  //Setting Initial values for the above pins
  analogWrite(MC_ENA,0);
  analogWrite(MC_ENB,0);

  digitalWrite(MC_IN1,LOW);
  digitalWrite(MC_IN2,LOW);
  digitalWrite(MC_IN3,LOW);
  digitalWrite(MC_IN4,LOW);


}

void InitBluetooth()
{
  //settig the bluetooth communication baudrate to be 9600
  Serial.begin(9600);
}

void ParseBluetooth()
{
 
  byte c; 
  bool start = false;
  int i = 0;
  for(;i<13;)
  {
    while(Serial.available()==0){}

    c = Serial.read();//reading the data in the serial buffer 
    
    if(c == '<')
    {
      start = true;
    }

    if(start == true)
    {
      //if c is a control charactor such as \0,\n we break from the loop

      datapacket[i] = c;//store the read byte
      
      //Serial.print(datapacket[i],HEX);
    
      i++;

      if(c == '>')
      {
        break;
      }
    }
    
  }

}

void DecodeBluetoothData()
{
  pitch = 0;
  roll = 0;

  pitch = datapacket[3]<<8;
  pitch = (pitch|datapacket[4]);

  roll = datapacket[8]<<8;
  roll = (roll|datapacket[9]);

}

void CalculateReferenceRPM()
{
  pitch = constrain(pitch,-60,60);  //clamp pitch 
  roll = constrain(roll,-60,60);  //clamp roll 

  referenceRPMr = (pitch + roll)*2;
  referenceRPMl = (pitch - roll)*2;

  //Reference RPM values
  referenceRPMr = constrain(referenceRPMr,-120,120);  
  referenceRPMl = constrain(referenceRPMl,-120,120); 

}

// PID calculation
void computePID() 
{
  noInterrupts();

  errorr = referenceRPMr - currentRPMr;
  errorl = referenceRPMl - currentRPMl;


  float Poutr = Kp * errorr;
  float Poutl = Kp * errorl;

  integralr += errorr;
  integrall += errorl;

  //integral anti-windup
  if ((motorOutputr >= 255 && errorr > 0) || (motorOutputr <= 0 && errorr < 0)) 
  {
    integralr = integralr - errorr * 0.1;
  }
  if ((motorOutputl >= 255 && errorl > 0) || (motorOutputl <= 0 && errorl < 0)) 
  {
    integrall = integrall - errorl * 0.1;
  }

  float Ioutr = Ki * integralr;
  float Ioutl = Ki * integrall;

  derivativer = (errorr - lastErrorr);
  derivativel = (errorl - lastErrorl);

  float Doutr = Kd * derivativer;
  float Doutl = Kd * derivativel;

  motorOutputr = Poutr + Ioutr + Doutr;
  motorOutputl = Poutl + Ioutl + Doutl;

  lastErrorr = errorr;
  lastErrorl = errorl;

  interrupts();
}

void sendDataToVisualizer()
{
  // Apply exponential moving average filter
  smoothedRPMr = ALPHA * currentRPMr + (1 - ALPHA) * smoothedRPMr;
  smoothedRPMl = ALPHA * currentRPMl + (1 - ALPHA) * smoothedRPMl;

  Serial.print(referenceRPMr);
  Serial.print(",");
  Serial.print(smoothedRPMr);
  Serial.print(",");
  Serial.print(referenceRPMl);
  Serial.print(",");
  Serial.println(smoothedRPMl);
}

// Timer ISR for speed calculation
ISR(TIMER0_COMPA_vect)
{
  currentRPMr = (encoderCountr / 400.0) * (60000000.0 / SPEED_CALC_INTERVAL);
  encoderCountr=0;
  currentRPMl = (encoderCountl / 400.0) * (60000000.0 / SPEED_CALC_INTERVAL);
  encoderCountl=0;
  New_Current_Speed = true;
}

// **Pin Change Interrupt for right encoder counting**
void encoderISRr()
{
  int currentEncAStater = digitalRead(encoderAr);
  if (currentEncAStater == HIGH && lastEncAStater == LOW) 
  {
    if (digitalRead(encoderBr) == LOW) {
      encoderCountr++;
    } else {
      encoderCountr--;
    }
  }
  lastEncAStater = currentEncAStater;
}

// **Pin Change Interrupt for left encoder counting**
ISR(PCINT1_vect)
{
  int currentEncAStatel = digitalRead(encoderAl);
  if (currentEncAStatel == HIGH && lastEncAStatel == LOW) 
  {
    if (digitalRead(encoderBl) == LOW) {
      encoderCountl++;
    } else {
      encoderCountl--;
    }
  }
  lastEncAStatel = currentEncAStatel;
}




