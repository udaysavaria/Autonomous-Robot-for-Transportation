/*
Autonomous Robot For Transportation

connections

Encoders 
left Encoder    -    Arduino Pin 2
Right Encoder   -    Arduino Pin 3

Motors
Left Motor Positive   -   Arduino Pin 10
Left Motor Negative   -   Arduino Pin 11
Right Motor Positive  -   Arduino Pin 5
Right Motor Negative  -   Arduino Pin 6

Ir Tx-Rx Used for Accident Avoidance
Ir Sensor Pin  -    Arduino pin A0

*/
#include "PinChangeInt.h"

#define Ir0 A0
#define GO 9
#define IrThresold 1000
#define left true
#define right false
#define forw true
#define back false

int le = 2, re = 3, lf = 10, lr = 11, rf = 5, rr = 6; // pin for encoder n motor
int cl = 0, cr = 0;
unsigned char sl = 190, sr = 255;
char ptemp = '0', ploc = '1';
int distCount = 0,angleCount = 0;
float tempCount = 0;


void countDistance();
void encoderl();
void encoderr();
void motor(boolean sel, boolean dir);
void mov(boolean drive, int countGiven);
void forward(int distInCent);
void backward(int distInCent);
void stopit();
void turn(boolean rot, float a);
void senddata();
void wait();


inline void wait()
{
  int val0;
  do
  {
    val0 = analogRead(Ir0);

    if (val0 <= IrThresold)
    {
      analogWrite(lf, 255);
      analogWrite(lr, 255);
      analogWrite(rf, 255);
      analogWrite(rr, 255);
    }
  }
  while (val0 <= IrThresold);
}

//encoder reading
inline void encoderl()
{
  cl++;
}

inline void encoderr()
{
  cr++;
}

//robot turning
inline void turn(boolean rot, float a)
{
  tempCount = 4 * (0.2444445) * a;    ///// 0.2444445 count per angle change
  angleCount = (int)tempCount;
  
  while (cl <= angleCount && cr <= angleCount)
  {
    //wait();
    if (left)
    {
      analogWrite(lf, 0);
      analogWrite(lr, sl);
      analogWrite(rf, sr);
      analogWrite(rr, 0);
    }
    else
    {
      analogWrite(lf, sl);
      analogWrite(lr, 0);
      analogWrite(rf, 0);
      analogWrite(rr, sr);
    }
  }
  delay(10);
  stopit();
  delay(50);
}

//forward drive
inline void forward(int distInCent)
{

  tempCount = 4 * (distInCent * 0.86705202);      // 30/34.6  = 0.86705202
  distCount = (int)tempCount;
  Serial.println(distCount);
  while (cl <= distCount && cr <= distCount)
  {
    //wait();
    analogWrite(lf, sl);
    analogWrite(lr, 0);
    analogWrite(rf, sr);
    analogWrite(rr, 0);
  }
  delay(10);
  stopit();
  delay(50);

}

//backward drive
inline void backward(int distInCent)
{
  tempCount = 4 * (distInCent * 0.86705202);      // 30/34.6  = 0.86705202
  distCount = (int)tempCount;

  while (cl <= distCount && cr <= distCount)
  {
    //wait();
    analogWrite(lf, 0);
    analogWrite(lr, sl);
    analogWrite(rf, 0);
    analogWrite(rr, sr);
  }
  delay(10);
  stopit();
  delay(50);
}

//stop robot
inline void stopit()
{
  Serial.println(cl);
  analogWrite(lf, 255);
  analogWrite(lr, 255);
  analogWrite(rf, 255);
  analogWrite(rr, 255);
  cl = 0;
  cr = 0;
}

inline void senddata()
{
  ploc = ptemp;
  Serial.print("Reached to Location No. ");
  Serial.println(ploc);

}

void setup()
{
  //encoder and motor setup
  pinMode(le, INPUT);
  digitalWrite(le, HIGH);
  pinMode(re, INPUT);
  digitalWrite(re, HIGH);

// motor pins
  pinMode(lf, OUTPUT);
  digitalWrite(lf, LOW);
  pinMode(lr, OUTPUT);
  digitalWrite(lr, LOW);
  pinMode(rf, OUTPUT);
  digitalWrite(rf, LOW);
  pinMode(rr, OUTPUT);
  digitalWrite(rr, LOW);

  //interrupt n serial setup
  Serial.begin(2400);
  attachInterrupt(0, encoderl, FALLING);
  attachInterrupt(1, encoderr, FALLING);

  //pin change interrupt
  pinMode(GO, INPUT);
  digitalWrite(GO, HIGH);
  PCintPort::attachInterrupt(GO, &senddata, FALLING);
}




void loop()
{
  if (Serial.available())
  {
    if (ploc == '1')                   //for present location is 1
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case  '2':
          forward(100);
          turn(left, 90);
          forward(114);
          turn(left, 90);
          ptemp = '2';
          break;
        case '3':
          forward(144);
          turn(left, 90);
          forward(114);
          forward(198);
          turn(left, 90);
          ptemp = '3';
          break;
        case '4':
          forward(144);
          turn(left, 90);
          forward(114);
          forward(198);
          turn(right, 90);
          forward(225);
          ptemp = '4';
          break;
        case '5':
          forward(144);
          turn(left, 90);
          forward(114);
          turn(right, 90);
          forward(225);
          ptemp = '5';
          break;
        case '6':
          forward(144);
          turn(left, 90);
          forward(114);
          forward(201);
          turn(right, 90);
          forward(114);
          turn(left, 90);
          ptemp = '6';
          break;
        default:
          stopit();
          break;
      }
    }
    if (ploc == '2')                   //for present location is 2
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case '1':
          turn(left, 90);
          forward(114);
          turn(right, 90);
          forward(144);
          ptemp = '1';
          break;
        case '3':
          turn(right, 90);
          forward(201);
          forward(198);
          turn(left, 90);
          ptemp = '3';
          break;
        case '4':
          delay(30);
          Serial.write('4');
          turn(right, 90);
          Serial.write('4');
          forward(201);
          Serial.write('4');
          forward(198);
          Serial.write('4');
          turn(right, 90);
          Serial.write('4');
          forward(225);
          ptemp = '4';
          break;
        case '5':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          ptemp = '5';
          break;
        case '6':
          turn(right, 90);
          forward(201);
          turn(right, 90);
          forward(114);
          turn(left, 90);
          ptemp = '6';
          break;
        default:
          stopit();
          break;
      }
    }
    if (ploc == '3')                  //for present location is 3
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case '2':
          turn(left, 90);
          forward(198);
          forward(201);
          turn(right, 90);
          ptemp = '2';
          break;
        case '1':
          turn(left, 90);
          forward(198);
          forward(201);
          forward(114);
          turn(right, 90);
          forward(144);
          ptemp = '1';
          break;
        case '4':
          turn(left, 90);
          turn(left, 90);
          forward(225);
          ptemp = '4';
          break;
        case '5':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          turn(right, 90);
          forward(198);
          forward(201);
          turn(left, 90);
          ptemp = '5';
          break;
        case '6':
          turn(left, 90);
          forward(198);
          turn(left, 90);
          forward(114);
          turn(left, 90);
          ptemp = '6';
          break;
        default:
          stopit();
          break;
      }
    }
    if (ploc == '4')                 //for present location is 4
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case '3':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          ptemp = '3';
          break;
        case '2':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          turn(left, 90);
          forward(198);
          forward(201);
          turn(right, 90);
          ptemp = '2';
          break;
        case '1':
          //     Serial.println("Ploc 4- 1");
          turn(right, 90);
          turn(right, 90);
          forward(225);
          turn(left, 90);
          forward(198);
          forward(201);
          forward(114);
          turn(right, 90);
          forward(144);
          ptemp = '1';
          break;
        case '5':
          turn(right, 90);
          forward(198);
          forward(201);
          turn(left, 90);
          ptemp = '5';
          break;
        case '6':
          turn(right, 90);
          forward(198);
          turn(right, 90);
          forward(111);
          turn(right, 90);
          ptemp = '6';
          break;
        default:
          stopit();
          break;
      }
    }
    if (ploc == '5')                //for present location is 5
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case '4':
          turn(left, 90);
          forward(201);
          forward(198);
          turn(right, 90);
          ptemp = '4';
          break;
        case '3':
          turn(left, 90);
          forward(201);
          forward(198);
          turn(left, 90);
          forward(225);
          ptemp = '3';
          break;
        case '2':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          ptemp = '2';
          break;
        case '1':
          turn(right, 90);
          turn(right, 90);
          forward(225);
          turn(left, 90);
          forward(114);
          turn(right, 90);
          forward(144);
          ptemp = '1';
          break;
        case '6':
          turn(left, 90);
          forward(201);
          turn(left, 90);
          forward(111);
          turn(right, 90);
          ptemp = '6';
          break;
        default:
          stopit();
          break;
      }
    }
    if (ploc == '6')                //for present location is 6
    {
      switch (Serial.read())            // read the next location from xbee
      {
        case '5':
          turn(right, 90);
          forward(111);
          turn(right, 90);
          forward(201);
          turn(left, 90);
          ptemp = '5';
          break;
        case '4':
          turn(right, 90);
          forward(111);
          turn(left, 90);
          forward(198);
          turn(right, 90);
          ptemp = '4';
          break;
        case '3':
          //     Serial.println("Ploc 6- 3");
          turn(left, 90);
          forward(114);
          turn(right, 90);
          forward(198);
          turn(left, 90);
          ptemp = '3';
          break;
        case '2':
          turn(left, 90);
          forward(114);
          turn(left, 90);
          forward(201);
          turn(right, 90);
          ptemp = '2';
          break;
        case '1':
          turn(left, 90);
          forward(114);
          turn(left, 90);
          forward(201);
          forward(114);
          turn(left, 90);
          forward(144);
          ptemp = '1';
          break;
        default:
          stopit();
          break;
      }
    }
  }
}


