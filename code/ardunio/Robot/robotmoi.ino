

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


int Enable1 = 9;
int Enable2 = 10;
int condition;
int dis;
int loi;


int IN1 = A0;
int IN2 = A1;
int IN3 = A2;
int IN4 = A3;
int Sp1;
int Sp2;


unsigned long LastReceivedTime = 0;
unsigned long CurrentTime = 0;

RF24 radio(7, 8);              // nRF24L01 (CE, CSN)
const byte address[6] = "00001";

// creating Structure Data
struct Data_Pack
{
  byte EN1_Speed;
  byte EN2_Speed;
  byte huong;
      byte distance;
      byte error;
};
Data_Pack data; 
void setup()
{
  Serial.begin(9600);
  pinMode(Enable1, OUTPUT);
  pinMode(Enable2, OUTPUT);

  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  radio.begin(); 
 
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); 
  delay(500);

}
void loop() 
{
  CurrentTime = millis();
if (CurrentTime - LastReceivedTime > 200)

  {
    Serial.println("");
    dung(0, 0);
    condition =6;
    delay(10); 
  }
if (radio.available())
  {
    radio.read(&data, sizeof(Data_Pack));
    Sp1 = data.EN1_Speed;
    Sp2 = data.EN2_Speed;
    //dis=data.distance;
    //loi=data.error;
    condition = data.huong;
    LastReceivedTime = millis();

  }
if (condition == 3) 
  {
      Right(Sp1, Sp2);
    //  Serial.print(Sp1); Serial.print(',');Serial.print(Sp2); Serial.print(',');Serial.print(dis); Serial.print(',');Serial.println(-loi); 
      condition =6;
      delay(10); 
      
  }
  if (condition == 4) 
  {
      Left(Sp1, Sp2);
   //   Serial.print(Sp1); Serial.print(',');Serial.print(Sp2); Serial.print(',');Serial.print(dis); Serial.print(',');Serial.println(loi); 
      condition =6;
      delay(10); 
      
  }
  if (condition == 1) //lui
  {
      lui(Sp1, Sp2);
      
   //   Serial.print(Sp1); Serial.print(',');Serial.print(Sp2+5); Serial.print(',');Serial.print(dis); Serial.print(',');Serial.println(loi); 
  
      condition =6;
      delay(10); 
      
  }
  if (condition == 2) //tien
  {
      tien(Sp1, Sp2);
    
     // Serial.print(Sp1);Serial.print(',');
    //  Serial.print(Sp2+5); Serial.print(',');Serial.print(dis); Serial.print(',');Serial.println(loi); 
      condition =6;
      delay(10);  
  }
  
  if (condition == 0)//Dung dong co
  {
       
      dung(0, 0);
    //  Serial.print(0);Serial.print(',');
   //   Serial.print(0); Serial.print(',');Serial.print(dis); Serial.print(',');Serial.println(loi); 
      delay(10);
      condition =6;
      delay(10); 
  }



}
void tien(int Speed1, int Speed2)
{
  analogWrite(Enable1, Speed1);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(Enable2, Speed2);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void lui(int Speed1, int Speed2)
{
  analogWrite(Enable1, Speed1);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(Enable2, Speed2);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void dung(int Speed1, int Speed2)
{
  analogWrite(Enable1, Speed1);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(Enable2, Speed2);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void Right(int Speed1, int Speed2)
{
  analogWrite(Enable1, Speed1);
  //    Serial.println("Move Left");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(Enable2, Speed2);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void Left(int Speed1, int Speed2)
{
  analogWrite(Enable1, Speed1);
  //  Serial.println("Move Right");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  analogWrite(Enable2, Speed2);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
