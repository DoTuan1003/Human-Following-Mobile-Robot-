
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001"; // Address
int huong;
int Speed1;
int Speed2;
struct Data_Pack {
    byte EN1_Speed;
    byte EN2_Speed;
    byte huong;
    byte distance;
 
}; 
Data_Pack data;
void setup()
{
    Serial.begin(9600);
    radio.begin(); 
    radio.openWritingPipe(address);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    
}

void loop()
{
 if (Serial.available()>0){
  if (Serial.read()=='C')
    {
  data.distance = Serial.parseInt();
    }  
    if (Serial.read()=='A')
    {
    data.EN1_Speed = Serial.parseInt();   
    }
  
    if (Serial.read()=='B')
    {
  data.EN2_Speed = Serial.parseInt();
    }
    if (Serial.read()=='D')
      {
    data.huong= Serial.parseInt();
      }
    radio.write(&data, sizeof(Data_Pack));
    delay(5);
 }
}
