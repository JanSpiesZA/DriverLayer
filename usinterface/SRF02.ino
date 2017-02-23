#define CMD_REAL_RANGE_TX_cm      84
#define CMD_REAL_RANGE_NO_TX_cm   81
#define CMD_GET_RANGE             94

//Serial port must be setup in setup() program
//      Serial3.begin(9600);  -- Use the serial port connected to the SRF sensors

//### Tasks a sensor in calculating range and holding range data in buffer
void ping(int _sensorAddr)
{
//   Serial.print("PING Sensor with adr: #");
//   Serial.println(_sensorAddr);
   Serial2.write(_sensorAddr);
   Serial2.write(CMD_REAL_RANGE_NO_TX_cm);
}


//###Reads range data from sensor buffer
int getRange(int _sensorAddr)
{  
//  Serial.print("Requesting data from sensor: #");
//  Serial.println(_sensorAddr);
  
  Serial2.write(_sensorAddr);
  Serial2.write(CMD_GET_RANGE);
  
  //int startTime = millis();
  while((Serial2.available() < 2));
  
  int rxData = Serial2.read()<<8;
  rxData |= Serial2.read();
  //Serial.println("RX'd");
  return rxData;
}

//void serialEvent2()
//{
//  while (Serial2.available()
//  {
//    char 
//}

















