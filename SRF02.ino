#define CMD_REAL_RANGE_TX_cm      84
#define CMD_REAL_RANGE_NO_TX_cm   81
#define CMD_GET_RANGE             94

//Serial port must be setup in setup() program
//      Serial3.begin(9600);  -- Use the serial port connected to the SRF sensors

void ping(int _sensorAddr)
{
   //Serial.print("PING Sensor with adr: #");
   //Serial.println(_sensorAddr);
   Serial3.write(_sensorAddr);
   Serial3.write(CMD_REAL_RANGE_NO_TX_cm);
}

int getRange(int _sensorAddr)
{
  //Serial.print("Requesting data from sensor: #");
  //Serial.println(_sensorAddr);
  Serial3.write(_sensorAddr);
  Serial3.write(CMD_GET_RANGE);

  while(Serial3.available() < 2);     //Wait until 2 bytes received
  int rxData = Serial3.read()<<8;
  rxData |= Serial3.read();
  return rxData;
}


