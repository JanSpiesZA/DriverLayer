//Code for ultrasonic SRF02 modules on a Arduino Mega board
//All sensors are in series and will be polled when distance data is needed

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//Sensors must be pre-programmed with unique IDs


#define CMD_REAL_RANGE_TX_cm      84
#define CMD_REAL_RANGE_NO_TX_cm   81
#define CMD_GET_RANGE             94

//## Defines the serial ports and baud rates used by the program
#define PCComms Serial
#define PCBaud 115200
#define USComms Serial2
#define USBaud 9600

//### Time variables used to ping each ultrasound sensor
unsigned long int oldTimePing = 0;
int deltaPing = 49;

//### Time variables used to timewhen new data must be sent serially
unsigned long int oldTimeTX = 0;
int deltaTX = 299;

unsigned long int time = 0;

//## This array holds the adresses of the sensors in the sequence with which each one will be pinged
int sensorAddr[] = {3,0,4,1,5,2,6};
//## The function SIZEOF gives the amount of bytes used in the array and not the actual elements. 
//##    When using int every element is 2 bytes long therefore the value must be divided by the siezeof(int) 
//##    to get the total amount of elements in the array
const int numSensors = sizeof(sensorAddr)/ sizeof(int);    
unsigned int sensorDist[numSensors];
int sensorCnt = 0; //Cntr used to keep track of the sensors ping'ed
unsigned int cntr = 0;

void setup()
{
  //## This serial port will be used to communicate with the algorithm layer
  PCComms.begin(PCBaud);
  PCComms.println("PC comms started:"); 
  //## Starts the serial port connected to the SRF sensors
  USComms.begin(USBaud); 
  //## Sends OK signal to PC after USComms started
  PCComms.println("US Serial port Started:"); 
  
}

void loop()
{
  time = millis();
  
  //### This routine sends serial data to PC every deltaTX time    
  int newTimeInterval = time - oldTimeTX;
  if (newTimeInterval > deltaTX)
  {
    //Serial.println(" TX Data ");    
    sendSensor();
    
    //###Tell sensors to determine range to nearest obstacle
//    for (int n = 0; n <= numSensors-1; n++)
//      { 
//        ping(sensorAddr[n]);
//        delay(20);
//      }
    oldTimeTX = time;
    delay(10);
    //Serial.println("Exit: send data");
  }



  //## This routine sends a ping request to a different SRF every deltaPing millis
  int newPingInterval = time - oldTimePing;
  if (newPingInterval > deltaPing)
  {    
    //Serial.print("PING ");
    if (sensorCnt <= numSensors-1)
    {      
      //Serial.print (sensorCnt);
      ping(sensorAddr[sensorCnt]);
      sensorCnt ++;
      //delay(1);
    }
    else
    {
      sensorCnt = 0;
    }
    oldTimePing = time;    
  }
  
}

//////////////////////////////////////////////////////////////////////////////////////
// Funtions and Procedures start here

//### Tasks a sensor in calculating range and holding range data in buffer
void ping(int _sensorAddr)
{
//   Serial.print("PING Sensor with adr: #");
//   Serial.println(_sensorAddr);
   USComms.write(_sensorAddr);
   USComms.write(CMD_REAL_RANGE_NO_TX_cm);
}


//###Reads range data from sensor buffer
int getRange(int _sensorAddr)
{  
//  Serial.print("Requesting data from sensor: #");
//  Serial.println(_sensorAddr);
  
  USComms.write(_sensorAddr);
  USComms.write(CMD_GET_RANGE);
  
  //int startTime = millis();
  while((USComms.available() < 2));
  
  unsigned int rxData = USComms.read()<<8;
  rxData |= USComms.read();
  //Serial.println("RX'd");
  return rxData;
}

//void serialEvent2()
//{
//  while (Serial2.available()
//  {
//    char 
//}

//###Send sensor data
void sendSensor()
{
  //###Read sensor distance data from all sensors
  for (int n = 0; n <= numSensors - 1; n++)
  {
    if (n != sensorCnt)
    {
      sensorDist[n] = getRange(sensorAddr[n]);
      delay(10);
    }    
  }

//  PCComms.println(cntr);
//  cntr++;
  //###Send sensor distance to PC
  PCComms.print('d');
  for (int n = 0; n <= numSensors-1; n++)
  {     
    PCComms.print(sensorAddr[n]);
    PCComms.print(':');
    PCComms.print(sensorDist[n]);  
    PCComms.print(',');    
    //delay(10);
  }
  PCComms.println();
}

















