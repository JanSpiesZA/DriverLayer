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
#define TimeOut 50

//## This array holds the adresses of the sensors in the sequence with which each one will be pinged
int sensorAddr[] = {3,0,4,1,5,2,6};
//## The function SIZEOF gives the amount of bytes used in the array and not the actual elements. 
//##    When using int every element is 2 bytes long therefore the value must be divided by the siezeof(int) 
//##    to get the total amount of elements in the array
const int numSensors = sizeof(sensorAddr)/ sizeof(int);    
unsigned int sensorDist[numSensors];    //Array used to store the values of the distances measured
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

//Program flow:
//  Ping [n]
//  Get [n-1]
//  Send [n-1]

void loop()
{
//  time = millis();

  for (int n = 0; n < numSensors; n++)
  {
    ping (sensorAddr[n]);

    int k = n - 1;
    if (n == 0) k = numSensors - 1;    

    int tmpRange = getRange(sensorAddr[k]);
    PCComms.print("d");
    PCComms.print(sensorAddr[k]);
    PCComms.print(":");
    PCComms.println(tmpRange);    
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
  static unsigned long timeLastInput = 0;
  unsigned long now;
  bool boolTimeOut = false;
  unsigned int rxData = 0;
  
//  Serial.print("Requesting data from sensor: #");
//  Serial.println(_sensorAddr);
  
  USComms.write(_sensorAddr);
  USComms.write(CMD_GET_RANGE);
  
  timeLastInput = millis();   //Set variable to current time before going into the timeout loop  
  while ((USComms.available() < 2) && (!boolTimeOut))
  {
    now = millis();  
    if (now - timeLastInput > TimeOut) boolTimeOut = true;          
  }

  if (USComms.available() == 2)
  {
    rxData = USComms.read() << 8;
    rxData |= USComms.read();    
  }
  
  return rxData;
}

















