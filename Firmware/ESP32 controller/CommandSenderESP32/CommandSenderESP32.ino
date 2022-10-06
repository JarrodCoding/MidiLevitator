 // --------------------------------------------------
//
// Simple test for Ableton <> Arduino connection
//     -> where and 8-bit control byte and a 16-bit 
//        value is sent back and forth between 
//        Ableton and Arduino-compatible HW.
//
// Four bytes are expected from Ableton
// Byte 1: 255 (sync byte)
// Byte 2: <control> - [0:255]
// Byte 3: <value MSB> - [0:255]
// Byte 4: <value LSB> - [0:255]
//
// --------------------------------------------------

#define CONNECTED_TIMEOUT 500000 // 0.5 second time for LED to indicate a command has been received

// for ESP32 define built-in LED (uncomment the statement below if you are using ESP32) -> Thanks to Ben Coleman for this addition :-)
 #define LED_BUILTIN 2

// internal variables
int rx_state = 0;
byte cc_type1;
byte cc_val1;
byte cc_val2;
byte cc_val3;
byte cc_val4;
byte cc_val5;

// needed for indication LED (goes on on arduino when serial info is received)
unsigned long   t = 0;                // current time
unsigned long   serial_t0 = 0;     // last time serial info was received

// simple function to merge most significant byte and least significant byte to single int
int bytesToInt(int l_highByte, int l_lowByte) {
  return ((unsigned int)l_highByte << 8) + l_lowByte;
}
#include "WiFi.h"

#include "EmitterPos.h"
#include "FocusAlg.h"
#include "PointsMov.h"

#define ESPFOCAL_VERSION "0.0"
#define stepSize 0.001
#define frameTime 25
#define BUTTON_PIN 12

//focal point current positons
float xCurrent = 0.0;
float yCurrent = ARRAY_HEIGHT/2;
float zCurrent = 0.0;

//activate certain operation modes
int modes = 0; //dial default

/* Array control functions */
int ibpIterations = 4;
byte buff[N_EMITTERS]; //phases to be emitted
float pointPos[MAX_POINTS * 3];


void switchOnArray(boolean onTop, boolean onBottom);
void switchBuffer();
void sendBuffer();
void sendPhases(String command);

void processCommand(String command);

int parseFloats(String s, int startIndex, float target[], int maxNumbers);

//animation functions
void animateDials(int valueX, int valueY, int valueZ);
void animateBPM(int bpm);
void animateKeyboard(int key);
void animateNotes(int key, int velocity);
//testing functions
void bufferLatency();
void setXZ(float minimumX, float minimumZ);
void moveObj(float newXPos, float newYPos, float newZPos);
void testAxis();
/* Network functions*/

//const char* ssid = "FOCAL_ESP";
//const char* password = "12345678";
//WiFiServer server(80);

//void configWifiAsAccessPoint(const char* ssidName) {
//  Serial.println("Configuring as access point...");
//  WiFi.mode(WIFI_AP);
//  WiFi.softAP( ssidName, password );
//  Serial.println("Wait 100 ms for AP_START...");
//  delay(100);
//
//  Serial.println("Set softAPConfig");
//  IPAddress Ip(192, 168, 1, 1);
//  IPAddress NMask(255, 255, 255, 0);
//  WiFi.softAPConfig(Ip, Ip, NMask);
//  IPAddress myIP = WiFi.softAPIP();
//  Serial.print("AP IP address: ");
//  Serial.println(myIP);
//}


/* Setup*/
void setup() {
  //Serial.begin(230400); // this serial is for receiving commands and sending debug info to the computer
  Serial2.begin(230400, SERIAL_8N1, 19,21); //RX2 TX2 This serial communicates with the array
//additional setup code
  pinMode(LED_BUILTIN, OUTPUT);   // initialize LED for serial connection verification
  Serial1.begin(115200, SERIAL_8N1, 5, 18); // IMPORTANT: in MAX the same data rate must be used! Feel free to experiment with higher rates.
  Serial.begin(115200);
  Serial.println("I'm working");
  
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
Serial.println(ESPFOCAL_VERSION);
//  Serial.println( ssid );

//  configWifiAsAccessPoint( ssid ); //start as access point with regular ssid
//  server.begin();
  delay(200);

  Serial2.write( 151 ); //switch on the FPGA clocks (in case that they were off with the 150 command send at random during start up)
  Serial.println("single node in center position...");
  //switchOnArray(false, false);
  
  ibp_initEmitters();
  String resetter="focus=" + String(xCurrent,7) + " " + String(yCurrent,7) + " " + String(zCurrent,7) + " ";
   processCommand(resetter);
   //bufferLatency();

}


void focusArrayAtPoints() {
  //this is the IBP method. slow but stronger trapping points. hard to join at closer distances.
  ibp_initPropagators(2, pointPos);
  for (int i = 0; i < ibpIterations; ++i)
    ibp_iterate( 2 );
  ibp_applySolution(2, buff);
  
  
  //this is the other method. faster, works at closer distances but not as strong
  //multiFocusAt(2,  pointPos,  buff);

  sendBuffer();
}

int previousButton = HIGH;
int cyclesButtonHigh = 0;
enum ePointsStatus {statusOff, statusFocus, statusMoving};
ePointsStatus currentStatus = statusOff;
void loop() {

  /* button press */
  const int currentButtonState = digitalRead(BUTTON_PIN);
  if (previousButton == HIGH && currentButtonState == LOW && cyclesButtonHigh > 100) { //when the button is pressed. last part is for debouncing
    cyclesButtonHigh = 0;
    /* button has been pressed*/
    Serial.println("Button has been pressed");
    if (currentStatus == statusOff) {
      Serial.println("Focusing points");
      currentStatus = statusFocus;
      points_reset(2, pointPos, POINTS_MOV_SEP, ARRAY_HEIGHT/2 );
      focusArrayAtPoints();
    } else if (currentStatus == statusFocus) {
      Serial.println("Moving");
      currentStatus = statusMoving;
    } else if (currentStatus == statusMoving) {
      Serial.println("Off");
      currentStatus = statusOff;
      switchOnArray(false, false);
    }
  }
  if (currentButtonState == HIGH && cyclesButtonHigh < 0xFF) {
    cyclesButtonHigh += 1;
  } else if (currentButtonState == LOW) {
    cyclesButtonHigh = 0;
  }
  previousButton = currentButtonState;

  if (currentStatus == statusMoving) {
    points_bringCloser(2,   pointPos, POINTS_MOV_ATTRACTION*2);
    points_rotateAroundY(2,   pointPos, POINTS_MOV_ROT*2);
    focusArrayAtPoints();
    //delay(25);
  }

  /*Commands comming from Serial*/
  if ( Serial.available() ) {
    String command = Serial.readStringUntil('\n');
    processCommand( command);
  }
  /* Commands comming from HTTP */
//  WiFiClient client = server.available();
//  if (client) {
//    String request = client.readStringUntil('\r');
//    client.println("HTTP/1.1 200 OK\nContent-type:text/html\nConnection: close\n\n");
//
//    const int sep1 = request.indexOf('/');
//    if (sep1 != -1) {
//      String command = request.substring(sep1 + 1);
//      processCommand( command );
//    }
//    client.stop();
//  }
    t = micros();           // take timestamp

  //-----------------------------update timeout led--------------------------------//
  if (digitalRead(LED_BUILTIN))
    if((t - serial_t0) > CONNECTED_TIMEOUT)
      digitalWrite(LED_BUILTIN, 0);

  //----------Check if control commands have been received from Ableton------------//
  int timecheck = millis();
  int timeBetweenReads=500;


    if (Serial1.available()) {
    serial_t0 = t;
    if (digitalRead(LED_BUILTIN) == 0)
      digitalWrite(LED_BUILTIN, 1);
 

      
  rx_state++;
    switch (rx_state) {
      case 1:                     // first byte is always 255 for sync 
        cc_type1 = Serial1.read();
        if(cc_type1 != 255) {     // reset if first is not 255 sync byte
          rx_state = 0;
        }
        break;
      case 2:                     // second is the control byte
        cc_val1 = Serial1.read();
        break;        
      case 3:                     // third is the most significant byte of the value
        cc_val2 = Serial1.read();     
        break;
      case 4:                     // fourth is the least significant byte of the value
        cc_val3 = Serial1.read();
        break;
      case 5:
        cc_val4=Serial1.read();
        break;
      case 6:
        cc_val5=Serial1.read();
        rx_state = 0;
      
        

//}
     if(!Serial1.available()){
        int xAxis =  cc_val1;
        int yAxis =  cc_val2;
        int zAxis =  cc_val3;
        int note = cc_val4;
        int velocity = cc_val5;
//        Serial.print(xAxis);  // second byte is the control byte
//        Serial.print(" ");
//        Serial.print(yAxis);  // third byte is the most significant byte of the value
//        Serial.print(" ");
//        Serial.print(zAxis); // fourth byte is the least significant byte of the value
//        Serial.print (" ");
//        Serial.print(note);
//        Serial.print(" ");
//        Serial.println(velocity);

        if (modes==0){
           animateDials(xAxis,yAxis,zAxis);
        }
        else if (modes==1){
          animateKeyboard(note);
        }
        else if(modes==2){
          animateDials(note,yAxis,zAxis);
        }
      
        
     }

        // This program simply echos the received commends back to Ableton (normally you would not do this as it loads the serial port)
//        Serial.print(255);      // first byte is 255 (could have written cc_type1 here as well
//        Serial.print(" ");

        
        // At this location you can define what the Arduino should do with the received command
        //0 0.1215 0 
        //Serial.printf("My values being displayed are %d %d %d for X Y and Z\n",xAxis, yAxis, zAxis);
        
        break;
    }
  }

 

  
}
void animateDials(int valueX,int valueY,int valueZ){
  //define ranges in metres
  float xMin = -0.65;
  float xMax = 0.65;
  float xRange = xMax - xMin;
  float yMin = ARRAY_HEIGHT/2-0.065;
  float yMax = ARRAY_HEIGHT/2+0.065;
  float yRange = yMax - yMin;
  float zMin = -0.065;
  float zMax = 0.065;
  float zRange = zMax - zMin;
  //setting step size and positions 
  int numSteps = 128;
  float xStep = xRange/numSteps;
  float yStep = yRange/numSteps;
  float zStep = zRange/numSteps;

  float newXPos = xMin + xStep * valueX;
  float newYPos = yMin + yStep * valueY;
  float newZPos = zMin + zStep * valueZ;
  Serial.printf("My new positions: %f %f %f\n", newXPos, newYPos, newZPos);
  
  //total time for this animation before reading next value 
  int xOldByte = (xCurrent - xMin)/xStep;
  int yOldByte = (yCurrent - yMin)/yStep;
  int zOldByte = (zCurrent - zMin)/zStep;

  
  //check stuff
//  Serial.print("heres some cool values");
//  Serial.print(xRange);
//  Serial.print(yRange);
//  Serial.printf("stepx: %f",xStep);
//  Serial.printf("Stepy %f",yStep);
//  Serial.printf("stepz: %f",zStep);
//  Serial.println("positions: ");
//  Serial.print(newXPos);
//  Serial.printf("Y %f" , newYPos);
//  Serial.printf("Z %f", newZPos);
//  Serial.printf("time for animation: %d",totalTimeBetweenMeasurements);
  
  //while loop for animation
  moveObj(newXPos, newYPos,newZPos);
  
  
 Serial.printf("I'm stationary at: %f %f %f\n",xCurrent, yCurrent, zCurrent);

}
//animate keys in single x axis
void animateKeyboard(int key){
  float xMin = -0.065;
  float xMax = 0.065;
  float xRange = xMax - xMin;
  //setting up new position
  int numSteps = 128;
  float xStep = xRange/numSteps;
  float newXPos = xMin + xStep * key;
   
  
 //while loop for animation
  moveObj(newXPos,yCurrent,zCurrent);

  Serial.printf("I'm stationary at: %f %f %f\n",xCurrent, yCurrent, zCurrent);
}


void animateBPM(int bpm){ 
 Serial.printf("animating bpm %d\n",bpm);
 String intialSetup="focus=" + String(xCurrent,7) + " " + String(yCurrent,7) + " " + String(zCurrent,7) + " ";
 processCommand(intialSetup);
 int numSteps=2;
 //float stepSize=0.001;
 int minFrameTime=200;
 int minAnimationTime=numSteps*2*minFrameTime;
 float center=xCurrent;
 int switcher=0;
 int frameComplete=0;
 int maxBPM=70;//round(60/(minAnimationTime/1000));
 int animationStart=1;
 //Serial.printf("Maximum bpm is %d\n",maxBPM);
if (bpm>maxBPM){
  Serial.printf("BPM set too high, maximum bpm is %d\n",maxBPM);
  animationStart=0;
}
float bpmforFrames=bpm;
float delayTime=(60/bpmforFrames)*1000/(2*numSteps);
//float actualbpm=60/(frameTime/1000);
//Serial.printf("Actual framerate is %f and the bpm is %f \n",frameTime,bpmforFrames);
 float newXPos=xCurrent;
 while(animationStart){
  //Serial.println(xCurrent-(center-stepSize*numSteps));
   if(switcher==0){
      newXPos=newXPos+stepSize;
      if(newXPos-(center+stepSize*numSteps)==0){
        switcher=1;
      }
   }
   else if(switcher==1){
      newXPos=newXPos-stepSize;
      
      if(newXPos-(center-stepSize*numSteps)==0){
        switcher=0;
      }
   }
  delay(delayTime-frameTime);
  moveObj(newXPos,yCurrent,zCurrent);
  
  if ( Serial.available() ) {
    String command = Serial.readStringUntil('\n');
    if (command=="stop"){
      Serial.println("metronome halted");
      break;
    }
  }  
 }

}
void bufferLatency(){
  Serial.println("start");
  int times[50];
  int k = 1;
  while(k<50){
      int start=micros();
      String frame="focus=" + String(xCurrent,7) + " " + String(yCurrent,7) + " " + String(zCurrent,7) + " ";
      processCommand(frame);
      int finish=micros();
      times[k]=finish-start;
      Serial.print(times[k]);
      Serial.print(", ");
      k++;
      
  }
  Serial.println("DONE");


}
void setXZ(float minimumX, float minimumZ){
    //moveObj(minimumX,yCurrent,zCurrent);
    //moveObj(xCurrent,yCurrent,minimumZ);
    moveObj(0,yCurrent,0);
}
void testAxis(){
      
   //frame and increment info
  
  
  float center=0;
  float maximumX=0.065;
  float minimumX=-0.065;
  float maximumZ=0.065;
  float minimumZ=-0.065;
  float maximumY=ARRAY_HEIGHT/2+0.06;
  float minimumY=ARRAY_HEIGHT/2-0.06;
  


 //while loop for testing 
  int testXMax=0;
  int testXMin=0;
  int testZMax=0;
  int testZMin=-0;
  int testing=1;
  int switchZ = 0;
  int switchY=0;

  
  setXZ(minimumX,minimumZ);
  float newXPos=xCurrent;
  float newYPos=yCurrent;
  float newZPos=zCurrent;
  int spiral=1;
  while(testing){
      if(Serial.available()){
        break;
      } 
    newXPos=xCurrent;
    newYPos=yCurrent;
    newZPos=zCurrent;
    //Serial.printf("Floats %f, %f, %f\n",newXPos,newYPos,newZPos);
  
      newXPos=stepSize*spiral;
      if(newXPos<maximumX){
          moveObj(newXPos,yCurrent,zCurrent);

      }
      else{
          newXPos=maximumX;
          moveObj(newXPos,yCurrent,zCurrent);
          testXMax=1;
          
      }
      Serial.printf("TESTING X: %f, Y: %f Z: %f\n", xCurrent, yCurrent,zCurrent);
         
      newZPos=stepSize*spiral;
      if(newZPos<maximumZ){
        
        moveObj(xCurrent,yCurrent,newZPos);
        
        
      }
      else{
        newZPos=maximumZ; 
        moveObj(xCurrent,yCurrent,newZPos);
        testZMax=1;
      }
      Serial.printf("TESTING X: %f, Y: %f Z: %f\n", xCurrent, yCurrent,zCurrent);
      newXPos=-stepSize*spiral;
      if(newXPos>minimumX){
         
         moveObj(newXPos,yCurrent,zCurrent);
       
         
      }
      else{
        newXPos=minimumX;
        moveObj(newXPos,yCurrent,zCurrent);
        testXMin=1;
      }
      Serial.printf("TESTING X: %f, Y: %f Z: %f\n", xCurrent, yCurrent,zCurrent);
      newZPos=-stepSize*spiral;
      if(newZPos>minimumZ){
        
        moveObj(xCurrent,yCurrent,newZPos);
        
        
      }
      else{
        newZPos=minimumZ;
        moveObj(xCurrent,yCurrent,newZPos);
        testZMin=1;
      }
      Serial.printf("TESTING X: %f, Y: %f Z: %f\n", xCurrent, yCurrent,zCurrent);

      spiral=spiral+5;

      //adjust y to change planes
      if (testXMax==1&&testZMax==1&&testXMin==1&&testZMin==1){
         if(switchY==0){
            newYPos=newYPos+stepSize*5;
        if(newYPos>=maximumY){
          switchY=1;
        }
         }
       else if (switchY==1){
          newYPos=newYPos-stepSize*5;
        if(newYPos<=minimumY){
          switchY=0;
        }
       }
        setXZ(minimumX, minimumZ);
        moveObj(0,newYPos,0);
        testXMax=0;
        testXMin=0;
        testZMax=0;
        testZMin=0;
        xCurrent=0;
        zCurrent=0;
        spiral=1;
      }
      

     }
     

      
      
      
 
      Serial.printf("I'm at: %f %f %f\n",xCurrent, yCurrent, zCurrent);
      
  }
  
    

void moveObj(float newXPos, float newYPos, float newZPos){
  Serial.printf("My old positions x: %f, y: %f, z: %f \n",xCurrent, yCurrent,zCurrent);
  Serial.printf("My new positions x: %f, y: %f, z: %f \n",newXPos,newYPos,newZPos);
  
  int tStart = micros();
while(abs(xCurrent-newXPos)>stepSize || abs(yCurrent-newYPos)>stepSize || abs(zCurrent-newZPos)>stepSize){
      
      //xpos movement
      if(xCurrent>newXPos && abs(xCurrent-newXPos)>stepSize){
        xCurrent=xCurrent-stepSize;
      }
      else if (xCurrent<newXPos && abs(xCurrent-newXPos)>stepSize){
        xCurrent=xCurrent+stepSize;
      }
      //ypos movement
      if(yCurrent>newYPos && abs(yCurrent-newYPos)>stepSize){
        yCurrent=yCurrent-stepSize/4;
      }
      else if(yCurrent<newYPos && abs(yCurrent-newYPos)>stepSize){
        yCurrent=yCurrent+stepSize/4;
      }
      //zpos movement
      if(zCurrent>newZPos && abs(zCurrent-newZPos)>0.001){
        zCurrent=zCurrent-stepSize;
      }
      else if (zCurrent<newZPos && abs(zCurrent-newZPos)>0.001){
        zCurrent=zCurrent+stepSize;
      }
  
      int frameComplete=0;
      unsigned long elapsed=millis();
       while(millis()-elapsed<frameTime){
        if(frameComplete==0){
          String frame="focus=" + String(xCurrent,7) + " " + String(yCurrent,7) + " " + String(zCurrent,7) + " ";
          processCommand(frame);
          //Serial.println(frame);
          frameComplete=1;            
        }
        
      }
      frameComplete=0;
      if(Serial1.available()){
        break;
      }  
     

      
}
      int tFinal=micros();
      Serial.printf("total time taken is %d \n", tFinal-tStart);
 }
void processCommand(String command) {
  if ( command.startsWith("focus=") ) {
    const int n = parseFloats( command, command.indexOf('=') + 1 , pointPos, 3 );
    
    //Serial.printf("Read %d %f %f %f\n", n, pointPos[0], pointPos[1], pointPos[2] );
    //convert to metres from mm
//      pointPos[0]=pointPos[0]/1000;
//      pointPos[1]=pointPos[1]/1000;
//      pointPos[2]=pointPos[2]/1000;
     
    if (n == 3) {
      simpleFocusAt(pointPos, buff );
      sendBuffer();
    }
  }
  //bpm commands
  else if(command.startsWith("bpm=")){
    String test = command.substring( command.indexOf('=') + 1);
    const int bpm = test.toInt();
    animateBPM(bpm);
  }
  else if(command.startsWith("test")){
    Serial.println("testing.....");
    testAxis();
  }
  else if(command.startsWith("reset")){
    float newXPos=0;
    float newYPos=ARRAY_HEIGHT/2;
    float newZPos=0;
    moveObj(newXPos,newYPos,newZPos);
    Serial.println("focal point reset to center.....");
  }
  else if (command.startsWith("current")){
    Serial.printf("Current positions are X: %f, Y: %f, Z: %f\n",xCurrent,yCurrent,zCurrent);
  }
  else if (command.startsWith("mode=")){
    String checks = command.substring(command.indexOf('=')+1);
    modes = checks.toInt();
    if (modes==0){
      Serial.println("Currently animating 3 dial system");
    }
    else if (modes==1){
      Serial.println("Currently animating keyboard system");
    }
    else if (modes==2){
      Serial.println("Currently animating ZY dials and keys in X");
    }
    else{
      Serial.println("No valid mode entered");
    }
  }
  else if ( command.startsWith("itersIBP=") ) {
    ibpIterations = command.substring( command.indexOf('=') + 1).toFloat();
    Serial.printf("IBP iterations set at %d\n", ibpIterations);
  }else if ( command.startsWith("focusIBP=") ) {
    const int n = parseFloats( command, command.indexOf('=') + 1 , pointPos, 3 * MAX_POINTS);
    if (n >= 3 && n % 3 == 0) {
      const int nPoints = n / 3;
      //const long timeBefore = millis();
      ibp_initPropagators(nPoints, pointPos);
      // with 8 iteration it takes 46ms, with 4 it takes 39.
      for (int i = 0; i < ibpIterations; ++i)
        ibp_iterate( nPoints );
      ibp_applySolution(nPoints, buff);
      //const int duration = millis() - timeBefore;
      //Serial.printf("Millis of IBP %d", duration);
      sendBuffer();
    }
  } else if ( command.startsWith("focusMulti=") ) {
    const int n = parseFloats( command, command.indexOf('=') + 1 , pointPos, 3 * MAX_POINTS);
    if (n >= 3 && n % 3 == 0) {
      const int nPoints = n / 3;
      //const long timeBefore = millis();
      //it takes less than 1ms
      multiFocusAt(nPoints,  pointPos,  buff);
      //const int duration = millis() - timeBefore;
      //Serial.printf("Millis of Multi %d", duration);
      sendBuffer();
    }
  } else if ( command.startsWith("phases=") ) {
    sendPhases( command );
  } else if ( command.startsWith("off") ) {
    Serial.println("Switching off the array...");
    switchOnArray(false, false);
  } else if ( command.startsWith("on") ) {
    Serial.println("Switching on the array...");
    switchOnArray(true, true);
  } else if ( command.startsWith("top") ) {
    Serial.println("Switching on top...");
    switchOnArray(true, false);
  } else if ( command.startsWith("bottom") ) {
    Serial.println("Switching on bottom...");
    switchOnArray(false, true);
  } else if ( command.startsWith("switch") ) {
    Serial.println("Switching emissions...");
    switchBuffer();
  } else if ( command.startsWith("version") ) {
    Serial.printf("Version is %s\n", ESPFOCAL_VERSION);
  }

}


void sendBuffer() {
  
  Serial2.write( 254 ); //start sending phases
  Serial2.write( 192 + 0 ); //for board id 0
  Serial2.write( buff, N_EMITTERS / 2);
  Serial2.write( 192 + 1 ); //for board id 1
  Serial2.write( & buff[N_EMITTERS / 2] , N_EMITTERS / 2);
  Serial2.write( 253 ); //commit
  Serial2.flush();
 
}

void switchOnArray(boolean onTop, boolean onBottom) {
  for (int j = 0; j < N_EMITTERS / 2; j += 1)
    buff[j] = onTop ? 1 : 32; //32 is the value for switching off the transducer

  for (int j = N_EMITTERS / 2; j < N_EMITTERS; j += 1)
    buff[j] = onBottom ? 1 : 32; //32 is the value for switching off the transducer

  sendBuffer();
}

void switchBuffer() {
  Serial2.write( 253 );
}

void sendPhases(String command) {
  int index = 0;
  int number = 0;
  const int n = command.length();
  for (int i = command.indexOf('=') + 1; i < n; ++i) {
    char c = command.charAt(i);
    if ( c >= '0' && c <= '9') {
      number = number * 10 + (c - '0');
    } else {
      buff[index] = number;
      number = 0;
      index += 1;
      if (index == N_EMITTERS)
        break;
    }
  }
  sendBuffer();
}


int parseFloats(String s, int startIndex, float target[], int maxNumbers) {
  int index = 0;
  const int n = s.length();
  float number = 0;
  boolean haveSeenDecPoint = false;
  boolean haveSeenMinus = false;
  float divider = 10;
  for (int i = startIndex; i < n; ++i) {
    char c = s.charAt(i);
    if ( c >= '0' && c <= '9') {
      if (! haveSeenDecPoint) {
        number = number * 10 + (c - '0');
      } else {
        number += divider * (c - '0');
        divider /= 10;
      }
    } else if ( c == '.') {
      haveSeenDecPoint = true;
      divider = 0.1;
    } else if (c == '-') {
      haveSeenMinus = true;
    } else {
      if (haveSeenMinus) {
        number = -number;
      }
      target[index] = number;
      number = 0;
      haveSeenDecPoint = false;
      haveSeenMinus = false;
      divider = 0.1;
      index += 1;
      if (index == maxNumbers)
        break;
    }
  }
  return index;
}
