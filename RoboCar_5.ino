/*************/
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>//get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>      //comes with Arduino IDE (www.arduino.cc)
#include <SPIFlash.h> //get it here: https://www.github.com/lowpowerlab/spiflash


#include <Adafruit_NeoPixel.h>

//#include <Servo.h> 

//#include <SoftwareServo.h>
//SoftwareServo rotateServo,armServo,clawServo;  // create servo object to control a servo 
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 17
Adafruit_SSD1306 display(OLED_RESET);


#define SSD1306_LCDHEIGHT 64





//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************

#define SERIAL_BAUD   115200

#ifdef __AVR_ATmega1284P__
  #define LED           15 // Moteino MEGAs have LEDs on D15
  #define FLASH_SS      23 // and FLASH SS on D23
#else
  #define LED           15 // Moteinos have LEDs on D9
  #define FLASH_SS      8 // and FLASH SS on D8
#endif

#ifdef ENABLE_ATC
  RFM69_ATC radio(15);
#else
  RFM69 radio(15);
#endif

/************/



#define PIN 8 //PB0

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, PIN, NEO_GRB + NEO_KHZ800);




//int motor1 = 10; //PB2
//int motor2 = 5; //PD5

byte motor3 = 6; //PD6
byte motor4 = 9; //PB1

int posRotation=32;
int posArm=20;
int posClaw=32;

int servoSensitivity=1;


//int motor1Dir = 16; //PC2
//int motor2Dir = 3; //PD3
byte motor3Dir = 4; //PD4
byte motor4Dir = 7; //PD7

byte debugMode1=0;
byte canGoForward=1;



//int stopSensor = 2; //PD2 int0
//int stopSensor = 3; //PD3 int1
int STOP_SENSOR = 16; //PC2


int const delayTime = 1000; //ms
int motorSpeed = 150;


int const DIR_STOP = 0;
int const DIR_FORWARD = 1;
int const DIR_BACK = 2;
int const DIR_RIGHT_FORWARD = 3;
int const DIR_LEFT_FORWARD = 4;
int const DIR_RIGHT_BACK = 5;
int const DIR_LEFT_BACK = 6;
int currentDir = DIR_STOP; //

int lastSensorState = HIGH;







  //ADC variables
word x1,y1,x2,y2,x3,y3;
word positionFrame[16];

int val;

long vccVoltage() {
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	//_delay_us(250);
	//delay(8);

	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA, ADSC));
	result = ADCL;
	result |= (ADCH << 8);
	//result = 1126400L / result; // Back-calculate AVcc in mV
	result = (1100L * 1023) / result; // Back-calculate AVcc in mV
	return result;
}

void displayBat() {
	
	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);

	display.print("B1:");
	display.print(vccVoltage() / 1000.0f, 2);
	display.print(" V");
	display.display();

	
	
}

volatile unsigned int displayBatFlag = 0;



void setup()
{
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

 
  displayBat();
    
  
  
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  
  currentDir = DIR_STOP;
  
  pinMode(STOP_SENSOR,INPUT);
  
  //pinMode(motor1, OUTPUT);   // sets the pin as output
  //pinMode(motor2, OUTPUT);   // sets the pin as output
  pinMode(motor3, OUTPUT);   // sets the pin as output
  pinMode(motor4, OUTPUT);   // sets the pin as output
  
  pinMode(3, OUTPUT);
  
   pinMode(5, OUTPUT);
   pinMode(10, OUTPUT);
  
  //pinMode(motor1Dir, OUTPUT);   // sets the pin as output
  //pinMode(motor2Dir, OUTPUT);   // sets the pin as output
  pinMode(motor3Dir, OUTPUT);   // sets the pin as output
  pinMode(motor4Dir, OUTPUT);   // sets the pin as output
  
  //pinMode(stopSensor, INPUT_PULLUP);   
  //attachInterrupt(1, stopCar2, FALLING);
  lastSensorState = digitalRead(STOP_SENSOR);
  
  
  //analogWrite(motor1,0);
  //analogWrite(motor2,0);
  analogWrite(motor3,0);
  analogWrite(motor4,0);
  
  //rainbow(20);
  strip.setPixelColor(0,64,0,0);
  strip.setPixelColor(1,0,64,0);
  strip.setPixelColor(2,0,0,64);
  strip.setPixelColor(3,0,0,64);
  strip.setPixelColor(4,0,64,0);
  strip.setPixelColor(5,64,0,0);
  strip.show();
  
  Serial.begin(115200);
  Serial.println("Czesc to ja!");
  //irrecv.enableIRIn(); // Start the Infrared receiver
  
  /** radio **********/
  Serial.begin(SERIAL_BAUD);
  delay(10);
  radio.initialize(FREQUENCY,NODEID,NETWORKID);
  //radio.readAllRegs();
#ifdef IS_RFM69HW
  radio.setHighPower(); //only for RFM69HW!
#endif
  radio.encrypt(ENCRYPTKEY);
  
  
  
  
  /******** servo setup ************/
  
  
  
  
  setPwmFrequency(3, 256);
  
  
  setPwmFrequency(5, 256);
  
  setPwmFrequency(10, 256);
  
  analogWrite(3,33);
  analogWrite(5,16);
  analogWrite(10,33);
 
  
  
  
  /******************************/
  
}

void signalDirection(int dir){
  strip.clear();
  strip.show();
  
  switch (dir){
     case DIR_FORWARD:
      strip.setPixelColor(2,0,64,0);
      strip.setPixelColor(3,0,64,0);
      break;
    case DIR_BACK:
      strip.setPixelColor(2,64,0,0);
      strip.setPixelColor(3,64,0,0);
      break;
    case DIR_RIGHT_FORWARD:
      strip.setPixelColor(5,64,35,0);

      break;
    case DIR_LEFT_FORWARD:
      strip.setPixelColor(0,64,35,0);
      break;
      
    case DIR_RIGHT_BACK:
      strip.setPixelColor(5,64,35,50);

      break;
    case DIR_LEFT_BACK:
      strip.setPixelColor(0,64,35,50);
      break;
     
    case DIR_STOP:
      break;
    default:
      break; 
  }
  
  strip.show();
  
}





void goForward(){
  //Serial.println("Going forward");
  
  if(currentDir != DIR_FORWARD)
    signalDirection(DIR_FORWARD);
  
  currentDir = DIR_FORWARD;
  
  //TO_DO: set direction
  //digitalWrite(motor1Dir,0);
  //digitalWrite(motor2Dir,0);
  digitalWrite(motor3Dir,0);
  digitalWrite(motor4Dir,0);
  
  //TO_DO: set speed
  //analogWrite(motor1,motorSpeed);
  //analogWrite(motor2,motorSpeed);
  analogWrite(motor3,motorSpeed);
  analogWrite(motor4,motorSpeed);
  
   
  
}

void goBack(){
  //Serial.println("Going back");
  
  if(currentDir != DIR_BACK)
    signalDirection(DIR_BACK);
  
    currentDir = DIR_BACK;

    //TO_DO: set direction
    //digitalWrite(motor1Dir,1);
  //digitalWrite(motor2Dir,1);
  digitalWrite(motor3Dir,1);
  digitalWrite(motor4Dir,1);
  
    //TO_DO: set speed
    //analogWrite(motor1,motorSpeed);
    //analogWrite(motor2,motorSpeed);
    analogWrite(motor3,motorSpeed);
    analogWrite(motor4,motorSpeed);


  
}

void turnRightForward(){
  //Serial.println("Going Right/Forward");
  
  if(currentDir != DIR_RIGHT_FORWARD)
    signalDirection(DIR_RIGHT_FORWARD);
    
    currentDir = DIR_RIGHT_FORWARD;

  
  digitalWrite(motor3Dir,0);
  digitalWrite(motor4Dir,0);
    
  analogWrite(motor3,motorSpeed);
  analogWrite(motor4,motorSpeed/4);
  
}

void turnRightBack(){
  //Serial.println("Going Right/Forward");
  
  if(currentDir != DIR_RIGHT_BACK)
    signalDirection(DIR_RIGHT_BACK);
    
    currentDir = DIR_RIGHT_BACK;

  
  digitalWrite(motor3Dir,1);
  digitalWrite(motor4Dir,1);
    
  analogWrite(motor3,motorSpeed);
  analogWrite(motor4,motorSpeed/4);
  
}

void turnLeftForward(){
  
  if(currentDir != DIR_LEFT_FORWARD)
    signalDirection(DIR_LEFT_FORWARD);
  
  currentDir = DIR_LEFT_FORWARD;

 
  digitalWrite(motor3Dir,0);
  digitalWrite(motor4Dir,0);
  
  
  analogWrite(motor3,motorSpeed/4);
  analogWrite(motor4,motorSpeed);
  
}  

void turnLeftBack(){
  
  if(currentDir != DIR_LEFT_BACK)
    signalDirection(DIR_LEFT_BACK);
  
  currentDir = DIR_LEFT_BACK;

 
  digitalWrite(motor3Dir,1);
  digitalWrite(motor4Dir,1);
  
  
  analogWrite(motor3,motorSpeed/4);
  analogWrite(motor4,motorSpeed);
  
}  
  
void updateSpeed(){
  
  switch (currentDir){
    case DIR_FORWARD:
      goForward();
      break;
    case DIR_BACK:
      goBack();
      break;
    case DIR_RIGHT_FORWARD:
      turnRightForward();
      break;
    case DIR_LEFT_FORWARD:
      turnLeftForward();
      break;
    default:
      break;
  
  
  }
}
     
void updateRadioSpeedDir(word x1,word y1){
  byte tmpspeed;
  
  //move forward
  if((canGoForward ==1) && (x1 < 526) && (x1>518)  && (y1>527)){
    motorSpeed = 128+(y1-527)/4;
    goForward();
  } 
 
 //move back
  if((x1 < 526) && (x1>518)  && (y1<516)){
    motorSpeed = 128+(516-y1)/4;
    goBack();
  } 
  
  //stop
  if((y1<527) && (y1>516)){
    stopCar();
  }
  
  //right-forward
  if((canGoForward ==1 ) && (x1<518)  && (y1>527)){
    motorSpeed = 128+(y1-527)/4;
    turnRightForward();
  } 
  
  //left-forward
  if((canGoForward ==1) && (x1>526)  && (y1>527)){
    motorSpeed = 128+(y1-527)/4;
    turnLeftForward();
  } 
  
  //right-back
  if((x1<518)  && y1<518){
    motorSpeed = 128+(516-y1)/4;
    turnRightBack();
  } 
  
  //left-back
  if((x1>526)  && y1<518){
    motorSpeed = 128+(516-y1)/4;
    turnLeftBack();
  } 
    
  
}
  


void stopCar(){
  //analogWrite(motor1,0);
  //analogWrite(motor2,0);
  analogWrite(motor3,0);
  analogWrite(motor4,0);
  
  currentDir = DIR_STOP;
  signalDirection(currentDir);
}

void speedUp(){
   if(motorSpeed<=251){
     motorSpeed = motorSpeed+4;
     updateSpeed();
   }
     
}

void slowDown(){
   if(motorSpeed>=4){
     motorSpeed = motorSpeed-4;
     updateSpeed();
   }
     
}

int parseFrame(word *framePtr){
  //check frame header
  if(framePtr[0] != 0xA0A1)
    return -1;
    
  x1 = framePtr[1];
  y1 = framePtr[2];
  x2 = framePtr[3];
  y2 = framePtr[4];
  x3 = framePtr[5];
  y3 = framePtr[6];
  
  return 0;
}




// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


byte ackCount=0;
uint32_t packetCount = 0;
int v1;
int currStopSensor;

void loop() {
  currStopSensor = digitalRead(STOP_SENSOR);
  
  
  if((canGoForward == 1) &&(lastSensorState == 1) && (currStopSensor == 0)){ // sensor detected stop 1-->0 
      lastSensorState = 0;
      stopCar();
      canGoForward = 0;
  }
  
  if((canGoForward == 0) &&(lastSensorState == 0) && (currStopSensor == 1)){ // sensor detected removal of the Obstacle 0-->1
      lastSensorState = 1;
     
      canGoForward = 1;
  }
  
  
  
  
  /********** radio *************/
  if (radio.receiveDone())
  {
    if(debugMode1==1){
      Serial.print("#[");
      Serial.print(++packetCount);
      Serial.print(']');
      Serial.print('[');Serial.print(radio.SENDERID, DEC);Serial.print("] ");
      
      
      
      Serial.print("   [RX_RSSI:");Serial.print(radio.RSSI);
      Serial.println("]");
    }


    if(!parseFrame((word *)radio.DATA)){
      
      if(debugMode1==1){
          Serial.print("X1: ");
          Serial.print(x1,DEC);
          
           Serial.print(" Y1: ");
          Serial.print(y1,DEC);
          
          Serial.print(" X2: ");
          Serial.print(x2,DEC);
          
          Serial.print(" Y2: ");
          Serial.println(y2,DEC);
          
          Serial.print(" X3: ");
          Serial.print(x3,DEC);
          
          Serial.print(" Y3: ");
          Serial.println(y3,DEC);
      }
          
          x2=x2+11;
          
          
          x3=x3+11;
          
    
          val = map(x2, 0, 1023, -servoSensitivity, servoSensitivity);
          
          posRotation = posRotation + val;
          if(posRotation>50){
            posRotation=50;
          }
          
          
         
          if(posRotation<14){
            posRotation=14;
          }
       
         
          
          analogWrite(3,posRotation);

  
            
          
          val=0;
          if(y2<500){
            val=-1;
          } 
          
          if(y2>520) {
            val=1;
          }
          
          posArm = posArm + val;
          
           if(posArm>104){ 
            posArm=104;
          }

          if(posArm<16){ //4*5 = 20
            posArm=16;
          }

         
          
         analogWrite(5,posArm);
         
          
          val = map(x3, 0, 1023, -servoSensitivity, servoSensitivity);
          
          posClaw = posClaw + val;
          
          if(posClaw>49){
            posClaw=49;
          }
          
         
          if(posClaw<13){
            posClaw=13;
          }
          
           
           
           //clawServo.write(posClaw);
          analogWrite(10,posClaw);
          
          
          
            updateRadioSpeedDir(x1,y1);
          
    } else {
		if (debugMode1 == 1) {
			Serial.println("Frame header incorrect");
		}
    }
    
	
	
  }
  /*******************************/
  
  displayBatFlag++;

  if (displayBatFlag == 65535) {
	  displayBat();
	  displayBatFlag = 0;
  }
  
  
  //SoftwareServo::refresh();
}

void blink1(byte pin,int delay1)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin,HIGH);
  delay(delay1);
  digitalWrite(pin,LOW);
}

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


