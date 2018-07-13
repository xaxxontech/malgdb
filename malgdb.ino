/*
ASCII Serial Commands

FORWARD = 'f', [0-255][0-255] (speed for each wheel) 
BACKWARD = 'b', [0-255][0-255] (speed for each wheel)
LEFT = 'l', [0-255][0-255] (speed for each wheel)
RIGHT = 'r', [0-255][0-255] (speed for each wheel)
FORWARDTIMED = 't' [0-255][0-255] (speed for each wheel) [0-255][0-255] DWORD milliseconds
BACKWARDTIMED = 'u' [0-255][0-255] (speed for each wheel) [0-255][0-255] DWORD milliseconds
LEFTTIMED = 'z'. [0-255][0-255] (speed for each wheel) [0-255][0-255] DWORD milliseconds
RIGHTTIMED = 'e'. [0-255] (speed) [0-255][0-255] DWORD milliseconds
CAM = 'v', [0-255] (servo angle)  
CAM RELEASE = 'w' (servo release)
CAM HORIZ SET = 'm' [0-255] (write horizontal position to eeprom)
STOP = 's' (DC motors stop)
HARD_STOP = 'h' (DC motors stop with brake)
GET VERSION = 'y'
GET FIRMWARE = 'x'
FWD FLOOD LIGHT = 'q', [0-255] (intensity)
REAR FLOOD LIGHT = 'o', [0-255] (intensity)
SPOT LIGHT = 'p', [0-255] (intensity)
ODOMETRY_START = 'i' (start recording encoder and gyro, zero values)
ODOMETRY_STOP = 'j' (stop recording encoder and gyro, and report)
ODOMETRY_REPORT = 'k' (report current encoder and gyro counts, then zero counts)
GYRO_TEST = 'g'
PING = 'c' (heartbeat)
EEPROM_CLEAR = '8' (set entire eeprom bank to 0)
EEPROM_READ = '9' (read camhoriz position) 
SEND_SLAVEI2C_BYTE = 'a' [0-255] (send specified byte to I2C slave, optional peripheral)
*/

/*
 * gyro function
 * 
 * config:
 * read z only, 380Mz rate, 250 degrees-per-second resolution
 * write to 32 value FIFO buffer, stream overwrite continuously 
 * 
 * when motors stopped, calibrate zero every 2 seconds
 * 
 * angle reading is cummulative, as gyro produces rate data (degrees per second)
 * if active (not stopped, and readangle boolean true), 
 * check if fifo threshold reached every loop, if so read up to threshold and add to angle
 * 
 * if info requested or stop detected, print angle to serial, start from 0 again
 * 
 * 
 */
 
#include <Wire.h> 
#include <Servo.h> 
#include <EEPROM.h>

#define GYROaddr 0x6b
#define SLAVEI2C 0x08
#define WHIGH 0 // 0 = default, reverse 0/1 if wheels wired backwards
#define WLOW 1  // 1 = default, reverse 0/1 if wheels wired backwards

// hbridge 
const int pwmA = 3; // pwm
const int in1 = 2;
const int in2 = 4;
const int pwmB = 11; // pwm
const int in3 = 7;
const int in4 = 8;

// lights 
const int spotPin = 6; // pwm
const int floodPWMPin = 5; // pwm
const int fwdFloodPin = 12;
const int rearFloodPin = 9;

// servo
const int servoPin = 10; // pwm
Servo camservo;  
const int eepromAddress = 0;

// encoder
const int encA = A0;

// spare 13, A2, A3, A1


// timers 
unsigned long time = 0;
unsigned long lastcmd = 0;
const unsigned long hostTimeout = 10000; // stop motors if no steady ping from host
unsigned long stoptime = 0;
int loopinc = 0; // TODO: testing
unsigned long lastloopinc = 0; // TODO: testing

// command byte buffer 
const int MAX_BUFFER = 32;
int buffer[MAX_BUFFER];
int commandSize = 0;

//pins 5,6 timer 0 factor: 62500/1 >> 64
// 62500/8 >> 8
// 62500/256 >> 0.25
const double timemult = 64;

// gyro
boolean readAngle = false;
double angle = 0;
// const double calibrationComp = 1.047; // 1.047; // 1.094;
const unsigned long GYROZEROINTERVAL = 2000;
unsigned long nextGyroZero = 0;
int zOff = 0;
// const double gyroSampleRate = 0.00263157; // 380hz, milliseconds
// const double gyroSampleRate =    0.00526315; // 190hz, milliseconds
const double gyroSampleRate =    0.0013158; // 760hz, milliseconds
int gyroZ[32]; // max FIFO buffer size
int gyrosamples = 0;
// double driftDegPerSec = 0;
boolean gyroFifoReadAfterStop = false;
int gyrozerosamples = 0;
unsigned long gyrozeroavg = 0;
// end of gyro

// encoder
volatile boolean encoderPinAtZero = false;
volatile int encoderTicks = 0;
const int gearRatio = 180; 
volatile boolean readEncoder = false;
volatile unsigned long lastEncoderTick = 0;
volatile boolean stopdetected = false;
// end of encoder

// stop detect control
int directioncmd = 0; // 0=stop, 1=forward, 2=backward, 3=left, 4=right
boolean stopped = true;
boolean stopPending = false;
unsigned long stopCommand = 0;
const unsigned long allowforstop = 1000;


void setup() { 

	// horiz servo
	int m = (int) EEPROM.read(eepromAddress);
	if (m < 30 || m > 100) { // not within range where it looks like its been set properly before 		
		m = 70;  //default
		EEPROM.write(eepromAddress, m);
	} 
	camservo.attach(servoPin);  
	camservo.write(m);
	delay(500);
	camservo.detach();
  
	pinMode(pwmA, OUTPUT);
	pinMode(pwmB, OUTPUT);
	pinMode(in1, OUTPUT); 
	pinMode(in2, OUTPUT); 
	pinMode(in3, OUTPUT); 
	pinMode(in4, OUTPUT); 

	pinMode(rearFloodPin, OUTPUT);  
	pinMode(spotPin, OUTPUT);  
	pinMode(fwdFloodPin, OUTPUT); 
	
	pinMode(encA, INPUT); 

	//pwm frequencies setup (from http://playground.arduino.cc/Code/PwmFrequency)
	TCCR2B = TCCR2B & 0b11111000 | 0x07;  // 30 Hz pin 3, 11 wheels
	TCCR0B = TCCR0B & 0b11111000 | 0x01; // pins 5, 6 62500/1 = 62kHz lights
	
	// motors fwd, off
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
	
	Wire.begin();
	Wire.setClock(400000L);
	
	// gyro setup
	gyroWrite(0x24, 64); // CTRL_REG5 01000000 FIFO enable
	gyroWrite(0x2E, 64); // FIFO_CTRL_REG 01000000 FIFO stream mode
	// gyroWrite(0x20, 172); // CTRL_REG1 10 10 1 1 0 0 380Hz ODR, 50Hz LPF2, pwr normal, z, x, y
	// gyroWrite(0x20, 175); // CTRL_REG1 10 10 1 1 1 1 380Hz ODR, 50Hz LPF2, pwr normal, z, x, y
	// gyroWrite(0x20, 108); // CTRL_REG1 01 10 1 1 0 0 190Hz ODR, 50Hz LPF2, pwr normal, z, x, y 
	gyroWrite(0x20, 239); // CTRL_REG1 11 10 1 1 1 1 760Hz ODR, 50Hz LPF2, pwr normal, z, x, y 
	// gyroWrite(0x20, 204); // CTRL_REG1 11 00 1 1 0 0 760Hz ODR, 30Hz LPF2, pwr normal, z, x, y 

	
	// encoder read interrupt setup
	cli();		// switch interrupts off while messing with their settings  
	PCICR =0x02;          // Enable PCINT1 interrupt
	PCMSK1 = 0b00000001; // mask A0
	sei();   // switch back on
	
	Serial.begin(115200);
	Serial.println("<reset>"); 

}


void loop(){

	time = millis() / timemult;

	if (readAngle && !stopped) { // && time - gyroReadFifoInterval > lastGyroFifoRead) { 
		if (getGyroFIFOcontents()) {
			boolean checkstop = false;
			if (stopPending && (directioncmd ==3 || directioncmd==4) && time- lastcmd > 0) checkstop = true; //was time-lastcmd>80
			
			int zavg = 0;

			for (int i=0; i<gyrosamples; i++ ) {
				int z = gyroZ[i]+zOff;
				double degPerSec = (double) z * 250/0x7fff * -1; // using 250dps default scale, negate because typically mounted upside down
				angle += degPerSec * gyroSampleRate;
				
				zavg += z;
			}
			
			zavg /= gyrosamples;
			if (checkstop && abs(zavg) < 150) stopDetect();
			
			// Serial.println(gyrosamples); // almost always 1!
		}
		
	}
	
	// periodically determine gyro zero offset when stopped
	if (false) { // (time > nextGyroZero && time >= GYROZEROINTERVAL) {
		if (stopped) {
			if (getGyroFIFOcontents()) {
				int zavg = 0;
				for (int i=0; i<gyrosamples; i++) {
					zavg += gyroZ[i];
				}
				zavg /= gyrosamples;  
				zOff = 0 - zavg;
				
				// driftDegPerSec = (double) zavg * 250/0x7fff * -1; // testing
				
				// TODO: testing only, nuke!
				// Serial.print("<zOff: ");
				// Serial.print(zOff);
				// Serial.println(">");
				
				// testing offset closest to zero
				// int zmin = 32767;
				// for (int i=0; i<gyrosamples; i++) {
					// if (abs(gyroZ[i]) < abs(zmin)) zmin = gyroZ[i];
				// }
				// zOff = -zmin;
			}
			else Serial.println("<getGyroFIFOcontents 0 samples");
		}
		nextGyroZero = time + GYROZEROINTERVAL;
	}
	
	if (time > nextGyroZero && time > 2000) {
		if (stopped) {
			gyroRead();
			gyrozeroavg += gyroZ[0];
			gyrozerosamples ++;
			
			if (gyrozerosamples > 1000) {
				
				zOff = -gyrozeroavg/gyrozerosamples;
				zOff *= 0.75;
				
				// TODO: testing only, nuke!
				// Serial.print("<zOff: ");
				// Serial.print(zOff);
				// Serial.println(">");
				
				gyrozerosamples = 0;
				gyrozeroavg = 0;
			}
			nextGyroZero = time + 3;
		}
		else {
			gyrozerosamples = 0;
			gyrozeroavg = 0;
			nextGyroZero = time + 1000;
		}
		
	}
	
	// allow for slow down to complete stop, timed
	if (stopPending && time - allowforstop > stopCommand && readAngle && readEncoder) {
		Serial.println("<stopdetectfail>"); // TODO: testing only 
		stopDetect();
	}
	
	if (stopdetected) { // used by interrupt only 
		stopdetected = false;
		if (readEncoder && stopPending && (directioncmd==1 || directioncmd==2) ) {
			stopDetect();
		}
	}
	
	// firmware timed moves
	if (stoptime != 0 && time > stoptime) {
		stop();
		stoptime = 0;
	}
	
	// manage serial input
	if( Serial.available() > 0) {
		lastcmd = time;
		manageCommand();
	}
	else if (time - hostTimeout > lastcmd ){ 
		 //if no comm with host, stop motors
		 allOff();
		 lastcmd = time; 
	}

	// loopinc ++;
	// if (loopinc > 1000) {
		// Serial.println(
	// }
}

// 
// buffer and/or execute commands from host controller 
//
void manageCommand() {
	int input = Serial.read();
	if((input == 13) || (input == 10)){
		if(commandSize > 0){
			  parseCommand();
			  commandSize = 0; 
		}
	} 
	else {
		buffer[commandSize++] = input;
	}
}

void allOff() {
	analogWrite(floodPWMPin, 0);
	analogWrite(spotPin, 0);
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	camservo.detach();
}

void parseCommand(){
  
	if (buffer[0] == 'q'){ // forward floodlight
		analogWrite(floodPWMPin, buffer[1]);
		digitalWrite(fwdFloodPin, LOW);
		digitalWrite(rearFloodPin, HIGH);
	}
  
	else if (buffer[0] == 'o'){ // floodlight
		analogWrite(floodPWMPin, buffer[1]);
		digitalWrite(fwdFloodPin, HIGH);
		digitalWrite(rearFloodPin, LOW);
	}
  
	else if (buffer[0] == 'p'){ // spotlight
		analogWrite(spotPin, buffer[1]);
	}

	else if(buffer[0] == 'v') { // camera to position
		camservo.attach(servoPin);  
		camservo.write(buffer[1]); 
	}

	else if(buffer[0]== 'w') camservo.detach();
	
	else if (buffer[0] == 'm') {  // camera horiz position to eeprom
		EEPROM.write(eepromAddress, buffer[1]);
	}

	// always set speed on each move command 
	else if(buffer[0] == 'f' || buffer[0] == 'b' || buffer[0] == 'l' || buffer[0] == 'r'
			|| buffer[0] == 'z' || buffer[0] == 'e' || buffer[0] == 't' || buffer[0] == 'u') {
		analogWrite(pwmA, buffer[1]); 
		analogWrite(pwmB, buffer[2]);

		if (buffer[0] == 'f') { // forward
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd =1;
		}

		else if (buffer[0] == 'b') { // backward
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd = 2;
		}

		else if (buffer[0] == 'l') { // left
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd=3;
		} 
		
		else if (buffer[0] == 'r') { // right
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd=4;
		}
		
		else if (buffer[0] == 'z') { // lefttimed
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd=3;
			// unsigned long delay = buffer[3]<<8 | buffer[4];
			// if (delay > 1000) { delay = 1000; }
			// stoptime = time + delay;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
		
		else if (buffer[0] == 'e') { // righttimed
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd=4;
			// unsigned long delay = buffer[3]<<8 | buffer[4];
			// if (delay > 1000) { delay = 1000; }
			// stoptime = time + delay;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
		else if (buffer[0] == 't') { // forwardtimed
			digitalWrite(in1, WLOW);
			digitalWrite(in2, WHIGH);
			digitalWrite(in3, WHIGH);
			digitalWrite(in4, WLOW);
			
			stopPending = false;
			stopped = false;
			directioncmd=1;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
		else if (buffer[0] == 'u') { // backwardtimed
			digitalWrite(in1, WHIGH);
			digitalWrite(in2, WLOW);
			digitalWrite(in3, WLOW);
			digitalWrite(in4, WHIGH);
			
			stopPending = false;
			stopped = false;
			directioncmd=2;
			stoptime = time + (buffer[3]<<8 | buffer[4]);
		}
	
	}

	else if (buffer[0] == 's') { // stop
		stop();
	}
	
	else if (buffer[0] == 'h') { // hard stop
		digitalWrite(in1, 0);
		digitalWrite(in2, 0);
		digitalWrite(in3, 0);
		digitalWrite(in4, 0);
		if (!stopped) {
			stopCommand = time;
			stopPending = true;	
		}
	}

	else if(buffer[0] == 'x') Serial.println("<id::malgdb>");

	else if(buffer[0] == 'y') version();
	
	else if (buffer[0] == 'c') Serial.println(""); // ping, single character to make it quick
  
	else if (buffer[0] == 'i') { // gyro and encoder start
		encoderPinAtZero = false;
		encoderTicks = 0;
		readEncoder = true;	
		
		readAngle = true;
		angle = 0;
		stopdetected = false;
		stopPending = false;
		stopped = true;
		directioncmd = 0;
	}
	else if (buffer[0] == 'j') { // gyro and encoder stop and report
		printMoved();
		readEncoder = false;
		encoderPinAtZero = false;
		readAngle = false;
	}
	else if (buffer[0] == 'k') { // ODOMETRY_REPORT 
		printMoved();	
	}
	else if (buffer[0] == 'g') gyroTest();
	
	else if (buffer[0] == '8') { // clear eeprom
		for (int i = 0; i < 512; i++)
			EEPROM.write(i, 0);
		Serial.println("<eeprom erased>");
	}
	else if (buffer[0] == '9') { // read eeprom
		int i = EEPROM.read(eepromAddress);
		Serial.print("<horiz_eeprom: ");
		Serial.print(i);
		Serial.println(">");
	}
	else if (buffer[0] == 'a') {
		// I2c.write(SLAVEI2C, 0x00, buffer[1]); // TODO: fix
	}	
	
/* end of command buffer[0] list */	

	
}


boolean getGyroFIFOcontents() {
	
	// FIFO_SRC_REG  2F  
	// WTM OVRN EMPTY FSS4 FSS3 FSS2 FSS1 FSS0
	byte a = gyroRead(0x2F);
	gyrosamples = a & B00011111; // 31 max
	int ovr = bitRead(a,6);
	gyrosamples += ovr; // add OVR bit
	
	// TODO: testing
	// note: always triggered after starting from full stop
	if (ovr==1 && readAngle && gyroFifoReadAfterStop) Serial.println("<gyroOVR>");  // && !stopped 
	if (!stopped) gyroFifoReadAfterStop = true;
		
	if (gyrosamples == 0) {
		// Serial.println("<gyrosamples == 0>"); // TODO: Testing
		return false; 
	 }
	
	// read Z only 
	int i = 0;
	for (int n=0; n < gyrosamples; n++ ) {
		Wire.beginTransmission(GYROaddr);
		Wire.write(0x2C | (1 << 7)); // shift required to read multiple vals
		Wire.endTransmission();
		Wire.requestFrom(GYROaddr, 2);
		
		int za =0;
		int zb = 0;
		while (Wire.available()) {
			za= Wire.read();
			zb=Wire.read();
		}
		
		gyroZ[n] = zb<<8 | za;
	}
	
	// if (ovr==1 && readAngle) return false; // TODO: testing
	
	return true;
}

void gyroTest() {
	Serial.print("<");

	gyroRead();
	Serial.print("z: ");
	Serial.println(gyroZ[0]);
	
	if (!getGyroFIFOcontents()) {
		Serial.println("<getGyroFIFOcontents false>");
		return;
	}
	
	Serial.print("<");
	
	Serial.print("zOff: ");
	Serial.println(zOff);

	Serial.print("gyrosamples: ");
	Serial.println(gyrosamples);
			
	int zavg = 0;
	for (int i=0; i<gyrosamples; i++ ) {
		Serial.print(gyroZ[i]);
		Serial.print(" ");	
		zavg += gyroZ[i];	
	}
	Serial.println("");

	zavg /= gyrosamples;
	Serial.print("zavg: ");
	Serial.print(zavg);
	
	Serial.println(">");
	
}

void gyroRead() {
	Wire.beginTransmission(GYROaddr);
	Wire.write(0x2C | (1 << 7)); // shift required to read multiple vals
	Wire.endTransmission();
	Wire.requestFrom(GYROaddr, 2);
	
	int za =0;
	int zb = 0;
	while (Wire.available()) {
		za= Wire.read();
		zb=Wire.read();
	}
	
	gyroZ[0] = zb<<8 | za;
}

void printMoved() {
	double revs = 0;
	if (directioncmd ==1 || directioncmd == 2) {
		revs = (double) encoderTicks/gearRatio;
		if (directioncmd==2) revs *= -1;
	}
	encoderTicks = 0;
	double a = angle; // * calibrationComp;
	angle = 0;
	Serial.print("<moved ");
	Serial.print(revs);
	Serial.print(" ");
	Serial.print(a);
	Serial.println(">");
}


void stop() {
	analogWrite(pwmA, 0); 
	analogWrite(pwmB, 0);
	
	if (!stopped) {
		if (readAngle || readEncoder) {
			stopCommand = time;
			stopPending = true;	
			stoptime = 0;
		}
		else stopped = true;
	}
}

void stopDetect() {
	printMoved(); 
	Serial.println("<stop>");
	stopPending = false;
	stopped = true;
	directioncmd = 0;
	nextGyroZero = time + GYROZEROINTERVAL; 
	gyroFifoReadAfterStop = false;
}

ISR(PCINT1_vect) {
	if (!readEncoder) return; 
	
	int m = digitalRead(encA);
	if (m == LOW) encoderPinAtZero = true; 
	else {
		if (encoderPinAtZero) { // tick
			encoderPinAtZero = false;
			encoderTicks ++;
			unsigned long t = millis()/timemult;
			if (t - lastEncoderTick > 25) { stopdetected = true; } // was 25
			lastEncoderTick = t;
		}
	}
}

void gyroWrite(int addr, int value) {
	Wire.beginTransmission(GYROaddr);
	Wire.write(addr); 
	Wire.write(value);
	Wire.endTransmission();
}

byte gyroRead(int addr) {
	Wire.beginTransmission(GYROaddr);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(GYROaddr, 1);
	byte a = Wire.read();
	return a;
}

void version() {
	Serial.println("<version:1.13>"); 
}

