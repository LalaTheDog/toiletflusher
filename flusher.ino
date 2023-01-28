#include <Stepper.h>
#include <i2cmaster.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/io.h>


#include <Wire.h>




#define STEPS 100


const int QUADRATURE_A = 2;           // PD2
const int QUADRATURE_B = 3;           // PD3
const int QUADRATURE_S = 4;           // PD4


const int POTENTIOMETER_A = A3;

// This is the US5881LUA Hall sensor.
// you want the flat 'back' side of the sensor to face the south pole of the magnet.
// with the package marking facing you, left to right, the pins are vcc, gnd, signal (pull this pin to high on a 10k pullup).
// signal will pull low on signal.

// Which sensor do you want to use as the spin interrupt?
const int STOP_SENSOR_IN_USE = 6;

const int STOP_SENSOR_IN_USE_INTERRUPT = PCINT22;  // Pin change interrupt on PD6

const int STATUS_LED = 9;
const int STEP_LED = 5;

const int PULSE_PER_REV = 200;

const bool DISPLAY_SCREEN = false;

// These are probably actually set up in the i2c WIRE library, but I'm putting these here
// for reference if you need to remember where to attach the wire
const int I2C_A_PIN = 28; // usually labeled SCL on PU package
const int I2C_B_PIN = 27;// usually labeled SDA on PU package
//attach pin 28 -> 28 on display slave
//attach pin 27 - 27 on display slave

// STRUCT FOR HOLDING ROTARY ENCODER VALUES
// GLOBAL VARIABLES

long Vin = 3300;
long Temp = 250000;
long Time;

byte mlxAddress = 0x5A;


//ain1 = a6
//ain2 = d10
//bin1 = d7
//bin2 = d8

//Stepper stepper(STEPS, ain1, AIN2, bin2, bin1);
//Stepper bstepper(STEPS,bin2, bin1, ain2, ain1;


// 360/1.8 degrees = 200 steps
//360/18 degrees = 20 steps
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
const byte pirPin = 7;


Stepper stepper(STEPS, A1, 10, 8, 7);
Stepper bstepper(STEPS,8, 7, 10, A1);
const byte resistSwitch = A2;  

const byte zigbeePin = A0;

const int distanceToLalaButt = 50;

int pirPinStatus;
int ultrasoundStatus;

//const unsigned long motionWindowTime = 10*1000;
const unsigned long minimumPeePeeTime = 20*1000;
const unsigned long extraDelayTime = 10*1000;
const byte debounceCount = 10;
unsigned long lastPirReadTime = millis();
unsigned long peePeeStartTime = millis();

double temperatureDifference;

// THIS TRACKS WHERE THE FLYWHEEL IS AT
volatile int rotaryPosition = 0;

int flushDelay = 2000;

// STRUCT FOR HOLDING ROTARY ENCODER VALUES
struct quadrature_encoder {
	uint8_t pinstate;
	uint8_t previous;
	uint8_t motion;
	uint8_t motionstate;
};

struct quadrature_encoder myDial;

// THESE ARE FOR ISR USE
volatile int photoTransistorStatusRead;
volatile int Count = 0;

// THESE ARE FOR ISR VERSION OF DEBOUNCING
// HIGH ISR RATES CAN HANG THE CHIP THO, SO...
// Called when encoder value changes
void ChangeValue (bool Up) {
	// digitalWrite(LED, Up);
	Count = max(min((Count + (Up ? 1 : -1)), 1000), 0);
	Serial.println(Count);
	if (Up == 1) {
		Serial.println(F("right"));
	} else {
		Serial.println(F("left"));
	}
}

// Pin change interrupt service routine
ISR (PCINT2_vect) {
	rotaryDial();
	int photoTransistorStatus = PIND>>STOP_SENSOR_IN_USE & 1;
	// PINC is the port C input register for the analog pins.  Read only.
	// PIND is digital pints 0-7
	if (photoTransistorStatus != photoTransistorStatusRead) {
		photoTransistorStatusRead = photoTransistorStatus;
		Serial.println(F("Change on photo transistor"));
		Serial.println(photoTransistorStatusRead, BIN);
		// pin pulled low means IR is detected.
		// So LED on, means IR detected
		// the hall sensor also pulls down when the magnet's south pole is near it. (unipolar, the north pole won't do anything)
		if (photoTransistorStatusRead == 0) {
			//digitalWrite(DRIVER_ENABLE, HIGH);
		//	digitalWrite(STATUS_LED, HIGH);
		} else {
		//	digitalWrite(STATUS_LED, LOW);

		}
	}
}


void wake ()
{
	// cancel sleep as a precaution
	sleep_disable();
	// precautionary while we do other stuff
	detachInterrupt (0);
}  // end of wake

void setup () 
{
	pinMode(pirPin, INPUT);
	digitalWrite (pirPin, LOW);

	pinMode(STEP_LED, OUTPUT);
	digitalWrite(STEP_LED, LOW);


	pinMode(zigbeePin, OUTPUT);
	digitalWrite(zigbeePin, LOW);



	pinMode(resistSwitch, INPUT_PULLUP); // enable pullup on pin resistSwitch 
	pinMode(STATUS_LED, OUTPUT);
	digitalWrite(STATUS_LED, LOW);
	Serial.begin(9600);
	delay(1000);
	Serial.println("Stepper test!");
	// set the speed of the motor to 30 RPMs
	stepper.setSpeed(60);
	bstepper.setSpeed(60);
	Serial.println("Goodmorning");

	i2c_init(); 


	Wire.begin();

	PCMSK2 = 1<<STOP_SENSOR_IN_USE_INTERRUPT;
// pins 2 3 and 4
	PCMSK2 |= bit (PCINT18);
	PCMSK2 |= bit (PCINT19);
	PCMSK2 |= bit (PCINT20);
	// Configure pin change interrupt
	// Pin change interrupt control register - enables interrupt vectors
	// Bit 2 = enable PC vector 2 (PCINT23..16)
	// Bit 1 = enable PC vector 1 (PCINT14..8)
	// Bit 0 = enable PC vector 0 (PCINT7..0)
	PCICR = 1<<PCIE2;
	// Enable interrupt
	PCIFR = 1<<PCIF2;
	// Clear interrupt flag




	// WRITE THE PINS TO INITIAL STATE
	pinMode(QUADRATURE_A, INPUT_PULLUP);
	pinMode(QUADRATURE_B, INPUT_PULLUP);
	pinMode(QUADRATURE_S, INPUT_PULLUP);


//		pinMode(DRIVER_STEP, OUTPUT);
//		pinMode(DRIVER_DIR, OUTPUT);
//		pinMode(DRIVER_ENABLE, OUTPUT);
//		digitalWrite(DRIVER_ENABLE, HIGH);

	pinMode(POTENTIOMETER_A, INPUT_PULLUP);
	pinMode(STOP_SENSOR_IN_USE, INPUT_PULLUP);



	pinMode(STEP_LED, OUTPUT);
	digitalWrite(STEP_LED, LOW);  





	// USE THESE IF DEBOUCING WITH POLLING
	myDial.pinstate = 0;
	myDial.previous = 0;
	myDial.motion = 0; 
	myDial.motionstate = B1110;


	// BEING SERIAL COMMUNICATION
	Serial.begin(9600);
	Serial.println("Ready");


}  // end of setup

int pushButtonAcceptedState;
int lastPushButtonReading;
unsigned long debounceTime = 0;
unsigned long debounceDelay = 50;



unsigned long sleepIdleTimeout = 30000;
unsigned long sleepIdleSinceTime = 0;

//700 for the little stepper
// untested for the large
int DEFAULT_PULSE_WIDTH = 400;  
int pulseWidth = DEFAULT_PULSE_WIDTH;  

volatile int package;

void sendToScreen (String message) {
	Wire.beginTransmission(8);
	Wire.write(message.c_str());
	Wire.endTransmission();
}

void loop () 
{



	
	

	// READ THE STATUS OF THE PUSH BUTTON TO DO STUFF ABOUT IT
	int pushButtonReading = digitalRead(QUADRATURE_S);
	if (pushButtonReading != lastPushButtonReading) {
		debounceTime = millis();
		//Begin required hold time
	}
	lastPushButtonReading = pushButtonReading;

	if ((millis() - debounceTime) > debounceDelay) {
		if (pushButtonReading != pushButtonAcceptedState) {
			pushButtonAcceptedState = pushButtonReading;

			if (pushButtonAcceptedState == LOW) {
				Count = 0;
				Serial.println(F("Reset Count to 0"));
				//spinToStop();
				flushDelay = 3000;
				sleepIdleSinceTime = millis();
			}
		}
	}

	//I don't remember what's going on, but I want to switch to a pedal.
	// the pedal is a 1kpot at max press, 55k pot at rest.
	// it reads about 175 at rest, and 45 fully depressed.

	int pedalReading = analogRead(POTENTIOMETER_A);
	if (pedalReading < 160) {
		while (analogRead(POTENTIOMETER_A) < 160) {
			int pedalReading = analogRead(POTENTIOMETER_A);
			if (pedalReading > 55) {
				pulseWidth = (pedalReading)*15+DEFAULT_PULSE_WIDTH;
			} else {
				pulseWidth = DEFAULT_PULSE_WIDTH;
			}
			// modifying the function of the magnet.  I want the needle to slow down during the punching phase
			// so I can control where the needle punctures very accurately.
			// if a magnet is in view, slow it down
			// I can line the gear with magnets in the region where I want the needle to go slower.
			// I don't remember how I set up the hall sensor, but it looks like it's on an interrupt, so this value should
			// update regardless, it looks like a volatile ISR routine pin
			// I'm not sure if I need to control the speed here, or if I'll have to drop into the spinForever routine.  
			// probably depends on how many pulses it sends relative to rotation on the motor
			// I'm going to try here first
			//if (photoTransistorStatusRead == 0) {
			//    pulseWidth = pulseWidth * 6;
			// }
			Serial.println(pedalReading);
			//spinForever();
			Serial.println("Pedal Flush Triggered");
			flush();
			break; // this is a while loop
		}

	}





	// check position of the flusher plunger



	if (digitalRead(resistSwitch)) {
		Serial.println("The plunger needs to be retracted");
		retractPlunger();	
	} else {
		//Serial.println("The plunger is already retracted");
		//pirPinStatus = digitalRead(pirPin);		
		pirPinStatus = photoTransistorStatusRead;;		
		if (pirPinStatus==1) {
			if (debounceCheck) {
				peePeeStartTime = millis();
				//Serial.println("PirPin is High");
				waitForPeePeePooPoo();
				long timeOnToilet = millis() - peePeeStartTime;
				if (timeOnToilet > minimumPeePeeTime) {

					digitalWrite(STEP_LED, HIGH);
					digitalWrite(zigbeePin, HIGH);
					// delay to let the camera snap a photo of the toilet
					delay(flushDelay);
					flush();
					digitalWrite(STEP_LED, LOW);
					digitalWrite(zigbeePin, LOW);
				} else {
					//Serial.print(timeOnToilet);
					sendToScreen((String) timeOnToilet);
					//Serial.println(" microseconds waited");
					//Serial.print(minimumPeePeeTime);
					sendToScreen((String) minimumPeePeeTime);
					//Serial.println(" Seconds on the Toilet. Probably just sniffing the toilet");
				}
			}

		}


	}

	unsigned long idleDuration = millis() - sleepIdleSinceTime;
	if (idleDuration > sleepIdleTimeout) {
		// SLEEP THE DRIVER
	if (digitalRead(resistSwitch)) {
		Serial.println("Retract before sleep");
		retractPlunger();	
	}
/*	
Serial.println("Goodnight");
	delay(6000);
	// Do not interrupt before we go to sleep, or the
	// ISR will detach interrupts and we won't wake.
	noInterrupts ();
	// disable ADC
	ADCSRA = 0;  

	set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
	sleep_enable();


	// will be called when pin D2 goes low  
	attachInterrupt (0, wake, RISING);
	EIFR = bit (INTF0);  // clear flag for interrupt 0

	// turn off brown-out enable in software
	// BODS must be set to one and BODSE must be set to zero within four clock cycles
	MCUCR = bit (BODS) | bit (BODSE);
	// The BODS bit is automatically cleared after three clock cycles
	MCUCR = bit (BODS); 

	// We are guaranteed that the sleep_cpu call will be done
	// as the processor executes the next instruction after
	// interrupts are turned on.
	interrupts ();  // one cycle
	sleep_cpu ();   // one cycle
		//digitalWrite(DRIVER_ENABLE, HIGH);

*/
	}

} // end of loop

void rotaryDial() {


	//package = 0;
	// BEGIN READING OF THE ROTARY ENCODER
	// OVERWRITE DIAL STATUS WITH NEW READINGS FROM THE ROTARY ENCODER
	myDial = quadrature(myDial);

	// DO STUFF BASED ON STATUS OF THE ROTARY ENCODER
	if (myDial.motion != 0) {

		// WAKE UP THE DRIVER
	//	digitalWrite(DRIVER_ENABLE, LOW);
		digitalWrite(STEP_LED, HIGH);

		// GOING CLOCKWISE
		if (myDial.motion == 1) {   
			// WE ARE GOING FORWARDS
			//digitalWrite(DRIVER_DIR, LOW);
			Count = Count + 1;

			flushDelay = flushDelay + 1000;
Serial.println(flushDelay);
			Serial.println("Delay Increased");
			// DRAW POSITION ON THE LCD
			// GOING COUNTER CLOCKWISE
		} else if (myDial.motion == 2) {
			// WE ARE GOING BACKWARDS, REVERSE THE DRIVER
			//digitalWrite(DRIVER_DIR, HIGH);
			Count = Count - 1;
			if (flushDelay > 1000) {
				flushDelay = flushDelay - 1000;
Serial.println(flushDelay);
			Serial.println("Delay Decreased");
			}

			// DRAW POSITION ON THE LCD
		}

		// SEND THE MOTION DIRECTIVE
		//Serial.println(pulsesCompleted);
		digitalWrite(STEP_LED, LOW);

		sleepIdleSinceTime = millis();

		////Serial.println(Count);


	}

}

void flush() {


	Serial.println("Flush");
	bstepper.step(STEPS);
	delay(10000);
	Serial.println("Reset");
	stepper.step(STEPS);

}

void retractPlunger() {
	Serial.println("Retract");
	stepper.step(STEPS);
}

void waitForPeePeePooPoo() {
	digitalWrite(STATUS_LED, HIGH);
	//Serial.println("PeePee Started");
	float targetTemperature = temperatureRead1(mlxAddress);
	float ambientTemperature = temperatureReadA(mlxAddress);
	temperatureDifference = targetTemperature - ambientTemperature;
	//Serial.println(targetTemperature);
	//Serial.println(ambientTemperature);
	//Serial.print("Initial Temp Diff:");
	sendToScreen((String) "Initial Temp Diff:");
	//Serial.println(temperatureDifference);
	int holdStatus = 0;
	if (temperatureDifference > 2) {
		holdStatus = 1;
	}
	while (holdStatus == 1) {
		if (pirPinStatus==0) {
			if (debounceCheck()) {
				if (debounceTemperature()) {
					//Serial.println("No motion, no temp diff");
					holdStatus = 0;
				}
			}
		} else {
			//Serial.println("There's either motion, or there's something in the way");
		}
		pirPinStatus = digitalRead(pirPin);		
	}
	digitalWrite(STATUS_LED, LOW);
}

bool debounceCheck() {

	int pirDebounceValue = pirPinStatus;
	int pirDebounceCount = 0;
	for (int i = 0; i < debounceCount ; i++) {

		if (pirDebounceValue == pirPinStatus) {
			delay(100);
			pirDebounceValue = digitalRead(pirPin);
			pirDebounceCount++;
		}	
	}

	if (pirDebounceCount >= debounceCount) {
		return true;
	}  
	return false;
}



bool debounceTemperature() {

	int temperatureDebounceCounter = 0;
	for (int i = 0; i < debounceCount ; i++) {

		delay(100);
		temperatureDifference = temperatureRead1(mlxAddress) - temperatureReadA(mlxAddress);
		if (temperatureDifference < 2) {
			temperatureDebounceCounter++;
			Serial.println("No Diff");
		}	
	}

	if (temperatureDebounceCounter >= debounceCount) {
		return true;
	} else {
		Serial.println("No match, debounce failed");
		return false;
	} 
}


float temperatureReadA(int address) {

	int deviceWrite = (address <<1 | B0);

	int deviceRead = (address <<1 | B1);

	int data_low = 0;
	int data_high = 0;
	int pec = 0;

	i2c_start_wait(deviceWrite);
	i2c_write(0x06);                  // Address of temp bytes

	// read
	i2c_rep_start(deviceRead);
	data_low = i2c_readAck();         //Read 1 byte and then send ack
	data_high = i2c_readAck();        //Read 1 byte and then send ack
	pec = i2c_readNak();
	i2c_stop();

	//This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
	float temperature = 0x0000;       // zero out the data

	// This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
	temperature = (float)(((data_high & 0x007F) << 8) + data_low);
	temperature = (temperature * 0.02) - 273.16;

	if (Vin>3000) {
		temperature = temperature - (0.6*((Vin/1000)-3));
	} else if (Vin<3000) {
		temperature = temperature + (0.6*(3-(Vin/1000)));
	}

	return temperature;

}

float temperatureRead2(int address) {

	int deviceWrite = (address <<1 | B0);
	int deviceRead = (address <<1 | B1);

	int data_low = 0;
	int data_high = 0;
	int pec = 0;

	i2c_start_wait(deviceWrite);
	i2c_write(0x08);                  // Address of temp bytes

	// read
	i2c_rep_start(deviceRead);
	data_low = i2c_readAck();         //Read 1 byte and then send ack
	data_high = i2c_readAck();        //Read 1 byte and then send ack
	pec = i2c_readNak();
	i2c_stop();

	//This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
	float temperature = 0x0000;       // zero out the data

	// This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
	temperature = (float)(((data_high & 0x007F) << 8) + data_low);
	temperature = (temperature * 0.02) - 273.16;

	if (Vin>3000) {
		temperature = temperature - (0.6*((Vin/1000)-3));
	} else if (Vin<3000) {
		temperature = temperature + (0.6*(3-(Vin/1000)));
	}

	return temperature;

}
float temperatureRead1(int address) {

	int deviceWrite = (address <<1 | B0);
	int deviceRead = (address <<1 | B1);

	int data_low = 0;
	int data_high = 0;
	int pec = 0;

	i2c_start_wait(deviceWrite);
	i2c_write(0x07);                  // Address of temp bytes

	// read
	i2c_rep_start(deviceRead);
	data_low = i2c_readAck();         //Read 1 byte and then send ack
	data_high = i2c_readAck();        //Read 1 byte and then send ack
	pec = i2c_readNak();
	i2c_stop();

	//This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
	float temperature = 0x0000;       // zero out the data

	// This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
	temperature = (float)(((data_high & 0x007F) << 8) + data_low);
	temperature = (temperature * 0.02) - 273.16;

	if (Vin>3000) {
		temperature = temperature - (0.6*((Vin/1000)-3));
	} else if (Vin<3000) {
		temperature = temperature + (0.6*(3-(Vin/1000)));
	}


	return temperature;

}

long readTemp() {
	long result;
	// Read temperature sensor against 1.1V reference
	ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	result = (result - 125) * 1075;
	return result; // Millis Cs
}

long readVcc() {
	long result;
	// Read 1.1V reference against AVcc
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(2); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	result = 1126400L / result; // Back-calculate AVcc in mV
	return result; // Milli Vs
}


struct quadrature_encoder quadrature(struct quadrature_encoder lastDial) {

	struct quadrature_encoder thisDial;
	thisDial.motion = 0;
	thisDial.previous = lastDial.pinstate;
	thisDial.motionstate = lastDial.motionstate;
	thisDial.pinstate = 0;
	uint8_t lookup = 0;


	// Input Pin PULLED LOW is signal
	if (digitalRead(QUADRATURE_A) == LOW) {
		thisDial.pinstate = thisDial.pinstate | (1 << 1);
	}
	if (digitalRead(QUADRATURE_B) == LOW) {
		thisDial.pinstate = thisDial.pinstate | (1 << 0);
	}

	lookup = (lastDial.pinstate << 2) | thisDial.pinstate;
	if (lastDial.pinstate != thisDial.pinstate) {
		if ((thisDial.motionstate & B1000) >> 3 == (thisDial.motionstate & B0010) >> 1) {
			// A was the same last time we moved
			if (lookup == B1000 || lookup == B0111) {
				// A rising against high B, or A falling against low B
				// go right
				thisDial.motion = 1;
				thisDial.motionstate = lookup;
			} else if (lookup == B0010 || lookup == B1101) {
				// A is falling against high B, or A rising against low B
				// go left
				thisDial.motion = 2;
				thisDial.motionstate = lookup;
			}
		} else if ((thisDial.motionstate & B0100) >> 2 == (thisDial.motionstate & B0001) >> 0) {
			// B was the same the last time we moved
			if (lookup == B1110 || lookup == B0001) {
				// B rising against low A, or B falling against high A
				// go right
				thisDial.motion = 1;
				thisDial.motionstate = lookup;
			} else if (lookup == B0100 || lookup == B1011) {
				// B is rising against high A, or B is falling against low A 
				// go left
				thisDial.motion = 2;
				thisDial.motionstate = lookup;
			}   
		}
	}

	return thisDial;
}


