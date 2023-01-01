

/*Arduino code for Mega2560 board (FA-DUINO-12RA)
Basic roof controller with two tier operation. Normal operation has a safety function that prevents roof from moving if the mount is not parked.
There is also an override set of controls that ignore safety sensors. These are not triggered by normal ASCOM commands.
The code does not react to rain but can detect it. If roof does not move within a certain time period it times-out.
Some possible expansions are protected for but not implemented, for example environmental sensing
*/

// SW version 3.0 Dec '22 - added beepcontrol - make sure to match with latest ASCOM and Obsy control
// SW version 2.9 - changed Baud to 19200 (check that ASCOM driver is the same),faster status response update after open or close command
// SW version 2.8 - double-inverted digital inputs and moved mount sensor to port 8 Dec 2016 - improved EEPROM write robustness
// SW version 2.7 - mount off park alarm was until mount is parkeds
// SW version 2.6 - Feb '21 - slight change in open command, recognizing already open before mount position to avoid unecessary error
// also decided to output sensor override status values ilo cloud and 
// swapped mount power to alarm output and added delay to its setting
// SW version  2.5
// introduced relay control of Rain detector power and added reset to initialise
// added abort command for stopping roof
// SW version  2.4
// added forceopen and forceclose - to ignore safetysensors - not standard ASCOM command (from my obsy controller)
// added safetysensor enable/disable function, stored in EEPROM - so that the system can act 'dumb'  (from my obsy controller)
// faster response time 2s, rather than 4s
// detects crisis and broadcasts if mount moves when roof is closed


#include <avr/wdt.h>   // library which has a watchdog function
#include <EEPROM.h>  // ( EEPROM for Arudino IDE,)

// Pin designations for Arduino controller

// inputs (debounce is done in hardware, not software)
const int RAIN = 5; //rain sensor input, raining when LOW
const int PARKED = 6; // mount in park position sensor when HIGH
const int OPEN = 4; // roof in fully open position when HIGH
const int CLOSED = 7; // for a full closed position sensor when HIGH.
					  // reserved for possible expansion
const int IR = 6; // for sky temp (PWM) - not used
const int LIGHT = 9; // for sky brightness - not used
const int AMBIENT = 10; // for ambient temp - not used
const int WIND = 11; // for wind speed - not used

 // outputs
const int DEBUG = 13; // led for checking inputs
const int OPENROOF = 22; // relay to open roof HIGH =active
const int CLOSEROOF = 23; // relay to close roof HIGH =active
const int ALARM = 24; // relay for mount power (optional)
const int RAINPOWER = 25; // relay to turn power on to rain sense
// global timer variable to check roof move completed in time
const unsigned long MOVETIME = 25000; // timeout period for roof to move (25 seconds)
const unsigned long SAFETIME = 45000;  // timeout period before alarm goes off
const unsigned long POLLTIME = 1500;  // period between status broadcasts (1.5 seconds)
unsigned long rooftimer;  // variable to hold current time for roof movement
unsigned long polltimer; // variable to prompt status update
unsigned long safetimer;  // variable to hold current elapsed time for unsafe condition


// enumerations (enum functions don't work unless placed in header file)
// these mimic ASCOM enumeration of ShutterState command
int shutterStatus; // global variable used in all functions that holds following enumeration:
const int shutterOpen = 0;
const int shutterClosed = 1;
const int shutterOpening = 2;
const int shutterClosing = 3;
const int shutterError = 4;  // used for if roof does not move - error state


// for internal use when instructing movements
const int close = 0; // close command
const int open = 1; // open command
bool override;  // flag used for special instances to override safety sensors
bool mountsafe;  // flag used to detect mount moving from safe to unsafe with roof close

// for memory functions, so the Arduino can assume different guises
int rainflag = 0;  // EEPROM address of rain sensor enable flag
int parkflag = 1;  // EEPROM address of park sensor enable flag
int beepflag = 2; // EEPROM address of beep  enable flag
byte parksensor = 0;
byte rainsensor = 0;
byte beepenable = 0;


void setup()
{
	// I/O setup for board - on Arduino, this is not the same serial port used for programming.
	Serial1.begin(19200);  // upped from 9600
	Serial1.flush();  // clear output buffer
					  // set pin modes for each of the I/O
	pinMode(OPENROOF, OUTPUT);
	pinMode(CLOSEROOF, OUTPUT);
	pinMode(RAIN, INPUT);
	pinMode(PARKED, INPUT);
	pinMode(OPEN, INPUT);
	pinMode(CLOSED, INPUT);
	pinMode(ALARM, OUTPUT);
	pinMode(RAINPOWER, OUTPUT);
	pinMode(IR, INPUT);
	pinMode(LIGHT, INPUT);
	pinMode(AMBIENT, INPUT);
	pinMode(WIND, INPUT);
	Initialise();  // sets up initial relay states and status variables
	wdt_enable(WDTO_8S);  // set up watchdog to kick in if it is not reset in 8 seconds
	shutterStatus = shutterClosed;   // initial state of roof
	mountsafe = true; // assume mount is safe on powerup
}

void loop()
// loop scans serial port for valid command or checks roof status and updates dynamic movements and status
{
	String cmd;  // for received string commands from serial port via ASCOM

	if (Serial1.available() > 0) 
	{
		cmd = Serial1.readStringUntil('#');
		if (cmd == "CLOSE") CloseRoof();
		else if (cmd == "OPEN") OpenRoof();
		else if (cmd == "INIT") Initialise();
		else if (cmd == "ABORT") AbortRoof();
		else if (cmd == "FORCEOPEN") ForceOpen();
		else if (cmd == "FORCECLOSE") ForceClose();
		else if (cmd == "NORAINSENSE") DisableRainSensor();
		else if (cmd == "NOPARKSENSE") DisableParkSensor();
		else if (cmd == "RAINSENSE") EnableRainSensor();
		else if (cmd == "PARKSENSE") EnableParkSensor();
		else if (cmd == "BEEPON") EnableBeeper();
		else if (cmd == "BEEPOFF") DisableBeeper();
	}
	DynamicRoof();  // check on ongoing actions and update as necessary
	DynamicMount(); // check for collision conditions
	wdt_reset(); // reset watchdog timer on each cycle
	PollStatus();  // send status string to serial and alarm states
}

// one-time only move roof, ignoring sensors (compare with OpenRoof)
void ForceOpen()
{
	if (RoofOpen()) shutterStatus = shutterOpen; // no action, already there
	else
	{
		MoveRoof(open); // or move roof and change status
		override = true;  // tell DynamicRoof() to override safety sensors
	}
}

// one-time only move roof, ignoring sensors void (compare with CloseRoof)
void ForceClose()
{
	if (RoofClosed()) shutterStatus = shutterClosed;	// no action, already there
	else
	{
		MoveRoof(close); // or move roof and change status
		override = true; // tell DynamicRoof() to override safety sensors
	}
}

// these routines set a flag in EEPROM to allow the Arduino to work without park and rain sensors
// these flags are used by the routine that reads the sensors and forces a safe sensor status if disabled
void DisableRainSensor()
{
	if (EEPROM.read(rainflag) != 0) EEPROM.write(rainflag, 0);
}

void DisableParkSensor()
{
	if (EEPROM.read(parkflag) != 0) EEPROM.write(parkflag, 0);
}

void EnableRainSensor()
{
	if (EEPROM.read(rainflag) != 1) EEPROM.write(rainflag, 1);
}

void EnableParkSensor()
{
	if (EEPROM.read(parkflag) != 1) EEPROM.write(parkflag, 1);
}

void DisableBeeper()
{
	if (EEPROM.read(beepflag) != 0) EEPROM.write(beepflag, 0);
	digitalWrite(ALARM, LOW); // also turns off beeper

}
void EnableBeeper()
{
	if (EEPROM.read(beepflag) != 1) EEPROM.write(beepflag, 1);
	digitalWrite(ALARM, HIGH); // beep to acknowledge
	delay(1000);
	digitalWrite(ALARM, LOW);
}

// generic ASCOM commands to open abort and close
void CloseRoof()
// Close roof if and when mount is parked. Do nothing if already open
{
	if (RoofClosed()) shutterStatus = shutterClosed;	// no action, already there
	else if (!MountParked()) shutterStatus = shutterError;  // no action 
	else
	{
		override = false;
		MoveRoof(close); // or move roof and change status
	}
}

// stops roof moving - by reversing and pausing if closing or pausing whilst opening
void AbortRoof()
{
	if (shutterStatus == shutterClosing) // if closing, first reverse
	{
		digitalWrite(CLOSEROOF, LOW); // precaution
		digitalWrite(OPENROOF, HIGH);
		delay(300);
		digitalWrite(OPENROOF, LOW);
		rooftimer = millis(); // start timer
		shutterStatus = shutterOpening;  // ASCOM status
		Status();
	}

	if (shutterStatus == shutterOpening) // if opening, pause
	{
		digitalWrite(OPENROOF, LOW); // precaution
		digitalWrite(CLOSEROOF, HIGH);
		delay(300);
		digitalWrite(CLOSEROOF, LOW);
		rooftimer = millis(); // start timer
		shutterStatus = shutterClosing;  // ASCOM status
		Status();
	}
}

void OpenRoof()
// Open roof if mount is parked, roof is closed and it is not raining.  Do nothing if already open.
{
	if (Raining()) shutterStatus = shutterError;  // do not open if it is raining
	else if (RoofOpen()) shutterStatus = shutterOpen; // no action, already there
	else if (!MountParked()) shutterStatus = shutterError; // do not open if mount is unparked
	else
	{
		MoveRoof(open); // or move roof and change status
		override = false;
	}
}

// consolidated intelligent move command which works out what to do in light of the latest command and the prior state
// roof controls into motor system are assymetric. If roof is moving it reverses on open command but pauses on close command
void MoveRoof(int command)
{
	if (command == close)
	{
		digitalWrite(OPENROOF, LOW); // should be redundant but no harm	
		if (shutterStatus == shutterOpening) // pause opening roof so you can close it
		{
			digitalWrite(CLOSEROOF, HIGH);  // to ensure pauses opening
			delay(300);
			digitalWrite(CLOSEROOF, LOW);  // pulse command only
			delay(300);
		}
		shutterStatus = shutterClosing;  // update ASCOM status
		Status(); // broadcast status
		digitalWrite(CLOSEROOF, HIGH); // start roof closing 
		delay(300);
		digitalWrite(CLOSEROOF, LOW);
		rooftimer = millis(); // start timer for roof movement
	}

	else if (command == open)
	{
		shutterStatus = shutterOpening;  // ASCOM status
		Status(); // broadcast status
		digitalWrite(CLOSEROOF, LOW); // should be redundant but no harm
		digitalWrite(OPENROOF, HIGH); // start roof opening
		delay(300);  // pulse commmand only
		digitalWrite(OPENROOF, LOW);
		rooftimer = millis(); // start timer
	}
}

// DynamicRoof checks roof actions in progress and concludes relay settings and updates roof status
// updated to check for independent roof operation outside this driver logic
// updated for all combinations of status and sensor readings
// if roof is moved manually, it cannot tell if it is opening or closing until it gets to a switch position

void DynamicRoof()
{
	if (RoofOpen()) // sensor reading status
	{
		// shutterStatus is the software 'status' but may not reflect reality
		if (shutterStatus == shutterOpen) rooftimer = millis(); // reset timer if roof in rest state
		if (shutterStatus == shutterClosed) shutterStatus = shutterOpen; // override if there was some manual intervention
		if (shutterStatus == shutterOpening)  // check to see if dynamic status needs updating
		{
			shutterStatus = shutterOpen;  // update ASCOM status
			rooftimer = millis();  // reset timer, roof has arrived at intended location
			override = false;  // reset override once operation is complete
		}
		if (shutterStatus == shutterClosing)  // has it been moving for too long?
		{
			if ((millis() - rooftimer) > MOVETIME) // check for timeout
			{
				shutterStatus = shutterError; // change ASCOM status for error condition
				override = false;  // reset override once operation is complete
			}
		}
		else if (shutterStatus == shutterError)
		{
			// do nothing at the moment
		}
	}
	else if (RoofClosed()) // sensor reading status
	{
		if (shutterStatus == shutterClosed) rooftimer = millis(); // reset timer if roof in rest state
		if (shutterStatus == shutterOpen) shutterStatus = shutterClosed; // override if there was some manual intervention
		if (shutterStatus == shutterClosing)
		{
			shutterStatus = shutterClosed;  // update ASCOM status
			rooftimer = millis();  // reset timer, roof has arrived at intended location
			override = false;  // reset override once operation is complete
		}
		if (shutterStatus == shutterOpening) // check for extended opening times
		{
			if ((millis() - rooftimer) > MOVETIME) // check for timeout
			{
				shutterStatus = shutterError; // update ASCOM status
				override = false;  // reset override once operation is complete
			}
		}
		else if (shutterStatus == shutterError)
		{
			if (MountParked() && !Raining()) shutterStatus = shutterClosed;  // clear error if rain has stopped and mount is parked
		}
	}
	// check for time-out when not open or closed
	else if ((millis() - rooftimer) > MOVETIME) // check for timeout
	{
		shutterStatus = shutterError; // ASCOM status
		override = false;  // reset override once operation is complete
	}
}

// checks for mount moving from park position with roof closed and issue immediate status
void DynamicMount()
{
	if (mountsafe) // was safe
	{
		if (RoofClosed() && !MountParked() && !override) // still moving but mount is not parked and no safety override
		{
			shutterStatus = shutterError;  // ASCOM status - caused by mount not being in position.
			mountsafe = false;
			Status();  // broadcast immediately
			if(EEPROM.read(beepflag)) digitalWrite(ALARM, HIGH); // turn alarm on if enabled
		}
		else if (RoofClosed())
			digitalWrite(ALARM, LOW); // turn alarm off
	}
	else  // reset flag if roof is opened or mount is parked
	{
		if (MountParked()) mountsafe = true;
		if (RoofOpen()) mountsafe = true;
		if (override) mountsafe = true;
	}

}

// Method to detect if it is raining true = raining
bool Raining()
{
	if (EEPROM.read(rainflag) == 1) return(digitalRead(RAIN) == LOW); //simple relay - reversed polarity in 2.7
	else return (false);
}

// Method to detect if mount is parked true if parked - override means it is always 'safe'
bool MountParked()
{
	if (digitalRead(PARKED) == 0) digitalWrite(DEBUG, 0);
	else digitalWrite(DEBUG, 1);
	if (EEPROM.read(parkflag) == 1) return(digitalRead(PARKED) == HIGH);
	else return (true);
}

// Method to detect if roof is fully OPEN true if roof full open
bool RoofOpen()
{
	return(digitalRead(OPEN) == HIGH);
}

// Method to detect if roof is fully CLOSED true if fully closed
bool RoofClosed()
{
	return(digitalRead(CLOSED) == HIGH);
}

// returns rain sensor enable flag
bool RainSenseFlag()
{
	return (EEPROM.read(rainflag) == 1);
}

// returns mount sensor enable flag
bool MountSenseFlag()
{
	return (EEPROM.read(parkflag) == 1);
}

// method to return beeper enable flag

bool BeepEnableFlag()
{
	return (EEPROM.read(beepflag) == 1);
}


void PollStatus()
{
	if ((millis() - polltimer) > POLLTIME)
	{
		Status(); // broadcast serial
		polltimer = millis();  // reset communication timer
		SafetyEvent();  // check for rain and do beeper / close roof automatically
	}
}

void SafetyEvent()  // check rain sensor (switch closure) and compare with roof status)
{
	if (Raining())
	{
		if (shutterStatus == shutterOpen)
		{
			if ((millis() - safetimer) > SAFETIME)
			{
				if (EEPROM.read(beepflag)) digitalWrite(ALARM, HIGH); // turn alarm on if enabled
				if (MountParked()) MoveRoof(close);
			}
		}
		else if (RoofClosed())  // when roof is closed, switch off alarm
		{
			if (MountParked())
			{
				digitalWrite(ALARM, LOW); // turn off any alarm (if used)
				safetimer = millis();  // and reset timer
			}
		}
	}
	else
	{
		safetimer = millis();  // and reset timer
	}
}

// Status - broadcasts status in fixed format message every POLLTIME ms:
//  "$Dry,Roof,Park,rainsense,mountsense,beepstatus#" where Dry and Park are boolean 1/0 and Roof is enumerated 0-4
// "$b,e,b,b,b,b#"  b - boolean, e - enumerated, 11 chars between end markers
// also detects rainstatus and sets alarm / closes roof (if safe)
void Status()  // broadcast status over serial port
{
	Serial1.print("$");
	Serial1.print(Raining());
	Serial1.print(",");
	Serial1.print(shutterStatus);
	Serial1.print(",");
	Serial1.print(MountParked());
	Serial1.print(",");
	Serial1.print(RainSenseFlag());
	Serial1.print(",");
	Serial1.print(MountSenseFlag());
	Serial1.print(",");
	Serial1.print(BeepEnableFlag());
	Serial1.print("#");
}

// Method to initialise state and reset power relays
void Initialise()
{
	// setup relays and initial status	
	digitalWrite(OPENROOF, LOW);
	digitalWrite(CLOSEROOF, LOW);
	digitalWrite(ALARM, HIGH);  // for diagnostics
	delay(500);				// for diagnostics
	digitalWrite(ALARM, LOW); // turn off any alarm (if used)
	delay(500);
	digitalWrite(ALARM, HIGH);  // for diagnostics
	delay(500);				// for diagnostics
	digitalWrite(ALARM, LOW); // turn off any alarm (if used)
	digitalWrite(RAINPOWER, LOW); // turn off / reset rain sensor
								  // delay for power to decay
	delay(1000);
	digitalWrite(RAINPOWER, HIGH); // turn on rain sensor
	shutterStatus = shutterClosed;  // default shutter state
	override = false;  // default operations are safe ones
	if (RoofOpen()) shutterStatus = shutterOpen;  // override if one of the proximity switches is closed
	rooftimer = millis();  // reset operational timers
	polltimer = millis();  // reset operational timers
	safetimer = millis(); // reset operational timerss
							 // flush out input buffer
	char c;
	while (Serial1.available() > 0)
	{
		c = Serial1.read();
	}
	// check for valid EEPROM values and if invalid, initialise set to 1 to enable
	rainsensor = EEPROM.read(rainflag);
	if (rainsensor != 1 && rainsensor != 0)  // initialise only if invalid byte in eeprom
	{
		EEPROM.write(rainflag, 1);   // default is to have sensors/alarm
		EEPROM.write(parkflag, 1);
		EEPROM.write(beepflag, 1);
	}
}
