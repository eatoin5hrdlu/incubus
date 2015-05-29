/*  Mini Incubator and TopAgar Prep Thermal Block */

// Set Target Temperature in degrees C here

double settemp = 49.0;      // 50 degrees C for TopAgar
// double settemp = 37.0;   // 98.6 F
// double settemp = 31.0;   // 87 F

// Set Failsafe shutdown time here
// The 8-hour shutoff was appropriate for Top Agar preparation
// CUTOFF_TIMEOUT must be changed for longer incubation periods.

#define SAMPLETIME_SECONDS 10    // How often to check temperature
#define CUTOFF_TIMEOUT ((8*60*60)/SAMPLETIME_SECONDS)  // 8 hour cutoff

// ((8*60*60)/SAMPLETIME_SECONDS)  // 8 hours in CYCLES
// ((24*60*60)/SAMPLETIME_SECONDS)  // 24 hours in CYCLES
// ((48*60*60)/SAMPLETIME_SECONDS)  // Two days in CYCLES

/*  setup()                 : Assign pins, (restore EEPROM settings)
 *  loop()                  :
 *    check_temperature()   : Check Temp and control HEATER
 *    respondToRequest(void): Check for serial input
 *      process(x,y,value)  : Respond to command "xy [value]"
 *
 *  int normalize(type, value) : Return (value-offset)*Scale for 'type' data
 *  dump(void)         : Dump configuration and sample data
 *  saveRestore(op)    : SAVE (RESTORE) configuration to (from) EEPROM
 *
 * Timeout (unit shutoff after 8 hours) related improvements planned:
 * 1) Add a reset (power?) switch to clear timeout and restart unit.
 * 2) Flash indicator light after timeout (e.g. isolate from heating control)
 */

#include "EEPROM.h"
#include "pid.h"
#define VERBOSE 1
#define SAMPLETIME (1000*SAMPLETIME_SECONDS)  // PID cycle in milliseconds

#define ONEMINUTE  60000 
#define HALFMINUTE  30000
#define OFFCYCLE_HEATER  (- HALFMINUTE/SAMPLETIME) // Forced off time in CYCLES

long int cutoff = 0;
int hotcycle = 2;          // Need values above zero to get started properly
double intemp = 21.0;      // Probably starting from room temperature
double outtemp = 21.0; 

PID temppid(&intemp, &outtemp, &settemp, 0.5, 0.5, 0.5, 0);

void printHelp()
{
	Serial.print("\n\n");
	Serial.println("d:  print all settings and stored data");
	Serial.println("h:  print this option menu");
	Serial.println("i:  print turbostat ID letter");
	Serial.println("ix: set turbostat ID to 'x'");
	Serial.println("The following commands without <value> will return the current value");
	Serial.println("ch <value> : set high temperature level");
	Serial.println("cl <value> : set low temperature level");
	Serial.println("cm <value> : set temperature delta");
	Serial.println("cs <value> : set temperature scaling");
	Serial.println("co <value> : set temperature offset");
}

/*
 * Sample, store, convert, and return analog inputs
 */
byte heaterstate = 'c';
bool override = false;
bool alldata = false;  // Dump entire array, or just non-zero data
byte id = 't'; // Temperature controller= unassigned, by default

int   scale[1]  = { 100 };
int   offset[1] = {   0 };
int   high[1]   = { 350 };
int   low[1]    = { 390 };
int   margin[1] = {  10 };

// Set global 'RomAddress' and address will be bumped
// by successive calls to moveData( SAVE|RESTORE, size, ptr)

int      SAVE   = 1;
int   RESTORE   = 0;
int RomAddress  = 0;

void moveData(int op, int size, byte *loc)
{
	for(int i=size;i>0;i--)
		if (op == SAVE)
			EEPROM.write(RomAddress++,*loc++);
		else
			*loc++ = EEPROM.read(RomAddress++);
}

byte identity(void)
{
	if (id == 't' && EEPROM.read(0) != 0)
	{
		id = EEPROM.read(0);
	}
	return id;
}


void saveRestore(int op)
{
	int i;
	RomAddress = 0;
	moveData(op, 1, &id);
	for(i=0;i<6;i++) moveData(op, sizeof(int), (byte *)(&high[i])  );
	for(i=0;i<6;i++) moveData(op, sizeof(int), (byte *)(&low[i])   );
	for(i=0;i<6;i++) moveData(op, sizeof(int), (byte *)(&margin[i]));
	for(i=0;i<6;i++) moveData(op, sizeof(int), (byte *)(&offset[i]));
}

// Set scale[] to 1.0 on first startup
// so we don't do the initial restore more than once.

int normalize(int i, int analog)
{ 
	float s = ( (float) scale[i] ) / 100.0;
	return( (int) ( s * (analog-offset[i]) ) ); // Offset, then scale
//	return( ((int)( s * analog )) - offset[i]); // Scale, then offset
}

int TEMPERATURE = 0;  // Analog pin numbers (and data indexes!)

int current(int needsame, int samples, int AnIn) // Noise reduction algorithm
{
float ctemp;
// float ftemp;
int same     = 0;
int start    = 0;
float avg;
int ts[10];
	for(int i=0; i< samples; i++) {
		ts[i] = analogRead(AnIn);
		delay(2);
	}
	while( same < needsame && start < (samples-needsame) )
	{
		same = 0;
		avg = 0.0;
		for(int j=start+1; j<10; j++) {
			if (abs(ts[start]-ts[j])<2) {
				same++;
				avg += (float)ts[j];
			}
		}
		start++;
	}
	if (same >= needsame) {
		ctemp = (((float)(avg)*100.0)/(((float)same)*200.0)); // Celcius
//		ftemp = 32.0 + (9.0*ctemp)/5.0;
		return round(ctemp);
	} else return current(needsame-1,samples,AnIn);
}

int HEATER      = 9;  // Digital pin numbers
int INDICATOR   = 10; 
int LED         = 13;
int flash;
/* The maximum variable space on Arduino is 2K.
 * Do not allocate space for much more than 200 samples
 * or the Arduino program will fail without warning.
 * 
 * 200 samples each of Temperature and Turbidity => 800 bytes
 *	1 sample pair every five minutes for 16.6 hours
 *	1 sample pair every half hour for 4.1 days
 *
 * We can easily double if the valid temp/turbidity values
 * fit into one byte (25% of the full 10-bit analog range)
 */

int numSamples = 30;
int  temperature[30];

int sampleIndex = 0;
static long int clock  = 0,       // Counts major loop iterations (~0.1 sec)
		control = 100,    // Update time for HEATER or VALVE
		recording = 10000; // 10000:10s, 300000:5 min, 1800000:30 min

// Print configuration settings, then all data as a time series
// Seconds   Temperature   Turbidity
//    0          334          680
//
void dump(void)
{
  int i;
  Serial.print("clock "); Serial.println(clock);
  Serial.print("ch "); Serial.println(high[TEMPERATURE]);
  Serial.print("cl "); Serial.println(low[TEMPERATURE]);
  Serial.print("cm "); Serial.println(margin[TEMPERATURE]);
  Serial.print("cs "); Serial.println(scale[TEMPERATURE]);
  Serial.print("co "); Serial.println(offset[TEMPERATURE]);
}

void check_temperature(void)
{
       int temp = current(7,10,TEMPERATURE);
       intemp = (double)temp;
       if (temppid.Compute()) { // True when we are at the next cycle
		if (cutoff > CUTOFF_TIMEOUT) {
			digitalWrite(HEATER,0);
			return;
		}
		cutoff = cutoff + 1;
#ifdef VERBOSE
	   	Serial.print("Compute succeeded: Temperature is ");
		Serial.println(intemp);
#endif
	       if (override) return;  // After PID controller updates values

	       if (hotcycle > OFFCYCLE_HEATER) { // Counting down the current heater-on cycle
			Serial.print(OFFCYCLE_HEATER);
			Serial.println(" OFFCYCLE_HEATER");
			Serial.print(hotcycle);
			Serial.println(" cycles to decrement");
		   	hotcycle--;
			if (hotcycle < 1) { // Heater off when we reach zero
#ifdef VERBOSE
			   	Serial.println("Heater off");
#endif
				digitalWrite(HEATER, 0);
				digitalWrite(INDICATOR, 0);
			}
			// But then stays off while hotcycle counts down to OFFCYCLE
		}

#ifdef VERBOSE
			Serial.print("in   " );
			Serial.print(intemp);
			Serial.print("    out ");
			Serial.print(outtemp);
			Serial.print("  cycles ");
			Serial.println(hotcycle);
#endif

		if (outtemp > intemp && settemp > intemp && hotcycle == OFFCYCLE_HEATER ) {
			hotcycle = 2 + 2*(int)( outtemp - intemp ); // Cycles, not Seconds.
//			hotcycle = outtemp
#ifdef VERBOSE
			Serial.print("Heater on for ");
			Serial.print(hotcycle);
			Serial.println(" cycles");
#endif
			digitalWrite(HEATER, 1);
			digitalWrite(INDICATOR, 1);
		}
	} // else we haven't reached the next time interval
}				



void respondToRequest(void)
{
	String is = "";
	while (Serial.available() > 0)  // Read a line of input
	{
		int c  = Serial.read();
		if ( c < 32 ) break;
		is += (char)c;
		if (Serial.available() == 0) // It is possible we're too fast
			delay(100);
	}
	if ( is.length() > 0 )  {   // process the command
		int value = 0;
		if (is.length() > 3)
			value = atoi(&is[3]);
		process(is[0], is[1], value);
	}
}

static int channel = TEMPERATURE;

void process(char c1, char c2, int value)
{
        long int tempint;
	float temp;
	switch(c1) {
		case 'c' :
			switch(c2)  {
				case 'c' : channel = value;  break;
				case 'h' : if (value == 0)
					           Serial.println(normalize(TEMPERATURE,high[TEMPERATURE]));
					   else {
						high[TEMPERATURE] = value;
						settemp = (double) value;
					   }
					   break;
				case 'l' : low[channel] = value;   break;
				case 'm' : margin[channel] = value;break;
				case 'o' : offset[channel] = value;break;
				case 's' : scale[channel] = value;break;
				default  : Serial.print("Ignoring [c");
					   Serial.write(c2);
					   Serial.println("]");
			}
			break;
		case 'd':
			dump();
			break;
		case 'h':
			if ( c2 == 0 ) {
				printHelp();
			} else switch(c2) {
			case 'h' :
				digitalWrite(HEATER, 1);
				digitalWrite(INDICATOR, 1);
				heaterstate = 'h';
				break;
			case 'c' :
				digitalWrite(HEATER, 0);
				digitalWrite(INDICATOR, 0);
				heaterstate = 'c';
				break;
			case 's' :
				Serial.write(heaterstate);
				Serial.println();
				break;
			default  : Serial.print("Ignoring [h");
				   Serial.write(c2);
				   Serial.println("]");
			     break;
			}
			break;
		case 'i':
			if ( c2 == 0 ) {
				Serial.write(id);
				Serial.println("");
			}
			else	       id = c2;
			break;
		case 'n' :
			override = false;
			break;
		case 'o' :
			override = true;
			break;
		case 'r' :
			saveRestore(RESTORE);
			break;
		case 's' :
			saveRestore(SAVE);
			
			break;
		default :
			Serial.print("Ignoring line begining with [");
			Serial.write(c1);
			if (!c2 == 0) Serial.write(c1);
			Serial.println("]");
	}
}

void toggle(void)
{
	if (flash == 0) flash = 1;
	else flash = 0;
	digitalWrite(INDICATOR, flash);
}

void setup() {        // setup() runs on power-up or after RESET
  Serial.begin(9600); // 9600 baud, 8-bits, no parity, one stop bit

  pinMode(HEATER, OUTPUT);     // up to 7KW heater
  pinMode(INDICATOR, OUTPUT);
  pinMode(LED, OUTPUT);     // Arduino Indicator light
  settemp = 49.0;           // 50 degrees C is the default target temperature
  flash = 0;

  if (EEPROM.read(0) != 0) {
  	saveRestore(RESTORE);
  }
  cutoff = 0;  // Run for 8 hours max (see CUTOFF_TIMEOUT setting)

// Initialize PID Controller
// Systems with high thermal inertia (=integration) can
// be controlled with a PD controller ( Ki == 0 ? )
// A little Ki can be added to eliminate long-term error
// Find optimal Kp with Ki = Kd = 0 (Result is constant amplitude oscillation)
// Then add Kd to dampen.
// Add Ki if there is long-term error.

//     outtemp = (double) current(7,10,TEMPERATURE);
     temppid = PID(&intemp, &outtemp, &settemp, 0.5, 0.05, 0.2, 0);
     temppid.SetSampleTime(SAMPLETIME);
     temppid.SetMode(AUTOMATIC);
     temppid.SetOutputLimits(10,70);
}

void loop() 
{
	delay(10);           // Delay between reads for stability
	respondToRequest();  // Handle queries from computer
	check_temperature();
	if (cutoff > CUTOFF_TIMEOUT) {
		delay(200);
		toggle();
	}
}




