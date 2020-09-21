/*
  Copyright (c) 2020 Grumpy Engineering
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:
  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include "BimSightBCM.h"
#include "BMSModuleManager.h"
#include <Arduino.h>
#include "CRC8.h"
#include "config.h"
#include <EEPROM.h>
#include <FlexCAN.h> //https://github.com/collin80/FlexCAN_Library
#include <ADC.h> //https://github.com/pedvide/ADC
#include <Filters.h> //https://github.com/JonHub/Filters

bcmstate_t bcmState;
metscistate_t metsciState;
batterystate_t batteryState;
cals_t cals;

IntervalTimer batsciTimer;
IntervalTimer consoleTimer;
IntervalTimer currentTimer;

BMSModuleManager bms;

CAN_message_t msg;
CAN_message_t inMsg;
CAN_filter_t filter;

char msgString[128];                        // Array to store serial string
uint8_t Imod, mescycle = 0;
uint8_t nextmes = 0;
uint16_t commandrate = 50;
uint8_t testcycle = 0;
uint8_t DMC[8] = {0, 0, 0, 0, 0, 0, 0, 0};
byte Unassigned, NextID = 0;
unsigned long balancetimer;
bool balancepauze = 0;
int balancecells;

//BMW checksum variable///
CRC8 crc8;
//uint8_t checksum;
const uint8_t finalxor [12] = {0xCF, 0xF5, 0xBB, 0x81, 0x27, 0x1D, 0x53, 0x69, 0x02, 0x38, 0x76, 0x4C};

unsigned long looptime = 0;

// current measurement
ADC *adc = new ADC(); // adc object

FilterOnePole vrefFilter( LOWPASS, VREFFILTER );
FilterOnePole currentFilter( LOWPASS, CURRENTFILTER );
FilterOnePole mcmCurrentFilter( LOWPASS, MCMCURRENTFILTER );
FilterOnePole bcmCurrentFilter( LOWPASS, BCMCURRENTFILTER );

void getCurrent()
{
	int value;
	float rawVoltage, rawVref, rawCurrent;
	static int count = 0;
	static float offset = 0.0;

	// CAN and current sensor power control
	switch(bcmState.currentState) {
		case BOOTING:
		case IDLE:
		case RUNNING:
		case CHARGING:
		case BALANCING:
		case OFFSETCAL: // Power on CAN and current sensor
			// get VREF
			value = (uint16_t)adc->adc0->analogReadContinuous() >> ADCSHIFT;
			rawVref = float((value * 3300.0) / (adc->adc0->getMaxValue() >> ADCSHIFT));
			vrefFilter.input(rawVref);

	//Serial.print("Raw VREF ");
	//Serial.print(rawVref);
	//Serial.print('\t');

			// get CURRENT
			value = (uint16_t)adc->adc1->analogReadContinuous() >> ADCSHIFT;
			rawVoltage = float((value * 3300.0) / (adc->adc1->getMaxValue() >> ADCSHIFT));
			currentFilter.input(rawVoltage);

	//Serial.print("Raw CurrentV ");
	//Serial.print(rawVoltage);
	//Serial.print('\t');

			rawVoltage = currentFilter.output() - vrefFilter.output() - cals.currentOffset;
			//rawVoltage = rawVoltage - rawVref - cals.currentOffset;

			if((bcmState.currentState == OFFSETCAL) && (cals.currentOffset == 0.0))
			{
				if(count == OFFSETCOUNT)
				{
					cals.currentOffset = offset / OFFSETCOUNT;
					count = 0;
					offset = 0.0;
				}
				else
				{
					offset += rawVoltage;
					count ++;
				}
			}

	//Serial.print("Offset CurrentV ");
	//Serial.print(rawVoltage);
	//Serial.print('\t');

			rawCurrent = rawVoltage * (200.0/1250.0);

			//if(abs(rawCurrent < ADCDEADZONE))
			//	rawCurrent = 0.0;

	//Serial.print("Current ");
	//Serial.print(rawCurrent);
	//Serial.print('\t');

			mcmCurrentFilter.input(rawCurrent);
			bcmCurrentFilter.input(rawCurrent);

	//Serial.print("Current ");
	//Serial.println(bcmCurrentFilter.output());

			break;
		default: // Do nothing
			break;
	}
}

byte insightCrc(byte *data, int len) {
	int i;
	byte crc = 0;

    for (i = 0; i < len; i++) {
        crc += data[i];
    }

    crc = ~crc;
    crc += 1;

    return (crc&0x7f);
}

void sendBatsci() {
	static int batsciPacket = 0;
	byte batsciData[12];
	byte voltage;
	byte batt1;
	byte batt2;
	byte battFlags;
	short current;

	// Get the current
	if((bcmState.currentState == IDLE) || (bcmState.currentState == RUNNING) || (bcmState.currentState == CHARGING))
	{
		batteryState.packCurrent = mcmCurrentFilter.output();
		//if(abs(batteryState.packCurrent < CURRENTDEADZONE))
		//	batteryState.packCurrent = 0.0;
	}
	else
	{
		batteryState.packCurrent = 0.0;
		return; // bodge, this probably won't work!
	}


	// don't send if metsci is not valid
	if(metsciState.valid == 1) {

		// quick bodge
		if(batteryState.overrideSoc == 0)
		{
			if(batteryState.packVoltage < 155.0)
				batteryState.canAssist = 0;
			if(batteryState.packVoltage > 195.0)
				batteryState.canRegen = 0;

			if((batteryState.canAssist == 0) && (batteryState.packVoltage > 170.0))
				batteryState.canAssist = 1;

			if((batteryState.canRegen == 0) && (batteryState.packVoltage < 180.0))
				batteryState.canRegen = 1;

		}

		// No regen. no assist
		if((batteryState.canAssist == 0) && (batteryState.canRegen == 0)) {
			batt1 = 0x12;
			batt2 = 0x32;
			battFlags = 0x30;
		}
		// Regen only
		if((batteryState.canAssist == 0) && (batteryState.canRegen == 1)) {
			batt1 = 0x11;
			batt2 = 0x6a;
			battFlags = 0x10;
		}
		// Assist only
		if((batteryState.canAssist == 1) && (batteryState.canRegen == 0)) {
			batt1 = 0x16;
			batt2 = 0x3c;
			battFlags = 0x20;
		}
		// Full assist and regen
		if((batteryState.canAssist == 1) && (batteryState.canRegen == 1)) {
			//batt1 = 0x15;
			//batt2 = 0x6f;

			batt1 = highByte(batteryState.mcmSoc << 1) & 0x7f;
			batt2 = lowByte(batteryState.mcmSoc) & 0x7f;

			battFlags = 0x00;
		}

		// calculate batsci voltage byte
		voltage = (byte)(batteryState.packVoltage / 2);

		// calculate batsci current value but don't & or shift it here
		current = (short)(2048.0 + (batteryState.packCurrent * 20.48));

		if(batsciPacket == 0) {
			batsciData[0] = 0x87;
			batsciData[1] = 0x40;
			batsciData[2] = voltage;
			batsciData[3] = batt1;
			batsciData[4] = batt2;
			batsciData[5] = highByte(current << 1) & 0x7f;
			batsciData[6] = lowByte(current) & 0x7f;
			batsciData[7] = 0x32;
			batsciData[8] = 0x39;
			batsciData[9] = 0x39;
			batsciData[10] = metsciState.b3;

			batsciPacket = 1;
		}
		else {
			batsciData[0] = 0xaa;
			batsciData[1] = 0x10;
			batsciData[2] = 0x00;
			batsciData[3] = 0x00;
			batsciData[4] = 0x00;
			batsciData[5] = battFlags;
			batsciData[6] = 0x40;
			batsciData[7] = 0x61;
			batsciData[8] = highByte(current << 1) & 0x7f;
			batsciData[9] = lowByte(current) & 0x7f;
			batsciData[10] = metsciState.b4;

			batsciPacket = 0;
		}
		batsciData[11] = insightCrc(batsciData, 11);
		batsci.write(batsciData, 12);
	}
}

void processMetsci(void) {
	// Got a 3 byte packet
	metsciState.byteCount = 0;

	if(metsciState.valid == 0) {
		metsciState.valid = 1;
		// Enable the transmitters
		digitalWrite(BATSCIDE, HIGH);
		digitalWrite(METSCIDE, HIGH);
	}

	// Save b3 byte
	if(metsciState.data[0] == 0xb3) {
		metsciState.b3 = metsciState.data[2];
	}
	// save b4 byte
	if(metsciState.data[0] == 0xb4) {
		metsciState.b4 = metsciState.data[2];
	}

	if(batteryState.rewriteSoc == 1)
	{
		// update packet with our SOC and regen CRC
		if(metsciState.data[0] == 0xe1) {
			metsciState.data[1] = 0x20 + batteryState.soc;
			metsciState.data[2] = insightCrc(metsciState.data, 2);
		}
	}
	metsci.write(metsciState.data, 3);
}

void canread()
{
  	Can0.read(inMsg);
  // Read data: len = data length, buf = data byte(s)
 // if (inMsg.id == 0x3C2)
  //{
    //CAB300();
  //}

	//ID not assigned//
	if (inMsg.id == 0xF0)
	{
		Unassigned++;
		Serial.print(millis());
		if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
			sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
		else
			sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

		Serial.print(msgString);

		if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
			sprintf(msgString, " REMOTE REQUEST FRAME");
			Serial.print(msgString);
		} else {
			for (byte i = 0; i < inMsg.len; i++) {
				sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
				DMC[i] = inMsg.buf[i];
				Serial.print(msgString);
			}
		}

		Serial.println();
		for (byte i = 0; i < 8; i++)
		{
		Serial.print(DMC[i], HEX);
		Serial.print("|");
		}
		Serial.println();
  	}

	if (inMsg.id > 0x99 && inMsg.id < 0x180)//do BMS magic if ids are ones identified to be modules
	{
		bms.decodecan(inMsg, 0); //do  BMS if ids are ones identified to be modules
	}
	if ((inMsg.id & 0xFF0) == 0x180)    // Determine if ID is standard (11 bits) or extended (29 bits)
	{
		bms.decodetemp(inMsg, 0 , 0);
	}

#ifdef DEBUG
	Serial.print(millis());
	if ((inMsg.id & 0x80000000) == 0x80000000)    // Determine if ID is standard (11 bits) or extended (29 bits)
    	sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (inMsg.id & 0x1FFFFFFF), inMsg.len);
    else
    	sprintf(msgString, ",0x%.3lX,false,%1d", inMsg.id, inMsg.len);

	Serial.print(msgString);

	if ((inMsg.id & 0x40000000) == 0x40000000) {  // Determine if message is a remote request frame.
		sprintf(msgString, " REMOTE REQUEST FRAME");
		Serial.print(msgString);
    } else {
    	for (byte i = 0; i < inMsg.len; i++) {
        	sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
    		Serial.print(msgString);
        }
    }
    Serial.println();
#endif
}


void balancing()
{
  	//Function to control balancing command, to be found
}

void sendcommand() //Send Can Command to get data from slaves
{
	///////module id cycling/////////

	if (nextmes == 6)
	{
		bms.getAllVoltTemp();

		mescycle ++;
		nextmes = 0;
		if (testcycle < 4)
		{
			testcycle++;
		}

		if (mescycle == 0xF)
		{
			mescycle = 0;

			if (balancetimer < millis())
			{
				balancepauze = 1;
				if (debug == 1)
				{
				Serial.println();
				Serial.println("Reset Balance Timer");
				Serial.println();
				}
				balancetimer = millis() + ((BALANCEDUTY + 60) * 1000);
			}
			else
			{
				balancepauze = 0;
			}
		}
	}

	if (balancepauze == 1)
	{
		balancecells = 0;
	}


	msg.id  = 0x080 | (nextmes);
	msg.len = 8;
	if (balancecells == 1)
	{
		msg.buf[0] = lowByte((uint16_t((bms.getLowCellVolt()) * 1000) + 5));
		msg.buf[1] = highByte((uint16_t((bms.getLowCellVolt()) * 1000) + 5));
	}
	else
	{
		msg.buf[0] = 0xc7;
		msg.buf[1] = 0x10;
	}
	msg.buf[2] = 0x00; //balancing bits
	msg.buf[3] = 0x00; //balancing bits

	if (testcycle < 3)
	{
		msg.buf[4] = 0x20;
		msg.buf[5] = 0x00;
	}
	else
	{

		if (balancecells == 1)
		{
		msg.buf[4] = 0x48;
		}
		else
		{
		msg.buf[4] = 0x40;
		}
		msg.buf[5] = 0x01;
	}

	msg.buf[6] = mescycle << 4;
	if (testcycle == 2)
	{
		msg.buf[6] = msg.buf[6] + 0x04;
	}

	msg.buf[7] = getcheck(msg, nextmes);

	//delay(2);
	Can0.write(msg);
	nextmes ++;

	if (bms.checkstatus() == true)
	{
		Serial.println("Reset Bal");
		resetbalancedebug();
	}
}

uint8_t getcheck(CAN_message_t &msg, int id)
{
	unsigned char canmes [11];
	int meslen = msg.len + 1; //remove one for crc and add two for id bytes
	canmes [1] = msg.id;
	canmes [0] = msg.id >> 8;

	for (int i = 0; i < (msg.len - 1); i++)
	{
		canmes[i + 2] = msg.buf[i];
	}
	/*
		Serial.println();
		for (int i = 0; i <  meslen; i++)
		{
		Serial.print(canmes[i], HEX);
		Serial.print("|");
		}
	*/
	return (crc8.get_crc8(canmes, meslen, finalxor[id]));
}

void resetbalancedebug()
{
	msg.id  =  0x0B0; //broadcast to all Elteks
	msg.len = 8;
	msg.ext = 0;
	msg.buf[0] = 0xFF;
	msg.buf[1] = 0x00;
	msg.buf[2] = 0xCD;
	msg.buf[3] = 0xA2;
	msg.buf[4] = 0x00;
	msg.buf[5] = 0x00;
	msg.buf[6] = 0x00;
	msg.buf[7] = 0x00;

	Can0.write(msg);
}

void resetIDdebug()
{
	//Rest all possible Ids
	for (int ID = 0; ID < 15; ID++)
	{
		msg.id  =  0x0A0; //broadcast to all CSC
		msg.len = 8;
		msg.ext = 0;
		msg.buf[0] = 0xA1;
		msg.buf[1] = ID;
		msg.buf[2] = 0xFF;
		msg.buf[3] = 0xFF;
		msg.buf[4] = 0xFF;
		msg.buf[5] = 0xFF;
		msg.buf[6] = 0xFF;
		msg.buf[7] = 0xFF;

		Can0.write(msg);

		delay(2);
	}
	//NextID = 0;

	//check for found unassigned CSC
	Unassigned = 0;

	msg.id  =  0x0A0; //broadcast to all CSC
	msg.len = 8;
	msg.ext = 0;
	msg.buf[0] = 0x37;
	msg.buf[1] = 0xFF;
	msg.buf[2] = 0xFF;
	msg.buf[3] = 0xFF;
	msg.buf[4] = 0xFF;
	msg.buf[5] = 0xFF;
	msg.buf[6] = 0xFF;
	msg.buf[7] = 0xFF;

	Can0.write(msg);
}

void findUnassigned ()
{
	Unassigned = 0;
	//check for found unassigned CSC
	msg.id  =  0x0A0; //broadcast to all CSC
	msg.len = 8;
	msg.ext = 0;
	msg.buf[0] = 0x37;
	msg.buf[1] = 0xFF;
	msg.buf[2] = 0xFF;
	msg.buf[3] = 0xFF;
	msg.buf[4] = 0xFF;
	msg.buf[5] = 0xFF;
	msg.buf[6] = 0xFF;
	msg.buf[7] = 0xFF;

	Can0.write(msg);
}

void assignID()
{
	msg.id  =  0x0A0; //broadcast to all CSC
	msg.len = 8;
	msg.ext = 0;
	msg.buf[0] = 0x12;
	msg.buf[1] = 0xAB;
	msg.buf[2] = DMC[0];
	msg.buf[3] = DMC[1];
	msg.buf[4] = DMC[2];
	msg.buf[5] = DMC[3];
	msg.buf[6] = 0xFF;
	msg.buf[7] = 0xFF;

	Can0.write(msg);

	delay(30);

	msg.buf[1] = 0xBA;
	msg.buf[2] = DMC[4];
	msg.buf[3] = DMC[5];
	msg.buf[4] = DMC[6];
	msg.buf[5] = DMC[7];

	Can0.write(msg);

	delay(10);
	msg.buf[0] = 0x5B;
	msg.buf[1] = NextID;
	Can0.write(msg);

	delay(10);
	msg.buf[0] = 0x37;
	msg.buf[1] = NextID;
	Can0.write(msg);

	NextID++;

	findUnassigned();
}

void showConsole(void) {
	// Only show console if we are running the CAN bus
	//if((bcmState.currentState != IDLE) && (bcmState.currentState != RUNNING))
	//	return;

	Serial.println("BimSightBCM Status");
	Serial.print("BCM State: ");
	Serial.print(bcmState.currentState);
	Serial.print('\t');
	Serial.print("Can Assist: ");
	Serial.print(batteryState.canAssist);
	Serial.print('\t');
	Serial.print("Can Regen: ");
	Serial.print(batteryState.canRegen);
	Serial.print('\t');
	Serial.print("MCM SOC: ");
	Serial.print(highByte(batteryState.mcmSoc << 1) & 0x7f, HEX);
	Serial.print(" ");
	Serial.println(lowByte(batteryState.mcmSoc) & 0x7f, HEX);

	Serial.print("Battery Cells: ");
	Serial.print(batteryState.cells);
	Serial.print('\t');
	Serial.print("Pack Voltage: ");
	Serial.print(batteryState.packVoltage);
	Serial.print('\t');
	Serial.print("Lowest Voltage: ");
	Serial.print(batteryState.minCellVoltage);
	Serial.print('\t');
	Serial.print("Highest Voltage: ");
	Serial.println(batteryState.maxCellVoltage);

	Serial.print("Lowest Temp ");
	Serial.print(batteryState.minCellTemp);
	Serial.print('\t');
	Serial.print("Highest Temp: ");
	Serial.print(batteryState.maxCellTemp);
	Serial.print('\t');
	Serial.print("Peak Charge: ");
	Serial.print(batteryState.maxChargeCurrent);
	Serial.print('\t');
	Serial.print("Peak Discharge: ");
	Serial.println(batteryState.maxDischargeCurrent);

	Serial.print("Pack Current: ");
	Serial.println(batteryState.packCurrent);
	Serial.print('\t');
	Serial.print("SOC: ");
	Serial.print(batteryState.soc);
	Serial.print('\t');
	Serial.print("Capacity: ");
	Serial.println(batteryState.capacity);

	bms.printPackDetails(3, 0);
}

void restoreCals()
{
	Serial.println("Restore cals");
	cals.version = CALS_VERSION;
	cals.lastSoc = 0;
	cals.currentOffset = 0.0;
	EEPROM.put(0, cals); // Save cals
}

void setup() {
	// initialise states
	memset(&bcmState,0,sizeof bcmState);
	memset(&metsciState,0,sizeof metsciState);
	memset(&batteryState,0,sizeof batteryState);

  	EEPROM.get(0, cals); // get cals
  	if (cals.version != CALS_VERSION)
  	{
    	restoreCals();
  	}

	// Start in the booting state
	bcmState.currentState = BOOTING;
	bcmState.newState = BOOTING;
	bcmState.lastStateMillis = millis();

	// bodge!!
	batteryState.canAssist = 1;
	batteryState.canRegen = 1;
	batteryState.rewriteSoc = 1;
	batteryState.overrideSoc = 0;
	batteryState.mcmSoc = 0xAEF;

  	// USB serial console
  	Serial.begin(115200);

  	// BATSCI and METSCI to the ECM
  	batsci.begin(9600,SERIAL_8E1);
  	metsci.begin(9600,SERIAL_8E1);

  	// Serial debug console
  	//debug.begin(115200);

	// Setup RS422 driver for BATSCI, transmitter disabled, receiver disabled because it is not connected
	pinMode(BATSCIRE, OUTPUT);
  	digitalWrite(BATSCIRE, HIGH);
	pinMode(BATSCIDE, OUTPUT);
  	digitalWrite(BATSCIDE, LOW);

	// Setup RS422 driver for METSCI, transmitter disabled, receiver enabled
	pinMode(METSCIRE, OUTPUT);
  	digitalWrite(METSCIRE, LOW);
	pinMode(METSCIDE, OUTPUT);
  	digitalWrite(METSCIDE, LOW);

  	// Force fan off for now
	pinMode(FAN, OUTPUT);
  	digitalWrite(FAN, HIGH);

	// General IO
	pinMode(ENABLE5V, OUTPUT);
  	digitalWrite(ENABLE5V, HIGH);

	// ADC
	adc->adc0->setAveraging(32); // set number of averages
	adc->adc0->setResolution(16); // set bits of resolution
	adc->adc0->setReference(ADC_REFERENCE::REF_3V3);
	adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
	adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
	adc->adc0->startContinuous(VREF);

	adc->adc1->setAveraging(32); // set number of averages
	adc->adc1->setResolution(16); // set bits of resolution
	adc->adc1->setReference(ADC_REFERENCE::REF_3V3);
	adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
	adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
	adc->adc1->startContinuous(CURRENT);

	Can0.begin(500000);
	crc8.begin();
	bms.setPstrings(1);
	bms.setSensors(0, 2.0, 0);

	//set filters for standard
	for (int i = 0; i < 8; i++)
	{
		Can0.getFilter(filter, i);
		filter.flags.extended = 0;
		Can0.setFilter(filter, i);
	}
	//set filters for extended
	for (int i = 9; i < 13; i++)
	{
		Can0.getFilter(filter, i);
		filter.flags.extended = 1;
		Can0.setFilter(filter, i);
	}

	//delay(10000);  //just for easy debugging. It takes a few seconds for USB to come up properly on most OS's

	looptime = millis();
	Serial.println("BimSightBCM v0.0.1a-ish");

	Serial.print("Offset Calibration value: ");
	Serial.println(cals.currentOffset);

  	// Display reason the Teensy was last reset
  	Serial.println();
  	Serial.println("Reason for last Reset: ");

  	if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("Stop Mode Acknowledge Error Reset");
  	if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("MDM-AP Reset");
  	if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("Software Reset");                   // reboot with SCB_AIRCR = 0x05FA0004
  	if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("Core Lockup Event Reset");
  	if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("Power-on Reset");                   // removed / applied power
  	if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("External Pin Reset");               // Reboot with software download
  	if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("Watchdog(COP) Reset");              // WDT timed out
  	if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("Loss of External Clock Reset");
  	if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("Loss of Lock in PLL Reset");
  	if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("Low-voltage Detect Reset");
  	Serial.println();

	// Get analogue current measurement
	currentTimer.begin(getCurrent, 5000);

	// Send BATSCI every 30ms
	batsciTimer.begin(sendBatsci, 30000);

	// Debug console
	consoleTimer.begin(showConsole, 2000000);

    bms.setBalIgnore(false); // bodge

}

void loop() {
	int data;
	static unsigned long commandtime = 0;

	while (Can0.available()) {
		canread();
	}

	// data on METSCI
	if (metsci.available()) {
		metsciState.lastByte = millis();
    	data = metsci.read();

		// Check when the last byte arrived, if its been too long assume this is a new packet
		if((millis() - metsciState.lastByte) > METSCIINTERPACKET)
			metsciState.byteCount = 0;
		// Save the METSCI data byte
		metsciState.data[metsciState.byteCount] = data;
		// If we have 3 bytes then process the packet
		if(metsciState.byteCount == 2)
			processMetsci();
		else
			metsciState.byteCount++;
  	}

	// has METSCI gone quiet?
	if(metsciState.valid == 1) {
		if((millis() - metsciState.lastByte) > METSCITIMEOUT)	{
			Serial.println("### METSCI Lost");
			// Mark METSCI as invalid
			metsciState.byteCount = 0;
			metsciState.valid = 0;
			// Flush the serial ports
			metsci.flush();
			batsci.flush();
			// Disable the transmitters
			digitalWrite(BATSCIDE, LOW);
			digitalWrite(METSCIDE, LOW);
		}
	}

	// Some debuggy stuff
	if (Serial.available())
	{
    	data = Serial.read();

		if(data == 's')
		{
			if(batteryState.soc < 20)
				batteryState.soc++;
			Serial.print("SOC ");
			Serial.println(batteryState.soc);
		}
		if(data == 'S')
		{
			if(batteryState.soc != 0)
				batteryState.soc--;
			Serial.print("SOC ");
			Serial.println(batteryState.soc);
		}

		if(data == 'c')
		{
			cals.currentOffset = 0.0;
			bcmState.newState = OFFSETCAL;
		}

		if(data == 'z')
		{
			batteryState.maxChargeCurrent = 0.0;
			batteryState.maxDischargeCurrent = 0.0;
		}

		if(data == 'a')
		{
			batteryState.canAssist = 1;
		}
		if(data == 'A')
		{
			batteryState.canAssist = 0;
		}

		if(data == 'r')
		{
			batteryState.canRegen = 1;
		}
		if(data == 'R')
		{
			batteryState.canRegen = 0;
		}

		if(data == 'w')
		{
			batteryState.rewriteSoc = 1;
		}
		if(data == 'W')
		{
			batteryState.rewriteSoc = 0;
		}

		if(data == 'm')
		{
			batteryState.mcmSoc++;
		}
		if(data == 'M')
		{
			batteryState.mcmSoc--;
		}
		if(data == 'n')
		{
			batteryState.mcmSoc+=0x80;
		}
		if(data == 'N')
		{
			batteryState.mcmSoc-=0x80;
		}

		if(data == 'o')
		{
			batteryState.overrideSoc = 1;
		}
		if(data == 'O')
		{
			batteryState.overrideSoc = 0;
		}
	}

	if((bcmState.currentState == IDLE) || (bcmState.currentState == RUNNING) || (bcmState.currentState == CHARGING))
	{
#ifdef NOBATT
		batteryState.cells = 48;
		batteryState.packVoltage = 170.0;
		batteryState.minCellVoltage = 3.542;
		batteryState.maxCellVoltage = 3.542;
		batteryState.averageCellTemp = 20.0;
		batteryState.minCellTemp = 20.0;
		batteryState.maxCellTemp = 20.0;
		if(batteryState.packCurrent > batteryState.maxChargeCurrent)
			batteryState.maxChargeCurrent = batteryState.packCurrent;
		if(batteryState.packCurrent < batteryState.maxDischargeCurrent)
			batteryState.maxDischargeCurrent = batteryState.packCurrent;
#else
		batteryState.cells = bms.seriescells();
		batteryState.packVoltage = bms.getPackVoltage();
		batteryState.minCellVoltage = bms.getLowCellVolt();
		batteryState.maxCellVoltage = bms.getHighCellVolt();
		batteryState.averageCellTemp = bms.getAvgTemperature(); // have to run this before you can get high and low temps!
		batteryState.minCellTemp = bms.getLowTemperature();
		batteryState.maxCellTemp = bms.getHighTemperature();
		if(batteryState.packCurrent > batteryState.maxChargeCurrent)
			batteryState.maxChargeCurrent = batteryState.packCurrent;
		if(batteryState.packCurrent < batteryState.maxDischargeCurrent)
			batteryState.maxDischargeCurrent = batteryState.packCurrent;
#endif
	}
	else
	{
		// reset mac currents
		batteryState.maxChargeCurrent = 0.0;
		batteryState.maxDischargeCurrent = 0.0;
	}

	// State machine
	switch(bcmState.currentState)
	{
		case BOOTING: // Booting, find all the cells and then go to idle
#ifdef NOBATT
			bcmState.newState = IDLE;
#else
			if((bms.seriescells() == 48) && (bms.getPackVoltage() > MINVOLTAGE))
			{
				Serial.print("### Found all cells after ");
				Serial.println(millis() - looptime);
				bcmState.newState = IDLE;
			}
#endif
			break;
		case IDLE: // Waiting for something to happen or to go to sleep
			if(metsciState.valid == 1)
			{
				Serial.println("### METSCI LIVE - RUNNING");
				bcmState.newState = RUNNING;
			}
			else
			{
#ifdef SLEEP
				if((millis() - bcmState.lastStateMillis) > SLEEPTIMEOUT)
				{
					bcmState.newState = SLEEP;
				}
#endif
			}
			break;
		case RUNNING: // Car is running
			if(metsciState.valid == 0) {
				if((millis() - metsciState.lastByte) > IDLEIMEOUT)
				{
					Serial.println("### METSCI GONE - IDLE");
					bcmState.newState = IDLE;
				}
			}
			break;
		case SLEEP: // Gone to sleep!
			// wake on ignition
			// wake on METSCI
			// wake on CAN
				bcmState.newState = IDLE;
			break;
		case BALANCING:
			break;
		case CHARGING:
			break;
		case OFFSETCAL:
			if(cals.currentOffset != 0.0)
			{
				Serial.print("Offset Calibration done: ");
				Serial.println(cals.currentOffset);
				EEPROM.put(0, cals); // Save cals
				bcmState.newState = IDLE;
			}
			break;
		case FAULT:
			Serial.println("### Oh fuck!");
			break;
		default:
			break;
	}

	// CAN messaging
	switch(bcmState.currentState) {
		case BOOTING:
		case IDLE:
		case RUNNING:
		case BALANCING:
		case CHARGING:
		case OFFSETCAL: // Send CAN commands to modules
			if (millis() - commandtime > COMMANDRATE) {
				commandtime = millis();
				sendcommand();
			}
			break;
		default: // Power off CAN and current sensor
			break;
	}

	if(bcmState.currentState != bcmState.newState)
	{
		// record time of state change
		bcmState.lastStateMillis = millis();

		// CAN and current sensor power control
		switch(bcmState.newState) {
			case BOOTING:
			case IDLE:
			case RUNNING:
			case BALANCING:
			case CHARGING:
			case OFFSETCAL: // Power on CAN and current sensor
				digitalWrite(ENABLE5V, HIGH);
				break;
			default: // Power off CAN and current sensor
				digitalWrite(ENABLE5V, LOW);
				break;
		}
		bcmState.currentState = bcmState.newState;
	}

}