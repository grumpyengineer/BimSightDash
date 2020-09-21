#define VERSION			0.1a
#define CALS_VERSION	1

#undef DEBUG
#undef NOBATT
#undef SLEEP

HardwareSerial & batsci = Serial1;
HardwareSerial & metsci = Serial2;
HardwareSerial & debug = Serial3;

#define BATSCIDE	6
#define BATSCIRE	5

#define METSCIDE	12
#define METSCIRE	11

#define MAMODE1IN	39
#define MAMODE1OUT	22

#define MAMODE2IN	37
#define MAMODE2OUT	23

#define CMDPWRIN	38
#define CMDPWROUT	14

#define FAN	4

#define ENABLE5V	24
#define GPIO1	25
#define GPIO2	26
#define GPIO3	27
#define GPIO4	28
#define GPIO5	29
#define GPIO6	30
#define GPIO7	31
#define GPIO8	32

#define CURRENT	A22
#define VREF	A1

#define BOOTING		0
#define IDLE		1
#define RUNNING		2
#define	SLEEP		3
#define BALANCING	4
#define CHARGING	5
#define OFFSETCAL	6
#define FAULT		255

#define MINVOLTAGE		140.0
#define IDLEIMEOUT		5000
#define SLEEPTIMEOUT	30000

typedef struct {
	int currentState;
	int newState;
	int lastStateMillis;
} bcmstate_t;

typedef struct {
	int valid;
	int byteCount;
	byte data[3];
	byte b3;
	byte b4;
	unsigned long lastByte;
} metscistate_t;

typedef struct {
	int cells;
	float packVoltage;
	float minCellVoltage;
	float maxCellVoltage;
	float averageCellTemp;
	float minCellTemp;
	float maxCellTemp;
	float packCurrent;
	float maxChargeCurrent;
	float maxDischargeCurrent;
	int soc;
	int canAssist;
	int canRegen;
	int rewriteSoc;
	int overrideSoc;
	int mcmSoc;
	float capacity;
} batterystate_t;

typedef struct {
	int version;
	int lastSoc;
	float currentOffset;
} cals_t;

#define METSCIINTERPACKET	50
#define METSCITIMEOUT		200

#define VREFFILTER			5.0
#define CURRENTFILTER		30.0
#define MCMCURRENTFILTER	30.0
#define BCMCURRENTFILTER	5.0
#define OFFSETCOUNT			1000
#define ADCDEADZONE			0.5
#define CURRENTDEADZONE		0.2
#define ADCSHIFT			3

#define SLEEPTIMEOUT		30000
