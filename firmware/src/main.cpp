#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6835/MagneticSensorMT6835.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

// #include "can.h"
#include "dfu.h"
#include "lemon-pepper.h"

#define USBD_MANUFACTURER_STRING     	"matei repair lab"
#define USBD_PRODUCT_STRING_FS     		"lemon-pepper-stepper"

// board specific data
typedef struct
{
	uint16_t signature;
	int8_t electricalDir;
	float electricalZero;
	uint8_t canID;
}userData;

userData boardData;
const uint16_t magicWord = 0xAF0C;

// canbus things
extern uint8_t TxData[8];
extern uint8_t RxData[8];

// simpleFOC things
#define POLEPAIRS 50
#define RPHASE 3
#define MOTORKV 200
#define ENC_PPR 0xFFFD // 65533 -> 65534 ppr (65535 cause overflow on 16 bit timer)

SPIClass spi1(ENC_COPI, ENC_CIPO, ENC_SCK);
SPISettings myMT6835SPISettings(168000000/16, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
STM32HWEncoder enc = STM32HWEncoder(ENC_PPR, ENC_A, ENC_B, ENC_Z);

StepperDriver4PWM driver = StepperDriver4PWM(MOT_A1, MOT_A2, MOT_B1, MOT_B2);
StepperMotor motor = StepperMotor(POLEPAIRS, RPHASE, MOTORKV);
Commander commander = Commander(SerialUSB);

uint16_t counter = 0;

// Prototypes
void configureFOC(void);
// void configureCAN(void);
// void configureEncoder(void);

void setup()
{
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_FAULT, OUTPUT);
	pinMode(CAL_EN, OUTPUT);
	pinMode(MOT_EN, OUTPUT);

	SerialUSB.begin(115200);

	// EEPROM.get(0, boardData);

	digitalWrite(MOT_EN, HIGH);
	digitalWrite(CAL_EN, LOW);

	// configureCAN();
	// configureEncoder();
	// configureFOC();
	sensor.init(&spi1);

	// if(boardData.canID == 0x000)
	// {
	// 	// If the can ID is not initialized, then we'll look for a free ID.
	// 	boardData.canID = FDCAN_FindUniqueID();
	// 	SerialUSB.println(boardData.canID);
	// }

	// if(boardData.signature != magicWord)
	// {
	// 	// If the EEPROM has not been initalized yet, save all the known data.
	// 	EEPROM.put(0, boardData);
	// }
}

void loop()
{	
	// motor.loopFOC();
	// motor.move();
	// commander.run();
	sensor.update();
	delay(10);
	SerialUSB.printf("%#06x\n", sensor.readRawAngle21());

	#ifdef HAS_MONITOR
	motor.monitor();
	#endif
}

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

void configureFOC(void){
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;
	
	#ifdef SIMPLEFOC_STM32_DEBUG
	SimpleFOCDebug::enable(&SerialUSB);
	#endif

	// Encoder initialization.
	// Ideally configuring the sensor over SPI then use STM32HWEncoder
	// sensor.init(&spi1);
	// sensor.setABZResolution(ENC_PPR);

	// enc.init();

	// Driver initialization.
	driver.pwm_frequency = 32000;
	driver.voltage_power_supply = 9;
	driver.voltage_limit = driver.voltage_power_supply/2;
	driver.init();

	// Motor PID parameters.
	motor.PID_velocity.P = 0.2;
	motor.PID_velocity.I = 3;
	motor.PID_velocity.D = 0.002;
	motor.PID_velocity.output_ramp = 100;
	motor.LPF_velocity.Tf = 0.5;
	motor.LPF_angle.Tf = 0; // try to avoid

	// Motor initialization.
	motor.voltage_sensor_align = 2;
	motor.current_limit = 0.35;
	motor.velocity_limit = 50;
	motor.controller = MotionControlType::velocity_openloop;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

	// Monitor initialization
	#ifdef HAS_MONITOR
	motor.useMonitoring(SerialUSB);
	motor.monitor_start_char = 'M';
	motor.monitor_end_char = 'M';
	motor.monitor_downsample = 250;
	#endif

	motor.linkSensor(&enc);
	motor.linkDriver(&driver);

	motor.target = 10;

	motor.zero_electric_angle = NOT_SET;
	motor.sensor_direction = Direction::UNKNOWN;

	motor.init();
	motor.initFOC();

	// if(boardData.signature != magicWord){
	// 	// If we have not initialized the EEPROM before.
	// 	motor.init();
	// 	motor.initFOC();

	// 	boardData.signature = magicWord;
	// 	boardData.electricalZero = motor.zero_electric_angle;
	// 	boardData.electricalDir = motor.sensor_direction;
	// }
	// else{
	// 	motor.zero_electric_angle = boardData.electricalZero;
	// 	motor.sensor_direction = boardData.electricalDir;
	// 	motor.init();
	// 	motor.initFOC();
	// }
}

// void configureCAN(void){
// 	FDCAN_Start(0x000);
// }


