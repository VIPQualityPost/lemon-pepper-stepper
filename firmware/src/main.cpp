#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/MT6835/MagneticSensorMT6835.h"
#include "encoders/stm32hwencoder/STM32HWEncoder.h"

#include "stm32g4xx_hal_conf.h"
#include "stm32g4xx_hal_fdcan.h"

#include "can.h"
#include "dfu.h"
#include "utils.h"
#include "lemon-pepper.h"

#define USBD_MANUFACTURER_STRING "matei repair lab"
#define USBD_PRODUCT_STRING_FS "lemon-pepper-stepper"

// board specific data
typedef struct
{
	uint16_t signature;
	Direction electricalDir;
	float electricalZero;
	uint16_t abzResolution;
	uint8_t encoderCalibrated;
	uint8_t canID;
} userData;

userData boardData;
uint8_t updateData = 0;

const uint16_t magicWord = 0xAF0C;

// canbus things
extern uint8_t TxData[8];
extern uint8_t RxData[8];

// simpleFOC things
#define POLEPAIRS 50
#define RPHASE 3
#define MOTORKV 40
#define ENC_PPR 16383 // max 16383 (zero index) -> *4 for CPR, -1 is done in init to prevent rollover on 16 bit timer

/**
 * SPI clockdiv of 16 gives ~10.5MHz clock. May still be stable with lower divisor.
 * The HW encoder is configured using PPR, which is then *4 for CPR (full 12384 gives overflow on 16 bit timer.)
 */
SPISettings myMT6835SPISettings(168000000 / 16, MT6835_BITORDER, SPI_MODE3);
MagneticSensorMT6835 sensor = MagneticSensorMT6835(ENC_CS, myMT6835SPISettings);
STM32HWEncoder enc = STM32HWEncoder(ENC_PPR, ENC_A, ENC_B, ENC_Z);

/**
 * The current sense amps have a gain of 90mA/V -> over 1.5A this is 135mA so we need gain of 24 to get full-scale.
 * Actually we are limited to powers of 2 for gain. So it should be 16. This gives sensitivity of 1440mV/A.
 * */
InlineCurrentSense currentsense = InlineCurrentSense(1440, ISENSE_U, ISENSE_V, ISENSE_W);

StepperDriver4PWM driver = StepperDriver4PWM(MOT_A1, MOT_A2, MOT_B1, MOT_B2);
StepperMotor motor = StepperMotor(POLEPAIRS);
Commander commander = Commander(SerialUSB);

uint16_t counter = 0;

// Prototypes
uint8_t configureFOC(void);
uint8_t configureCAN(void);
uint8_t calibrateEncoder(void);

void setup()
{
	pinMode(LED_GOOD, OUTPUT);
	pinMode(LED_FAULT, OUTPUT);
	pinMode(CAL_EN, OUTPUT);
	pinMode(MOT_EN, OUTPUT);

	SerialUSB.begin(115200);

	EEPROM.get(0, boardData);

	digitalWrite(MOT_EN, HIGH);
	digitalWrite(CAL_EN, LOW);

	if (!configureCAN())
		SIMPLEFOC_DEBUG("CAN init failed.");
	if (!configureFOC())
		SIMPLEFOC_DEBUG("FOC init failed.");

	if (sensor.getABZResolution() != ENC_PPR) // Check that PPR of the encoder matches our expectation.
	{
		digitalWrite(LED_FAULT, HIGH);
		SIMPLEFOC_DEBUG("Encoder ABZ resolution unexpected.");
	}

	// if (!boardData.encoderCalibrated) // If the encoder has not had self-calibration done, try.
	// {
	// 	uint8_t calibrationResult = calibrateEncoder();
	// 	if (calibrationResult == 0x3)
	// 	{
	// 		boardData.encoderCalibrated = 1;
	// 		updateData = 1;
	// 		SIMPLEFOC_DEBUG("Encoder self calibration successful.");
	// 	}
	// 	else
	// 	{
	// 		boardData.encoderCalibrated = 0;
	// 		digitalWrite(LED_FAULT, HIGH);
	// 		SIMPLEFOC_DEBUG("Encoder self calibration failed! Result: %#02x", calibrationResult);
	// 	}
	// }

	// if (boardData.canID == 0x000) // If the can ID is not set, then we'll look for a new, free ID.
	// {
	// 	uint8_t foundID = FDCAN_FindUniqueID();
	// 	if (foundID != 0)
	// 	{
	// 		boardData.canID = foundID;
	// 		updateData = 1;
	// 		SIMPLEFOC_DEBUG("Unique CAN ID found: %i", foundID);
	// 	} else {
	// 		digitalWrite(LED_FAULT, HIGH);
	// 		SIMPLEFOC_DEBUG("Failed to find a unique CAN ID!");
	// 	}
	// }

	// if(updateData) // If the configuration data has changed at all, update the flash.
	// {
	// 	EEPROM.put(0, boardData);
	// }
}

void loop()
{
	motor.loopFOC();
	motor.move();
	commander.run();

	if (counter == 0xFFF)
	{
		digitalToggle(LED_GOOD);
		SerialUSB.print(sensor.getAngle());
		SerialUSB.print("\t");
		SerialUSB.print(enc.getAngle());
		SerialUSB.print("\t");
		SerialUSB.println(sensor.getABZResolution());

		// SerialUSB.printf("%d\t%d\t%d\n", sensor.getAngle(), sensor.getABZResolution(), enc.getAngle());
		counter = 0;
	}

	counter++;

#ifdef HAS_MONITOR
	motor.monitor();
#endif
}

void doMotor(char *cmd)
{
	commander.motor(&motor, cmd);
}

uint8_t configureFOC(void)
{
	commander.add('M', doMotor, "motor");
	commander.verbose = VerboseMode::machine_readable;

#ifdef SIMPLEFOC_STM32_DEBUG
	SimpleFOCDebug::enable(&SerialUSB);
#endif

	// Encoder initialization.
	// Ideally configuring the sensor over SPI then use STM32HWEncoder
	enc.init();
	if (!enc.initialized)
		digitalWrite(LED_FAULT, HIGH);

	sensor.init();

	// Check if the encoder has loaded the right PPR, if not, update and then write to EEPROM.
	if (sensor.getABZResolution() != ENC_PPR)
	{
		delay(200);
		sensor.setABZResolution(ENC_PPR);
		sensor.writeEEPROM();

		digitalWrite(LED_GOOD, HIGH);
		digitalWrite(LED_FAULT, LOW);

		for (uint8_t i = 0; i < 60; i++)
		{ // Datasheet says we need to wait 6 seconds after writing EEPROM.
			digitalToggle(LED_GOOD);
			digitalToggle(LED_FAULT);
			delay(100);
		}

		digitalWrite(LED_GOOD, LOW);
		digitalWrite(LED_FAULT, LOW);
	}

	// Driver initialization.
	driver.pwm_frequency = 32000;
	driver.voltage_power_supply = 12;
	driver.voltage_limit = driver.voltage_power_supply / 2;
	driver.init();

	// Motor PID parameters.
	motor.PID_velocity.P = 5;
	motor.PID_velocity.I = 24;
	motor.PID_velocity.D = 0.01;
	motor.PID_velocity.output_ramp = 750;
	motor.PID_velocity.limit = 500;
	motor.LPF_velocity.Tf = 4;

	motor.P_angle.P = 600;
	motor.P_angle.limit = 10000;
	motor.LPF_angle.Tf = 0; // try to avoid

	// Motor initialization.
	// motor.voltage_sensor_align = 2;
	motor.current_limit = 1;
	motor.velocity_limit = 500;
	motor.controller = MotionControlType::velocity;
	motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

// Monitor initialization
#ifdef HAS_MONITOR
	motor.useMonitoring(SerialUSB);
	motor.monitor_start_char = 'M';
	motor.monitor_end_char = 'M';
	motor.monitor_downsample = 250;
#endif

	motor.linkSensor(&sensor);
	motor.linkDriver(&driver);

	currentsense.linkDriver(&driver);
	currentsense.init();

	motor.linkCurrentSense(&currentsense);

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
	//  updateData = 1;
	// }
	// else{
	// 	motor.zero_electric_angle = boardData.electricalZero;
	// 	motor.sensor_direction = boardData.electricalDir;
	// 	motor.init();
	// 	motor.initFOC();
	// }

	return 1;
}

uint8_t configureCAN(void)
{
	FDCAN_Start(0x000);
	return 1;
}

uint8_t calibrateEncoder(void)
{
	motor.target = 35; // roughly 2000rpm -> need to write 0x1 to Reg. AUTOCAL_FREQ

	MT6835Options4 currentSettings = sensor.getOptions4();
	currentSettings.autocal_freq = 0x1;
	sensor.setOptions4(currentSettings);

	uint16_t calTime = micros();
	while (calTime - micros() < 2000000)
	{
		motor.loopFOC();
		motor.move();

		if (calTime - micros() > 2000)
		{
			// after motor is spinning at constant speed, enable calibration.
			digitalWrite(LED_GOOD, HIGH);
			digitalWrite(CAL_EN, HIGH);
		}
	}

	digitalWrite(LED_GOOD, LOW);
	digitalWrite(CAL_EN, LOW);

	return sensor.getCalibrationStatus();
}
