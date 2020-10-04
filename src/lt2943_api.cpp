#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <path/to/i2c_driver.h>

#define LTC2943_I2C_ADDRESS 0x64

// LTC2943 MAP
#define LTC2943_STATUS_REG                          0x00
#define LTC2943_CONTROL_REG                         0x01
#define LTC2943_ACCUM_CHARGE_MSB_REG                0x02
#define LTC2943_ACCUM_CHARGE_LSB_REG                0x03
#define LTC2943_CHARGE_THRESH_HIGH_MSB_REG          0x04
#define LTC2943_CHARGE_THRESH_HIGH_LSB_REG          0x05
#define LTC2943_CHARGE_THRESH_LOW_MSB_REG           0x06
#define LTC2943_CHARGE_THRESH_LOW_LSB_REG           0x07
#define LTC2943_VOLTAGE_MSB_REG                     0x08
#define LTC2943_VOLTAGE_LSB_REG                     0x09
#define LTC2943_VOLTAGE_THRESH_HIGH_MSB_REG         0x0A
#define LTC2943_VOLTAGE_THRESH_HIGH_LSB_REG         0x0B
#define LTC2943_VOLTAGE_THRESH_LOW_MSB_REG          0x0C
#define LTC2943_VOLTAGE_THRESH_LOW_LSB_REG          0x0D
#define LTC2943_CURRENT_MSB_REG                     0x0E
#define LTC2943_CURRENT_LSB_REG                     0x0F
#define LTC2943_CURRENT_THRESH_HIGH_MSB_REG         0x10
#define LTC2943_CURRENT_THRESH_HIGH_LSB_REG         0x11
#define LTC2943_CURRENT_THRESH_LOW_MSB_REG          0x12
#define LTC2943_CURRENT_THRESH_LOW_LSB_REG          0x13
#define LTC2943_TEMPERATURE_MSB_REG                 0x14
#define LTC2943_TEMPERATURE_LSB_REG                 0x15
#define LTC2943_TEMPERATURE_THRESH_HIGH_REG         0x16
#define LTC2943_TEMPERATURE_THRESH_LOW_REG          0x17

// CMD codes
#define LTC2943_AUTOMATIC_MODE                  0xC0
#define LTC2943_SCAN_MODE                       0x80
#define LTC2943_MANUAL_MODE                     0x40
#define LTC2943_SLEEP_MODE                      0x00

#define LTC2943_PRESCALAR_M_1                   0x00
#define LTC2943_PRESCALAR_M_4                   0x08
#define LTC2943_PRESCALAR_M_16                  0x10
#define LTC2943_PRESCALAR_M_64                  0x18
#define LTC2943_PRESCALAR_M_256                 0x20
#define LTC2943_PRESCALAR_M_1024                0x28
#define LTC2943_PRESCALAR_M_4096                0x30
#define LTC2943_PRESCALAR_M_4096_2              0x31

#define LTC2943_ALERT_MODE                      0x04
#define LTC2943_CHARGE_COMPLETE_MODE            0x02

#define LTC2943_DISABLE_ALCC_PIN                0x00
#define LTC2943_SHUTDOWN_MODE                   0x01

#define LTC2943_TEMP_ALERT			0x10
#define LTC2943_VOL_ALERT			0x02

bool i2cInit(void);
bool i2cDeinit(void);
bool i2cIsInitialized(void);
bool i2cRead(uint8_t address, uint8_t *pDst, uint16_t dataSize); // Read data from I2C device to pDst buffer
bool i2cWrite(uint8_t address, const uint8_t *pSrc, uint16_t dataSize); // Write data from pSrc buffer to I2C device

enum adc_modes {
	AUTO_MODE,
	SCAN_MODE,
	MAN_MODE,
	SLEEP_MODE,
	MODE_CNT
};

struct lt2943 {
	int adc_mode;
	int temp_alert;
	int volt_alert;
};

/*
   1.       Change chip's ADC mode into one of these modes: automatic mode, scan mode, manual mode, sleep.

   2.       Get chip's ADC mode.

   3.       Check if temperature alert is pending.

   4.       Check if voltage alert is pending.
*/

class lt2943_api {
	private:
		const int chip_adc_modes[] = {
			[AUTO_MODE]  = LTC2943_AUTOMATIC_MODE,
			[SCAN_MODE]  = LTC2943_SCAN_MODE,
			[MAN_MODE]   = LTC2943_MANUAL_MODE,
			[SLEEP_MODE] = LTC2943_SLEEP_MODE,
		};

		int handle_adc_mode(uint8_t mode)
	       	{
			for (int i = AUTO_MODE; i < MODE_CNT; i++) {
				if (mode & chip_adc_modes[i]) 
					return i;	
			}

			return -1;
		}

		int handle_temp_alert(uint8_t status)
	       	{
			if (status & LTC2943_TEMP_ALERT) {
				return 1;
			}

			return 0;
		}

		int handle_vol_alert(uint8_t status)
	       	{
			if (status & LTC2943_VOL_ALERT) {
				return 1;
			}

			return 0;
		}

	public:
		struct lt2943 lt2943;

		int get_adc_mode()
		{
			int ret = -1;
			uint8_t i2c_cache;

			if (!i2cIsInitialized() ||
			    !i2cInit()) {
				fprintf(stderr, "Failed to initialize api\n");
				goto end;
			}

			i2c_cache = LTC2943_CONTROL_REG; 
			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
				goto end;
			}

			i2c_cache = LTC2943_CONTROL_REG; 
			if (!i2cRead(LTC2943_I2C_ADDRESS,
				     &i2c_cache,
				     8)) {
				fprintf(stderr, "Failed to read from %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
				goto end;
			}

			if ((lt2943.adc_mode = handle_adc_mode(i2c_cache)) < 0) {
				fprintf(stderr, "Failed to handle adc mode\n");
				goto end;
			}

			ret = 0;
		end:
			if (i2cIsInitialized() &&
		 	    i2cDeinit()) {
				fprintf(stderr, "Failed to deinitialize i2c ctx\n");
			}

			return ret;
		}

		int get_vol_alert_pend()
	       	{
			int ret = -1;	
			uint8_t i2c_cache;

			if (!i2cIsInitialized() ||
			    !i2cInit()) {
				fprintf(stderr, "Failed to initialize api\n");
				goto end;
			}

			i2c_cache = LTC2943_STATUS_REG; 
			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_STATUS_REG);
				goto end;
			}

			i2c_cache = LTC2943_STATUS_REG; 
			if (!i2cRead(LTC2943_I2C_ADDRESS,
				     &i2c_cache,
				     8)) {
				fprintf(stderr, "Failed to read from %d LTC2943 address\n",
					LTC2943_STATUS_REG);
			}

			lt2943.vol_alert = handle_vol_alert(i2c_cache);

			ret = lt2943.vol_alert;
		end:
			if (i2cIsInitialized() &&
		 	    i2cDeinit()) {
				fprintf(stderr, "Failed to deinitialize i2c ctx\n");
			}

			return ret;
		}

		int get_temp_alert_pend()
	       	{
			int ret = -1;	
			uint8_t i2c_cache;

			if (!i2cIsInitialized() ||
			    !i2cInit()) {
				fprintf(stderr, "Failed to initialize api\n");
				goto end;
			}

			i2c_cache = LTC2943_STATUS_REG; 
			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_STATUS_REG);
				goto end;
			}

			i2c_cache = LTC2943_STATUS_REG; 
			if (!i2cRead(LTC2943_I2C_ADDRESS,
				     &i2c_cache,
				     8)) {
				fprintf(stderr, "Failed to read from %d LTC2943 address\n",
					LTC2943_STATUS_REG);
			}

			lt2943.temp_alert  = handle_temp_alert(i2c_cache);

			ret = lt2943.temp_alert;
		end:
			if (i2cIsInitialized() &&
		 	    i2cDeinit()) {
				fprintf(stderr, "Failed to deinitialize i2c ctx\n");
			}

			return ret;
		}

		int set_adc_mode(enum adc_modes mode)
		{
			int ret = -1;
			uint8_t i2c_cache = LTC2943_CONTROL_REG;

			if (!i2cIsInitialized() ||
			    !i2cInit()) {
				fprintf(stderr, "Failed to initialize api\n");
				goto end;
			}

			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
			}

			i2c_cache = chip_adc_modes[mode];
			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
			}

			ret = 0;
		end:
			if (i2cIsInitialized() &&
		 	    i2cDeinit()) {
				fprintf(stderr, "Failed to deinitialize i2c ctx\n");
			}

			return ret;
		};

		lt2943_api()
		{
			int ret = -1;
			uint8_t i2c_cache;

			if (!i2cIsInitialized() ||
			    !i2cInit()) {
				fprintf(stderr, "Failed to initialize api\n");
				goto end;
			}

			if (get_adc_mode()) {
				fprintf(stderr, "Failed to get adc mode\n");	
				goto end;
			}

			i2c_cache = LTC2943_CONTROL_REG; 
			if (!i2cWrite(LTC2943_I2C_ADDRESS,
				      &i2c_cache,
				      8)) {
				fprintf(stderr, "Failed to write to %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
				goto end;
			}

			i2c_cache = LTC2943_CONTROL_REG; 
			if (!i2cRead(LTC2943_I2C_ADDRESS,
				     &i2c_cache,
				     8)) {
				fprintf(stderr, "Failed to read from %d LTC2943 address\n",
					LTC2943_CONTROL_REG);
				goto end;
			}

			lt2943.volt_alert  = handle_temp_alert(i2c_cache);
			lt2943.temp_alert  = handle_temp_alert(i2c_cache);

			ret = 0;
		end:
			if (i2cIsInitialized() &&
		 	    i2cDeinit()) {
				fprintf(stderr, "Failed to deinitialize i2c ctx\n");
			}

			return ret;
		};
};
