/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	int16_t linacc_odr;
	float linacc_mG; // milli G per Bit
	int16_t linacc_aabw;
	int16_t gyro_odr;
	float gyro_mDPS; // milli degrees per second per Bit
} sIMUsettings;

typedef struct {
	float fTemperatureDegC; //°C
	float fAccX, fAccY, fAccZ; // m/s^2
	float fGyroXradps, fGyroYradps, fGyroZradps; // rad/s
} sIMUoutput;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define WITHACCGYRO
//#define WITHMINMAX

#define ADDR_TS_CAL1 ((uint16_t*) ((uint32_t) 0x1FFF7A2CUL)) /* 30 deg C calibration value @3.3V */
#define ADDR_TS_CAL2 ((uint16_t*) ((uint32_t) 0x1FFF7A2EUL)) /* 110 deg C calibration value @3.3V */

#define GRAVITY_EARTH					9.80665F	// m/s^2

#define LSM6DS33_I2C_ADDR				0x6A
#define LSM6DS33_WHOAMI					0x69
#define LSM6DS33_TIMEOUT				3 			// 3ms

// LSM6DS33 register map
#define LSM6DS33_FUNC_CFG_ACCESS_ADDR	0x01
#define LSM6DS33_FIFO_CTRL1_ADDR		0x06
#define LSM6DS33_FIFO_CTRL2_ADDR		0x07
#define LSM6DS33_FIFO_CTRL3_ADDR		0x08
#define LSM6DS33_FIFO_CTRL4_ADDR		0x09
#define LSM6DS33_FIFO_CTRL5_ADDR		0x0A
#define LSM6DS33_ORIENT_CFG_G_ADDR		0x0B
#define LSM6DS33_INT1_CTRL_ADDR			0x0D
#define LSM6DS33_INT2_CTRL_ADDR			0x0E
#define LSM6DS33_WHO_AM_I_ADDR			0x0F
#define LSM6DS33_CTRL1_XL_ADDR			0x10
#define LSM6DS33_CTRL2_G_ADDR			0x11
#define LSM6DS33_CTRL3_C_ADDR			0x12
#define LSM6DS33_CTRL4_C_ADDR			0x13
#define LSM6DS33_CTRL5_C_ADDR			0x14
#define LSM6DS33_CTRL6_C_ADDR			0x15
#define LSM6DS33_CTRL7_G_ADDR			0x16
#define LSM6DS33_CTRL8_XL_ADDR			0x17
#define LSM6DS33_CTRL9_XL_ADDR			0x18
#define LSM6DS33_CTRL10_C_ADDR			0x19
#define LSM6DS33_WAKE_UP_SRC_ADDR		0x1B
#define LSM6DS33_TAP_SRC_ADDR			0x1C
#define LSM6DS33_D6D_SRC_ADDR			0x1D
#define LSM6DS33_STATUS_REG_ADDR		0x1E
#define LSM6DS33_OUT_TEMP_L_ADDR		0x20
#define LSM6DS33_OUT_TEMP_H_ADDR		0x21
#define LSM6DS33_OUTX_L_G_ADDR			0x22
#define LSM6DS33_OUTX_H_G_ADDR			0x23
#define LSM6DS33_OUTY_L_G_ADDR			0x24
#define LSM6DS33_OUTY_H_G_ADDR			0x25
#define LSM6DS33_OUTZ_L_G_ADDR			0x26
#define LSM6DS33_OUTZ_H_G_ADDR			0x27
#define LSM6DS33_OUTX_L_XL_ADDR			0x28
#define LSM6DS33_OUTX_H_XL_ADDR			0x29
#define LSM6DS33_OUTY_L_XL_ADDR			0x2A
#define LSM6DS33_OUTY_H_XL_ADDR			0x2B
#define LSM6DS33_OUTZ_L_XL_ADDR			0x2C
#define LSM6DS33_OUTZ_H_XL_ADDR			0x2D
#define LSM6DS33_FIFO_STATUS1_ADDR		0x3A
#define LSM6DS33_FIFO_STATUS2_ADDR		0x3B
#define LSM6DS33_FIFO_STATUS3_ADDR		0x3C
#define LSM6DS33_FIFO_STATUS4_ADDR		0x3D
#define LSM6DS33_FIFO_DATA_OUT_L_ADDR	0x3E
#define LSM6DS33_FIFO_DATA_OUT_H_ADDR	0x3F
#define LSM6DS33_TIMESTAMP0_REG_ADDR	0x40
#define LSM6DS33_TIMESTAMP1_REG_ADDR	0x41
#define LSM6DS33_TIMESTAMP2_REG_ADDR	0x42
#define LSM6DS33_STEP_TIMESTAMP_L_ADDR	0x49
#define LSM6DS33_STEP_TIMESTAMP_H_ADDR	0x4A
#define LSM6DS33_STEP_COUNTER_L_ADDR	0x4B
#define LSM6DS33_STEP_COUNTER_H_ADDR	0x4C
#define LSM6DS33_FUNC_SRC_ADDR			0x53
#define LSM6DS33_TAP_CFG_ADDR			0x58
#define LSM6DS33_TAP_THS_6D_ADDR		0x59
#define LSM6DS33_INT_DUR2_ADDR			0x5A
#define LSM6DS33_WAKE_UP_THS_ADDR		0x5B
#define LSM6DS33_WAKE_UP_DUR_ADDR		0x5C
#define LSM6DS33_FREE_FALL_ADDR			0x5D
#define LSM6DS33_MD1_CFG_ADDR			0x5E
#define LSM6DS33_MD2_CFG_ADDR			0x5F

#define LSM6DS33_ODR_0Hz				0x00
#define LSM6DS33_ODR_12_5Hz				0x01
#define LSM6DS33_ODR_26Hz				0x02
#define LSM6DS33_ODR_52Hz				0x03
#define LSM6DS33_ODR_104Hz				0x04
#define LSM6DS33_ODR_208Hz				0x05
#define LSM6DS33_ODR_416Hz				0x06
#define LSM6DS33_ODR_833Hz				0x07
#define LSM6DS33_ODR_1_66kHz			0x08
#define LSM6DS33_ODR_3_33kHz			0x09
#define LSM6DS33_ODR_6_66kHz			0x0A

#define LSM6DS33_LA_FS_2G				0x00
#define LSM6DS33_LA_FS_16G				0x01
#define LSM6DS33_LA_FS_4G				0x02
#define LSM6DS33_LA_FS_8G				0x03

#define LSM6DS33_LA_AABW_400Hz			0x00
#define LSM6DS33_LA_AABW_200Hz			0x01
#define LSM6DS33_LA_AABW_100Hz			0x02
#define LSM6DS33_LA_AABW_50Hz			0x03

#define LSM6DS33_GY_FS_125dps			0x01
#define LSM6DS33_GY_FS_250dps			0x00
#define LSM6DS33_GY_FS_500dps			0x02
#define LSM6DS33_GY_FS_1000dps			0x04
#define LSM6DS33_GY_FS_2000dps			0x05
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
sIMUsettings IMUsettings = {-1, -1.0, -1, -1, -1.0};

sIMUoutput IMUoutput = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC3_Init(void);
static void MX_DAC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define CRLF "\r\n"

#define PRINTF_BUFFER_SIZE    128

void serialPrintf(const char * format, ...)
{
	va_list arglist;
	char tmp[PRINTF_BUFFER_SIZE + 1];
#ifdef WITHMINMAX
	snprintf(tmp, PRINTF_BUFFER_SIZE, "%.3f;", (float)(HAL_GetTick())/1000.0);
#else
	snprintf(tmp, PRINTF_BUFFER_SIZE, "%.0f;", (float)(HAL_GetTick())/1000.0);
#endif
	va_start(arglist, format);
	vsnprintf(tmp + strlen(tmp), PRINTF_BUFFER_SIZE - strlen(tmp), format, arglist);
	tmp[PRINTF_BUFFER_SIZE] = '\0';
	va_end(arglist);

	const char *t = tmp;
	HAL_UART_Transmit(&huart6, (uint8_t *) t, strlen(t), 10);
}

#define debugPrintf(...) do { serialPrintf(__VA_ARGS__); } while(0)
#define TRACE(f_, ...)        debugPrintf((f_ CRLF), ##__VA_ARGS__)

void TOUCH_AF_INT_Change(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = TOUCH_INT_Pin;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(TOUCH_INT_GPIO_Port, &GPIO_InitStructure);
}

bool I2C_LSM6DS33_ReadRegister(uint8_t reg, uint8_t * buf, uint8_t len)
{
	if (HAL_I2C_Master_Transmit(&hi2c2, LSM6DS33_I2C_ADDR << 1, &reg, 1, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: ReadRegister write reg. address failed");
		asm("bkpt 255");
		return false;
	}

	if (HAL_I2C_Master_Receive(&hi2c2, LSM6DS33_I2C_ADDR << 1, buf, len, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: ReadRegister read register failed");
		asm("bkpt 255");
		return false;
	}
	return true;
}

bool I2C_LSM6DS33_WriteRegister(uint8_t reg, uint8_t * buf, uint8_t len)
{
	uint8_t uAddrAndBuf[15];
	uAddrAndBuf[0] = reg;

	if (len > 0)
	{
		for (int i = 0;i < len;i++)
		{
			uAddrAndBuf[i + 1] = buf[i];
		}
	}

	if (HAL_I2C_Master_Transmit(&hi2c2, LSM6DS33_I2C_ADDR << 1, uAddrAndBuf, len + 1, 10000) != HAL_OK)
	{
		TRACE("I2C ERROR: WriteRegister failed");
		asm("bkpt 255");
		return false;
	}
	return true;
}

bool IMUgetConf(void)
{
	uint8_t ui8_regs[2] = {0};

	if (!I2C_LSM6DS33_ReadRegister(LSM6DS33_CTRL1_XL_ADDR, ui8_regs, 2))
	{
		TRACE("ERROR: LSM6DS33 did not respond to I2C address");
		return false;
	}

	switch ((ui8_regs[0] >> 4) & 0x0F) // CTRL1_XL
	{
		case LSM6DS33_ODR_0Hz:			IMUsettings.linacc_odr = 0; TRACE("LSM6DS33 lin. acc. output turned off"); break;
		case LSM6DS33_ODR_12_5Hz:		IMUsettings.linacc_odr = 13; break;
		case LSM6DS33_ODR_26Hz:			IMUsettings.linacc_odr = 26; break;
		case LSM6DS33_ODR_52Hz:			IMUsettings.linacc_odr = 52; break;
		case LSM6DS33_ODR_104Hz:		IMUsettings.linacc_odr = 104; break;
		case LSM6DS33_ODR_208Hz:		IMUsettings.linacc_odr = 208; break;
		case LSM6DS33_ODR_416Hz:		IMUsettings.linacc_odr = 417; break;
		case LSM6DS33_ODR_833Hz:		IMUsettings.linacc_odr = 833; break;
		case LSM6DS33_ODR_1_66kHz:		IMUsettings.linacc_odr = 1667; break;
		case LSM6DS33_ODR_3_33kHz:		IMUsettings.linacc_odr = 3333; break;
		case LSM6DS33_ODR_6_66kHz:		IMUsettings.linacc_odr = 6667; break;
		default: 						TRACE("ERROR: LSM6DS33 lin.acc. output data rate at illegal setting"); return false; break;
	}
	if (IMUsettings.linacc_odr > 0) 	TRACE("LSM6DS33 lin. acc. output at %i Hz", IMUsettings.linacc_odr);

	switch ((ui8_regs[0] >> 2) & 0x03)
	{
		case LSM6DS33_LA_FS_2G:			IMUsettings.linacc_mG = 0.061; break;
		case LSM6DS33_LA_FS_4G:			IMUsettings.linacc_mG = 0.122; break;
		case LSM6DS33_LA_FS_8G:			IMUsettings.linacc_mG = 0.244; break;
		case LSM6DS33_LA_FS_16G:		IMUsettings.linacc_mG = 0.488; break;
		default:						TRACE("ERROR: LSM6DS33 lin. acc full scale value at illegal setting"); return false; break;
	}
	TRACE("LSM6DS33 lin. acc. resolution %.2f G", IMUsettings.linacc_mG);

	switch ((ui8_regs[0] >> 2) & 0x03)
	{
		case LSM6DS33_LA_AABW_400Hz:	IMUsettings.linacc_aabw = 400; break;
		case LSM6DS33_LA_AABW_200Hz:	IMUsettings.linacc_aabw = 200; break;
		case LSM6DS33_LA_AABW_100Hz:	IMUsettings.linacc_aabw = 100; break;
		case LSM6DS33_LA_AABW_50Hz:		IMUsettings.linacc_aabw = 50; break;
		default:						TRACE("ERROR: LSM6DS33 lin. acc anti aliasing bandwidth at illegal setting"); return false; break;
	}
	TRACE("LSM6DS33 lin. acc. anti aliasing bandwidth at %i Hz", IMUsettings.linacc_aabw);

	switch ((ui8_regs[1] >> 4) & 0x0F) // CTRL2_G
	{
		case LSM6DS33_ODR_0Hz:			IMUsettings.gyro_odr = 0; TRACE("LSM6DS33 gyro output turned off"); break;
		case LSM6DS33_ODR_12_5Hz:		IMUsettings.gyro_odr = 13; break;
		case LSM6DS33_ODR_26Hz:			IMUsettings.gyro_odr = 26; break;
		case LSM6DS33_ODR_52Hz:			IMUsettings.gyro_odr = 52; break;
		case LSM6DS33_ODR_104Hz:		IMUsettings.gyro_odr = 104; break;
		case LSM6DS33_ODR_208Hz:		IMUsettings.gyro_odr = 208; break;
		case LSM6DS33_ODR_416Hz:		IMUsettings.gyro_odr = 417; break;
		case LSM6DS33_ODR_833Hz:		IMUsettings.gyro_odr = 833; break;
		case LSM6DS33_ODR_1_66kHz:		IMUsettings.gyro_odr = 1667; break;
		default: 						TRACE("ERROR: LSM6DS33 gyro output data rate at illegal setting"); return false; break;
	}
	if (IMUsettings.gyro_odr > 0) 	TRACE("LSM6DS33 gyro output at %i Hz", IMUsettings.gyro_odr);

	switch ((ui8_regs[1] >> 1) & 0x07)
	{
		case LSM6DS33_GY_FS_125dps:		IMUsettings.gyro_mDPS	= 4.375; break;
		case LSM6DS33_GY_FS_250dps:		IMUsettings.gyro_mDPS	= 8.75; break;
		case LSM6DS33_GY_FS_500dps:		IMUsettings.gyro_mDPS	= 17.5; break;
		case LSM6DS33_GY_FS_1000dps:	IMUsettings.gyro_mDPS	= 35.0; break;
		case LSM6DS33_GY_FS_2000dps:	IMUsettings.gyro_mDPS	= 70.0; break;
		default:						TRACE("ERROR: LSM6DS33 gyro full scale value at illegal setting"); return false; break;
	}
	TRACE("LSM6DS33 raw gyro resolution: %.2f milli-degrees/sec.", IMUsettings.gyro_mDPS);
	return true;
}

bool IMUinit(void)
{
	uint8_t ui8_reg[2] = {0};

	if (!I2C_LSM6DS33_ReadRegister(LSM6DS33_WHO_AM_I_ADDR, ui8_reg, 1))
	{
		TRACE("ERROR: LSM6DS33 did not respond to I2C address");
		return false;
	}
	if (ui8_reg[0] != LSM6DS33_WHOAMI)
	{
		TRACE("LSM6DS33 ERROR: Product ID read failes");
		return false;
	}
	TRACE("LSM6DS33 detected");

	// Set IMU configuration
	ui8_reg[0] = (LSM6DS33_ODR_104Hz << 4) | (LSM6DS33_LA_FS_4G << 2) | LSM6DS33_LA_AABW_100Hz; // Linear acceleration configuration
	ui8_reg[1] = (LSM6DS33_ODR_104Hz << 4) | (LSM6DS33_GY_FS_125dps < 1);						// Gyro configuration
	if (!I2C_LSM6DS33_WriteRegister(LSM6DS33_CTRL1_XL_ADDR, ui8_reg, 2))
	{
		TRACE("ERROR: Failed to write LSM6DS33 configuration");
		return false;
	}

	if (!IMUgetConf())
	{
		TRACE("ERROR: Failed ro read LSM6DS33 configuration");
		return false;
	}
	return true;
}

bool IMUread()
{
	uint8_t ui8_regs[14] = {0};

	int16_t i16_rawTemperature = 0;
	int16_t i16_rawGyroX = 0;
	int16_t i16_rawGyroY = 0;
	int16_t i16_rawGyroZ = 0;
	int16_t i16_rawAccX = 0;
	int16_t i16_rawAccY = 0;
	int16_t i16_rawAccZ = 0;

	// Check first if new data is available
	if (!I2C_LSM6DS33_ReadRegister(LSM6DS33_STATUS_REG_ADDR, ui8_regs, 1))
	{
		TRACE("ERROR: LSM6DS33 did not respond to I2C address");
		return false;
	}
	if ((ui8_regs[0] & 0x07) != 0x07)
	{
		TRACE("ERROR: LSM6DS33 has no new data available yet");
		return false;
	}

	if (!I2C_LSM6DS33_ReadRegister(LSM6DS33_OUT_TEMP_L_ADDR, ui8_regs, 14))
	{
		TRACE("ERROR: LSM6DS33 did not respond to I2C address");
		return false;
	}

	i16_rawTemperature = (ui8_regs[1] << 8) | ui8_regs[0];
	i16_rawGyroX = (ui8_regs[3] << 8) | ui8_regs[2];
	i16_rawGyroY = (ui8_regs[5] << 8) | ui8_regs[4];
	i16_rawGyroZ = (ui8_regs[7] << 8) | ui8_regs[6];
	i16_rawAccX = (ui8_regs[9] << 8) | ui8_regs[8];
	i16_rawAccY = (ui8_regs[11] << 8) | ui8_regs[10];
	i16_rawAccZ = (ui8_regs[13] << 8) | ui8_regs[12];

	IMUoutput.fGyroXradps = i16_rawGyroX * IMUsettings.gyro_mDPS * M_PI/180.0 / 1000.0;
	IMUoutput.fGyroYradps = i16_rawGyroY * IMUsettings.gyro_mDPS * M_PI/180.0 / 1000.0;
	IMUoutput.fGyroZradps = i16_rawGyroZ * IMUsettings.gyro_mDPS * M_PI/180.0 / 1000.0;

	IMUoutput.fAccX =  i16_rawAccX * IMUsettings.linacc_mG * GRAVITY_EARTH / 1000.0;
	IMUoutput.fAccY =  i16_rawAccY * IMUsettings.linacc_mG * GRAVITY_EARTH / 1000.0;
	IMUoutput.fAccZ =  i16_rawAccZ * IMUsettings.linacc_mG * GRAVITY_EARTH / 1000.0;

	IMUoutput.fTemperatureDegC = (float)i16_rawTemperature / 16.0 + 25.0;
	return true;
}

uint32_t ReadAnalogChannel(ADC_HandleTypeDef* hadc, uint32_t ui32Channel)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ui32Channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK)
	{
	    Error_Handler();
	}
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	return HAL_ADC_GetValue(hadc);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_DAC_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_SET); // Turn on power
  HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEDgreen_GPIO_Port, LEDgreen_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_RESET);

  // Turn on UART3 power
  HAL_GPIO_WritePin(UART3Pwr_GPIO_Port, UART3Pwr_Pin, GPIO_PIN_SET);

  // Turn on UART6 power
  HAL_GPIO_WritePin(UART6pwr_GPIO_Port, UART6pwr_Pin, GPIO_PIN_SET);

  HAL_Delay(100);

  if (!IMUinit())
  {
	  TRACE("ERROR: touchPanelInit() failed");
	  asm("bkpt 255");
  }

#ifdef WITHACCGYRO
#ifdef WITHMINMAX
  TRACE("STM32_T;T;AX;AY;AZ;GX;GY;GZ;LHm;LHc;LHM;LVm;LVc;LHM;RHm;RHc;RHM;RVm;RVc;RVM;S1m;S1c;S1M;VBatt");
#else
  TRACE("STM32_T;T;AX;AY;AZ;GX;GY;GZ;LH;LV;RH;RV;S1;VBatt");
#endif
#else
#ifdef WITHMINMAX
  TRACE("STM32_T;T;LHm;LHc;LHM;LVm;LVc;LHM;RHm;RHc;RHM;RVm;RVc;RVM;S1m;S1c;S1M;VBatt");
#else
  TRACE("STM32_T;T;LH;LV;RH;RV;S1;VBatt");
#endif
#endif
  HAL_Delay(1000);
#ifdef WITHMINMAX
  uint32_t ui32_StickLHmin = 4095;
  uint32_t ui32_StickLVmin = 4095;
  uint32_t ui32_StickRHmin = 4095;
  uint32_t ui32_StickRVmin = 4095;
  uint32_t ui32_Slider1min = 4095;
  uint32_t ui32_StickLHmax = 0;
  uint32_t ui32_StickLVmax = 0;
  uint32_t ui32_StickRHmax = 0;
  uint32_t ui32_StickRVmax = 0;
  uint32_t ui32_Slider1max = 0;

  uint8_t ui8_iterator = 0;
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (IMUread())
	  {
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		  uint32_t ui32_STM32Temperature = HAL_ADC_GetValue(&hadc1);
		  float fSTM32TempDegC = (80.0/(float)(*ADDR_TS_CAL2 - *ADDR_TS_CAL1))*(float)((float)ui32_STM32Temperature - *ADDR_TS_CAL1) + 30.0;

		  uint32_t ui32_StickLH = ReadAnalogChannel(&hadc3, ADC_CHANNEL_0);  // ADC3_IN0
		  uint32_t ui32_StickLV = ReadAnalogChannel(&hadc3, ADC_CHANNEL_1);  // ADC3_IN1
		  uint32_t ui32_StickRH = ReadAnalogChannel(&hadc3, ADC_CHANNEL_2);  // ADC3_IN2
		  uint32_t ui32_StickRV = ReadAnalogChannel(&hadc3, ADC_CHANNEL_3);  // ADC3_IN3
		  uint32_t ui32_Slider1 = ReadAnalogChannel(&hadc3, ADC_CHANNEL_4);  // ADC3_IN4
		  uint32_t ui32_VBatt   = ReadAnalogChannel(&hadc3, ADC_CHANNEL_5);  // ADC3_IN5

		  float f_VBatt = (float)(ui32_VBatt)*(3.3/4095.0) * ((120+39)/39);

		  //if (ui32_VBatt < 1950) // Low-voltage cut-off at 6.4V
		  if (f_VBatt < 6.4) // Low-voltage cut-off
		  {
			  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_RESET); // Turn off power
			  HAL_Delay(1000);
		  }

#ifdef WITHMINMAX
		  if (ui32_StickLH < ui32_StickLHmin) ui32_StickLHmin = ui32_StickLH;
		  if (ui32_StickLH > ui32_StickLHmax) ui32_StickLHmax = ui32_StickLH;

		  if (ui32_StickLV < ui32_StickLVmin) ui32_StickLVmin = ui32_StickLV;
		  if (ui32_StickLV > ui32_StickLVmax) ui32_StickLVmax = ui32_StickLV;

		  if (ui32_StickRH < ui32_StickRHmin) ui32_StickRHmin = ui32_StickRH;
		  if (ui32_StickRH > ui32_StickRHmax) ui32_StickRHmax = ui32_StickRH;

		  if (ui32_StickRV < ui32_StickRVmin) ui32_StickRVmin = ui32_StickRV;
		  if (ui32_StickRV > ui32_StickRVmax) ui32_StickRVmax = ui32_StickRV;

		  if (ui32_Slider1 < ui32_Slider1min) ui32_Slider1min = ui32_Slider1;
		  if (ui32_Slider1 > ui32_Slider1max) ui32_Slider1max = ui32_Slider1;

		  ui8_iterator++;
		  HAL_Delay(50);


		  if (ui8_iterator > 20)
		  {
			  ui8_iterator = 0;
#endif

#ifdef WITHACCGYRO
#ifdef WITHMINMAX
			  TRACE("%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%.1f",
		  		  		  fSTM32TempDegC, IMUoutput.fTemperatureDegC, IMUoutput.fAccX, IMUoutput.fAccY, IMUoutput.fAccZ, IMUoutput.fGyroXradps, IMUoutput.fGyroYradps, IMUoutput.fGyroZradps,
						  ui32_StickLHmin, ui32_StickLH, ui32_StickLHmax,
						  ui32_StickLVmin, ui32_StickLV, ui32_StickLVmax,
						  ui32_StickRHmin, ui32_StickRH, ui32_StickRHmax,
						  ui32_StickRVmin, ui32_StickRV, ui32_StickRVmax,
						  ui32_Slider1min, ui32_Slider1, ui32_Slider1max, f_VBatt);
#else
			  TRACE("%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%lu;%lu;%lu;%lu;%lu;%.1f",
		  		  		  fSTM32TempDegC, IMUoutput.fTemperatureDegC, IMUoutput.fAccX, IMUoutput.fAccY, IMUoutput.fAccZ, IMUoutput.fGyroXradps, IMUoutput.fGyroYradps, IMUoutput.fGyroZradps,
						  ui32_StickLH, ui32_StickLV, ui32_StickRH, ui32_StickRV, ui32_Slider1, f_VBatt);
#endif
#else
#ifdef WITHMINMAX
			  TRACE("%.2f;%.2f;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%lu;%.1f",
		  		  		  fSTM32TempDegC, IMUoutput.fTemperatureDegC,
						  ui32_StickLHmin, ui32_StickLH, ui32_StickLHmax,
						  ui32_StickLVmin, ui32_StickLV, ui32_StickLVmax,
						  ui32_StickRHmin, ui32_StickRH, ui32_StickRHmax,
						  ui32_StickRVmin, ui32_StickRV, ui32_StickRVmax,
						  ui32_Slider1min, ui32_Slider1, ui32_Slider1max, f_VBatt);
#else
			  TRACE("%.2f;%.2f;%lu;%lu;%lu;%lu;%lu;%.1f",
		  		  		  fSTM32TempDegC, IMUoutput.fTemperatureDegC,
						  ui32_StickLH, ui32_StickLV, ui32_StickRH, ui32_StickRV, ui32_Slider1, f_VBatt);
#endif
#endif
#ifdef WITHMINMAX
		  }
#else
		  HAL_Delay(10000); // Measurement every 10 sec.
#endif
	  }
	  else
		  TRACE("Failed reading IMU");

	  // Check Power-Off
	  if (HAL_GPIO_ReadPin(PWRswitch_GPIO_Port, PWRswitch_Pin) == GPIO_PIN_RESET)
	  {
		  //HAL_GPIO_WritePin(LEDred_GPIO_Port, LEDred_Pin, GPIO_PIN_SET);
		  //HAL_GPIO_WritePin(LEDgreen_GPIO_Port, LEDgreen_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_SET);

		  // Check again in 1 second
		  HAL_Delay(1000);
		  if (HAL_GPIO_ReadPin(PWRswitch_GPIO_Port, PWRswitch_Pin) == GPIO_PIN_RESET)
		  {
			  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_RESET); // Turn off power
		  }
	  }
	  HAL_GPIO_WritePin(LEDblue_GPIO_Port, LEDblue_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 50;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 2;
  hltdc.Init.VerticalSync = 10;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 12;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 284;
  hltdc.Init.TotalWidth = 525;
  hltdc.Init.TotalHeigh = 286;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 479;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 271;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 261120;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 479;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 271;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg1.Alpha = 255;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 261120;
  pLayerCfg1.ImageWidth = 480;
  pLayerCfg1.ImageHeight = 272;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 16;
  SdramTiming.ExitSelfRefreshDelay = 16;
  SdramTiming.SelfRefreshTime = 16;
  SdramTiming.RowCycleDelay = 16;
  SdramTiming.WriteRecoveryTime = 16;
  SdramTiming.RPDelay = 16;
  SdramTiming.RCDDelay = 16;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEDred_Pin|LEDgreen_Pin|LEDblue_Pin|HAPTIC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INTMODboot_GPIO_Port, INTMODboot_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCDnRST_GPIO_Port, LCDnRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TOUCH_RST_GPIO_Port, TOUCH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TESTPOINT_Pin|IntModPwr_Pin|UART3Pwr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AudioMute_GPIO_Port, AudioMute_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UART6pwr_Pin|LCDbacklight_Pin|ExtModPwr_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWRon_GPIO_Port, PWRon_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TrainerOut_GPIO_Port, TrainerOut_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TelemDir_GPIO_Port, TelemDir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BluetoothEn_GPIO_Port, BluetoothEn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEDred_Pin LEDgreen_Pin LEDblue_Pin HAPTIC_Pin */
  GPIO_InitStruct.Pin = LEDred_Pin|LEDgreen_Pin|LEDblue_Pin|HAPTIC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SWEL_Pin */
  GPIO_InitStruct.Pin = SWEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SWEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYenter_Pin KEYpageprevious_Pin SWAL_Pin KEYrtn_Pin
                           KEYtelem_Pin KEYmdl_Pin KEYsys_Pin */
  GPIO_InitStruct.Pin = KEYenter_Pin|KEYpageprevious_Pin|SWAL_Pin|KEYrtn_Pin
                          |KEYtelem_Pin|KEYmdl_Pin|KEYsys_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : KEYpagenext_Pin TrimLHR_Pin */
  GPIO_InitStruct.Pin = KEYpagenext_Pin|TrimLHR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INTMODboot_Pin LCDnRST_Pin */
  GPIO_InitStruct.Pin = INTMODboot_Pin|LCDnRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_RST_Pin */
  GPIO_InitStruct.Pin = TOUCH_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOUCH_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TOUCH_INT_Pin PCBREV1_Pin PCBREV2_Pin ROTENCB_Pin
                           ROTENCA_Pin */
  GPIO_InitStruct.Pin = TOUCH_INT_Pin|PCBREV1_Pin|PCBREV2_Pin|ROTENCB_Pin
                          |ROTENCA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : SWF_Pin SWEH_Pin SWAH_Pin SWBH_Pin
                           SWI_Pin SWJ_Pin */
  GPIO_InitStruct.Pin = SWF_Pin|SWEH_Pin|SWAH_Pin|SWBH_Pin
                          |SWI_Pin|SWJ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : TESTPOINT_Pin AudioMute_Pin IntModPwr_Pin UART3Pwr_Pin */
  GPIO_InitStruct.Pin = TESTPOINT_Pin|AudioMute_Pin|IntModPwr_Pin|UART3Pwr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TrimLHL_Pin */
  GPIO_InitStruct.Pin = TrimLHL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TrimLHL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SDpresent_Pin TrainerIn_Pin */
  GPIO_InitStruct.Pin = SDpresent_Pin|TrainerIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : UART6pwr_Pin LCDbacklight_Pin ExtModPwr_Pin */
  GPIO_InitStruct.Pin = UART6pwr_Pin|LCDbacklight_Pin|ExtModPwr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWRswitch_Pin */
  GPIO_InitStruct.Pin = PWRswitch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWRswitch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWRon_Pin */
  GPIO_InitStruct.Pin = PWRon_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWRon_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWBL_Pin TrainerDetect_Pin */
  GPIO_InitStruct.Pin = SWBL_Pin|TrainerDetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TrimRSD_Pin TrimRSU_Pin SWCL_Pin */
  GPIO_InitStruct.Pin = TrimRSD_Pin|TrimRSU_Pin|SWCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SWCH_Pin TrimLSU_Pin TrimR_Pin TrimRHR_Pin */
  GPIO_InitStruct.Pin = SWCH_Pin|TrimLSU_Pin|TrimR_Pin|TrimRHR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : HEARTBEAT_Pin */
  GPIO_InitStruct.Pin = HEARTBEAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEARTBEAT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SWDH_Pin TrimLSD_Pin TrimRV_Pin TrimRVD_Pin
                           TrimLVU_Pin */
  GPIO_InitStruct.Pin = SWDH_Pin|TrimLSD_Pin|TrimRV_Pin|TrimRVD_Pin
                          |TrimLVU_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : SWDL_Pin SWGL_Pin SWGH_Pin SWH_Pin
                           TrimLVD_Pin */
  GPIO_InitStruct.Pin = SWDL_Pin|SWGL_Pin|SWGH_Pin|SWH_Pin
                          |TrimLVD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : TrainerOut_Pin */
  GPIO_InitStruct.Pin = TrainerOut_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TrainerOut_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ExtModTX_Pin */
  GPIO_InitStruct.Pin = ExtModTX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ExtModTX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TelemDir_Pin */
  GPIO_InitStruct.Pin = TelemDir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TelemDir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BluetoothEn_Pin */
  GPIO_InitStruct.Pin = BluetoothEn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BluetoothEn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USBchaCtrl_Pin USBchgDetect_Pin */
  GPIO_InitStruct.Pin = USBchaCtrl_Pin|USBchgDetect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
