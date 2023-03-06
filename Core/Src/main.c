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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash_if.h"
#include "command.h"

#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//CRC module for integrity checking
CRC_HandleTypeDef hcrc;
extern ApplicationTypeDef Appli_state;

//FATFS variables
FATFS myUsbFatFS;
//USB logical path
extern char USBHPath[4];   /* USBH logical drive path */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_USB_HOST_Process(void);
void MX_CRC_Init(void);
void HAL_CRC_MspInit(CRC_HandleTypeDef*);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef*);

/* USER CODE BEGIN PFP */
static void bootloader_jump_to_user_app(void);
static uint32_t calcCrc32(uint8_t*, uint32_t);
uint8_t usb_test_write(void);
uint8_t usb_test_read(void);

static void flash_status_led(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t app_is_valid = 0;
volatile uint8_t usb_stick_connected = 0;
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
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */
  MX_CRC_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t usb_start_ms = HAL_GetTick();
  uint32_t usb_current_ms = 0;
  uint32_t startup_timeout = 1500;
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process(); //handles status update of usb, if this is not running, usb never changes

    /* USER CODE BEGIN 3 */
    switch(Appli_state) //handle the status that is updated in above HOST_Process()
    {
    case APPLICATION_IDLE:
    	usb_current_ms = HAL_GetTick();
    	if((usb_current_ms - usb_start_ms) > startup_timeout)
    	{
    		uint32_t valCrc32 = calcCrc32((uint8_t*)APPLICATION_ADDRESS, (0x1F4000));
//			if (valCrc32 == 0) //forced pass for debugging
    		if(1)
			{
				/* Check Vector Table: Test if user code is programmed starting from address
				"APPLICATION_ADDRESS" */
				if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20020000)
				{
					bootloader_jump_to_user_app();
				}
				else
				{
					//unsuccessful program installation (vector table error)
					HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);
					while(1)
					{
						HAL_Delay(600);
						flash_status_led(5);
					}
				}
			}
			else
			{
				HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);
				while(1)
				{
					HAL_Delay(600);
					flash_status_led(4);
				}
			}
    	}
    	break;
    case APPLICATION_START:
    	if(f_mount(&myUsbFatFS, (TCHAR const*)USBHPath, 0) == FR_OK)
    	{
    		//turn status led on when usb is plugged in
    		HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_SET);
    	}
        break;
    case APPLICATION_READY:
    	COMMAND_DOWNLOAD();
    	HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);
    	while(1)
    	{
    		HAL_Delay(600);
    		HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_SET);
    		flash_status_led(3);
    	}
        break;
    case APPLICATION_DISCONNECT:
    	//turn status led off when usb is unplugged
    	HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);
        break;
    default:
    	//default handler
    	break;
    }
  }
  /* USER CODE END 3 */
}

static void flash_status_led(uint8_t num_of_flashes)
{
	for(uint8_t flashes = 0; flashes < num_of_flashes; flashes++)
	{
		HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_SET);
		HAL_Delay(200);
		HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);
		HAL_Delay(200);
	}
}

FIL myFile;
FRESULT res;
UINT byteswritten, bytesread;
char rwtext[100];

uint8_t usb_test_write(void)
{
	//open or create file for writing
	if(f_open(&myFile, "TEST2.TXT", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
	{
		return 0;
	}
	sprintf(rwtext, "hello from stm32f4.");
	res = f_write(&myFile, (const void*)rwtext, strlen(rwtext), &byteswritten);
	if((res != FR_OK) || (byteswritten == 0))
	{
		return 0;
	}

	f_close(&myFile);
	return 1;
}

uint8_t usb_test_read(void)
{
	return 0;
}

static void bootloader_jump_to_user_app(void)
{
    //just a function pointer to hold the address of the reset handler of the user app
    void (*app_reset_handler)(void);

    //1. configure the MSP by reading the value from the base address of the sector 2
    uint32_t msp_value = *(volatile uint32_t *)APPLICATION_ADDRESS;

    //this function comes from CMSIS
    __set_MSP(msp_value);

    //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

    //now fetch the reset handler address of the user app
    //from the location FLASH_SECTOR2_BASE_ADDRESS+4
    uint32_t resethandler_address = *(volatile uint32_t *) (APPLICATION_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;//this line give void pointer error, suppressed in compiler options!!

    //jump to reset handler of user application
    //bootloader hands off control to user application
    app_reset_handler();
}

/* CRC init function */
void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CRC_MspInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspInit 0 */

  /* USER CODE END CRC_MspInit 0 */
    /* CRC clock enable */
    __HAL_RCC_CRC_CLK_ENABLE();
  /* USER CODE BEGIN CRC_MspInit 1 */

  /* USER CODE END CRC_MspInit 1 */
  }
}

void HAL_CRC_MspDeInit(CRC_HandleTypeDef* crcHandle)
{

  if(crcHandle->Instance==CRC)
  {
  /* USER CODE BEGIN CRC_MspDeInit 0 */

  /* USER CODE END CRC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRC_CLK_DISABLE();
  /* USER CODE BEGIN CRC_MspDeInit 1 */

  /* USER CODE END CRC_MspDeInit 1 */
  }
}

static uint32_t calcCrc32(uint8_t* data, uint32_t len)
{
   uint32_t* pBuffer = (uint32_t*) data;
   uint32_t BufferLength = len/4;
   uint32_t index = 0;
   uint32_t uwCRCValue = 0xff;

   //reset crc unit
    __HAL_CRC_DR_RESET(&hcrc);

   for(index = 0; index < BufferLength; index++)
   {
     uint32_t i_data = pBuffer[index];
     uwCRCValue = HAL_CRC_Accumulate(&hcrc, &i_data, 1);

   }

   return uwCRCValue;
}

uint8_t get_flash_rdp_level(void)
{
    uint8_t rdp_status = 0;

#if 0
    FLASH_OBProgramInitTypeDef ob_handle;
    HAL_FLASHex_OBGetConfig(&ob_handle);
    rdp_status = (uint8_t)ob_handle.RDPLevel;
#else

    volatile uint32_t *pOB_addr = (uint32_t*) 0x1FFFC000;
    rdp_status = (uint8_t)(*pOB_addr >> 8);
#endif

    return rdp_status;

}

uint8_t execute_flash_erase(uint8_t sector_number, uint8_t number_of_sector)
{
    FLASH_EraseInitTypeDef flashErase_handle;
    uint32_t sectorError;
    HAL_StatusTypeDef status;

    if(number_of_sector > 23)
    {
        return INVALID_SECTOR;
    }

    if( (sector_number == 0xff) || (sector_number <= 7))
    {
        if(sector_number == (uint8_t) 0xff)
        {
            flashErase_handle.TypeErase = FLASH_TYPEERASE_MASSERASE;
        }
        else
        {
            uint8_t remaining_sector = 24 - sector_number;
            if( number_of_sector > remaining_sector)
            {
                number_of_sector = remaining_sector;
            }
            flashErase_handle.TypeErase = FLASH_TYPEERASE_SECTORS;
            flashErase_handle.Sector = sector_number;
            flashErase_handle.NbSectors = number_of_sector;

        }
        flashErase_handle.Banks = FLASH_BANK_BOTH;

        HAL_FLASH_Unlock();
        flashErase_handle.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        status = HAL_FLASHEx_Erase(&flashErase_handle, &sectorError);
        HAL_FLASH_Lock();

        return status;
    }

    return INVALID_SECTOR;
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(status_led_GPIO_Port, status_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(usb_power_en_GPIO_Port, usb_power_en_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(usb_power_en_2_GPIO_Port, usb_power_en_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : status_led_Pin */
  GPIO_InitStruct.Pin = status_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(status_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : usb_power_en_Pin */
  GPIO_InitStruct.Pin = usb_power_en_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(usb_power_en_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = usb_power_en_2_Pin;
  HAL_GPIO_Init(usb_power_en_2_GPIO_Port, &GPIO_InitStruct);


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
