/**
  ******************************************************************************
  * @file    FW_upgrade/src/command.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    28-October-2011
  * @brief   This file provides all the IAP command functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "command.h"
#include "main.h"

/** @addtogroup STM32F4-Discovery_FW_Upgrade
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define UPLOAD_FILENAME            "0:UPLOAD.BIN"
#define DOWNLOAD_FILENAME          "image.bin"

/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t RAM_Buf[BUFFER_SIZE] =
  {
    0x00
  };
static uint32_t TmpProgramCounter = 0x00, TmpReadSize = 0x00 , RamAddress = 0x00;
static uint32_t LastPGAddress = APPLICATION_ADDRESS;

//extern FATFS fatfs;
extern FIL file;
FIL fileR;
//extern DIR dir;
//extern FILINFO fno;

//extern USB_OTG_CORE_HANDLE          USB_OTG_Core;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief IAP Read all flash memory
  * @param  None
  * @retval None
  */
void COMMAND_UPLOAD(void)
{
  __IO uint32_t address = APPLICATION_ADDRESS;
  __IO uint32_t counterread = 0x00;

  uint32_t tmpcounter = 0x00, indexoffset = 0x00;
  FlagStatus readoutstatus = SET;
  uint16_t bytesWritten;

  /* Get the read out protection status */
  readoutstatus = FLASH_If_ReadOutProtectionStatus();
  if (readoutstatus == RESET)
  {
    /* Remove UPLOAD file if exist on flash disk */
    f_unlink (UPLOAD_FILENAME);

    /* Init written byte counter */
    indexoffset = (APPLICATION_ADDRESS - USER_FLASH_STARTADDRESS);

    /* Open binary file to write on it */
    if ((f_open(&file, UPLOAD_FILENAME, FA_CREATE_ALWAYS | FA_WRITE) == FR_OK))
    {
      /* Read flash memory */
      while ((indexoffset != USER_FLASH_SIZE))
      {
        for (counterread = 0; counterread < BUFFER_SIZE; counterread++)
        {
          /* Check the read bytes versus the end of flash */
          if (indexoffset + counterread != USER_FLASH_SIZE)
          {
            tmpcounter = counterread;
            RAM_Buf[tmpcounter] = (*(uint8_t*)(address++));
          }
          /* In this case all flash was read */
          else
          {
            break;
          }
        }

        /* Write buffer to file */
        f_write (&file, RAM_Buf, BUFFER_SIZE, (void *)&bytesWritten);

        /* Number of byte written  */
        indexoffset = indexoffset + counterread;
      }

      /* Set Green LED ON: Upload Done */
//      STM_EVAL_LEDOff(LED6);
//      STM_EVAL_LEDOn(LED4);

      /* Close file and filesystem */
      f_close (&file);
      f_mount(0, NULL, 0);
    }
  }
  else
  {
    /* Message ROP active: Set Orange LED ON and Toggle Red LED in infinite loop */
//    STM_EVAL_LEDOn(LED3);
//    Fail_Handler();
  }
}

/**
  * @brief  IAP write memory
  * @param  None
  * @retval None
  */
void COMMAND_DOWNLOAD(void)
{
    FRESULT fatfs_res = FR_OK;
    /* Open the binary file to be downloaded */
    fatfs_res = f_open(&fileR, DOWNLOAD_FILENAME, FA_READ);
    if (fatfs_res == FR_OK)
    {
        uint32_t file_size = f_size(&fileR);
        if (file_size > USER_FLASH_SIZE)
        {
            //file too large error
        }
        else
        {
            /* Download On Going */
            /* Erase FLASH sectors to download image */
            if (execute_flash_erase(3,21) != 0x00)
            {
                //flash erase error
            }
            /* Program flash memory */
            COMMAND_ProgramFlashMemory();

            /* Close file and filesystem */
            f_close (&fileR);
        }
    }
    else
    {
        //file not available error
    }
}

/**
  * @brief  IAP jump to user program
  * @param  None
  * @retval None
  */
void COMMAND_JUMP(void)
{
  /* Software reset */
  NVIC_SystemReset();
}

/**
  * @brief  Programs the internal Flash memory
  * @param  None
  * @retval None
  */
void COMMAND_ProgramFlashMemory(void)
{
    __IO uint32_t programcounter = 0x00;
    uint8_t readflag = 1;
    uint16_t BytesRead;
    volatile FRESULT res;

    /* RAM Address Initialization */
    RamAddress = (uint32_t) & RAM_Buf;

    /* Erase address init */
    LastPGAddress = APPLICATION_ADDRESS;

    /* While file still contain data */
    while ((readflag == 1))
    {
        /* Read maximum 512 Kbyte from the selected file */
        res = f_read (&fileR, RAM_Buf, BUFFER_SIZE, (void *)&BytesRead);
        if(res)
        {
            HAL_Delay(1);
        }

        /* Temp variable */
        TmpReadSize = BytesRead;

        /* The read data < "BUFFER_SIZE" Kbyte */
        if (TmpReadSize < BUFFER_SIZE)
        {
            readflag = 0;
        }

        /* Program flash memory */
        HAL_FLASH_Unlock();
        for (programcounter = TmpReadSize; programcounter != 0; programcounter -= 4)
        {
            TmpProgramCounter = programcounter;
            /* Write word into flash memory */
            if (FLASH_If_ProgramWord((LastPGAddress - TmpProgramCounter + TmpReadSize), \
            *(__IO uint32_t *)(RamAddress - programcounter + TmpReadSize)) != HAL_OK)
            {
                /* Toggle Red LED in infinite loop: Flash programming error */
                while(1);
            }

            if(programcounter < 4)
            {
                programcounter = 4;
            }
        }

        HAL_FLASH_Lock();
        /* Update last programmed address value */
        LastPGAddress = LastPGAddress + TmpReadSize;
    }
}

/**
  * @}
  */

/*******************(C)COPYRIGHT 2011 STMicroelectronics *****END OF FILE******/
