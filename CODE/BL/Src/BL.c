#include <stdint.h>

#include "../../Core/Inc/main.h"

#include "../Header/BootLoader.h"
#include "../Header/BL_Prv.h"

extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;

uint8_t u8_VerifyCrc(uint8_t * const Copyu8_dataARR,uint8_t LEN_Arr,uint32_t u32_CopyHostCRC)
{
	uint32_t Local_u32AccCRCVAL=0;
	uint32_t Local_u32CopyTemp=0;
	uint8_t  Local_u8CRCStatus=0;
	uint8_t  Local_u8Iterator=0;

	for(Local_u8Iterator=0;Local_u8Iterator<LEN_Arr;Local_u8Iterator++)
	{
		Local_u32CopyTemp = (uint32_t)(Copyu8_dataARR[Local_u8Iterator]);

		Local_u32AccCRCVAL = HAL_CRC_Accumulate(&hcrc, &Local_u32CopyTemp, 1);
	}
	/*Reset CRC Calculation unit*/
	__HAL_CRC_DR_RESET(&hcrc);

	if (Local_u32AccCRCVAL == u32_CopyHostCRC)
	{
		Local_u8CRCStatus = CRC_SUCCES;
	}
	else
	{
		Local_u8CRCStatus = CRC_FAIL;
	}

	return Local_u8CRCStatus;
}

static void voidSendAck(const uint8_t ReplyLen)
{
	uint8_t Local_u8AckBuffer[2]={BL_ACK,ReplyLen};

	HAL_UART_Transmit(&huart2, Local_u8AckBuffer, 2, HAL_MAX_DELAY);
}
void voidSendNAck(void)
{
	uint8_t Local_u8NACK = BL_NACK;

	HAL_UART_Transmit(&huart2, &Local_u8NACK, 1, HAL_MAX_DELAY);
}

static uint8_t u8_ValidateAdd(uint32_t Copy_u32ADD)
{
	/*address is valid if is within SRAM or Flash*/

	uint8_t Local_u8State=0;

	if(Copy_u32ADD>=FLASH_BASE&&Copy_u32ADD<=FLASH_END)
	{
		Local_u8State=Add_Valid;
	}
	else if(Copy_u32ADD>=SRAM1_BASE&&Copy_u32ADD<=(SRAM1_BASE+128*1024))
	{
		Local_u8State=Add_Valid;
	}
	else
	{
		Local_u8State=Add_NValid;
	}

	return Local_u8State;
}
static void Void_BL_JMP_TO_ADD(uint32_t ADD)
{
	/*pointer to fnc.holds the add of the reset handler of userapp*/
	void (*App_HANDLER)(void)=NULL;

	/*Inc add with 1. TO match 1st bit with T-Bit in control reg*/
	ADD|=1U;

	uint32_t MSP_BASE_ADD = (*(uint32_t *)(FLASH_SEC2_BASE_ADD));

	__asm volatile("MSR MSP,%0"::"r"(MSP_BASE_ADD));

	App_HANDLER = (void *)ADD;

	App_HANDLER();
}
static uint8_t u8_ExeFlashERASE(uint8_t SecNum,uint8_t NoOfSec)
{
	FLASH_EraseInitTypeDef LocalSectorErase ;
	uint32_t LocalSectorErr;

	if(SecNum==BL_MASS_ERASE_CODE)
	{
		LocalSectorErase.TypeErase=FLASH_TYPEERASE_MASSERASE;
	}
	else if(SecNum>0U&&SecNum<7U)
	{

		uint8_t LocalRemSec = (8U-SecNum);

		if(NoOfSec>LocalRemSec)
		{
			/*if no of sectors is bigger than max. make it equal to max.*/
			NoOfSec = LocalRemSec;
		}
		else
		{
			/*Nothing*/
		}

		LocalSectorErase.TypeErase=FLASH_TYPEERASE_SECTORS;
		LocalSectorErase.Sector=SecNum;
		LocalSectorErase.NbSectors=NoOfSec;
	}
	else
	{
		return HAL_ERROR;
	}

	LocalSectorErase.Banks=FLASH_BANK_1;
	LocalSectorErase.VoltageRange=FLASH_VOLTAGE_RANGE_3;

	/*unlock the flash before using*/
	HAL_FLASH_Unlock();

	/*start erasing*/
	HAL_FLASHEx_Erase(&LocalSectorErase,&LocalSectorErr);

	/*lock the flash after lock using*/
	HAL_FLASH_Lock();

	return HAL_OK;
}
static uint8_t u8_ExeMemWrite(const uint8_t *DataBuff,uint32_t BaseAdd,uint8_t PayloadLen)
{
	uint8_t Local_u8ErrState = HAL_OK;
	uint8_t Local_u8Iterartor = 0U;

	if( (BaseAdd>=FLASH_BASE) && (BaseAdd<=FLASH_END) )
	{
		/*
		 *
		 * writing to Flash case
		 *
		 * */

		/*Unlock the flash*/
		HAL_FLASH_Unlock();

		for(Local_u8Iterartor = 0U;Local_u8Iterartor<PayloadLen;Local_u8Iterartor++)
		{
			Local_u8ErrState=HAL_FLASH_Program((uint32_t)FLASH_TYPEPROGRAM_BYTE,(uint32_t)(BaseAdd+Local_u8Iterartor),(uint64_t)DataBuff[Local_u8Iterartor]);
		}

		/*Lock the flash*/
		HAL_FLASH_Lock();
	}
	else
	{
		/*
		 *
		 * writing to SRAM1 case
		 *
		 * */
		uint8_t *SRAM_ADD = (uint8_t *)BaseAdd;

		for(Local_u8Iterartor = 0U;Local_u8Iterartor<PayloadLen;Local_u8Iterartor++)
		{
			*(SRAM_ADD+Local_u8Iterartor) = *(DataBuff+Local_u8Iterartor);
		}
	}

	return Local_u8ErrState;
}
void BL_voidHandlerGetVerCMD()
{

	uint8_t Local_u8BLVer=0U;

	voidSendAck(1U);

	Local_u8BLVer = BL_VERION;

	HAL_UART_Transmit(&huart2, &Local_u8BLVer, 1, HAL_MAX_DELAY);
}
void BL_voidHandlerGetHelpCMD(uint8_t * const CMDpacket)
{

	uint8_t Local_u8BLGetHelp[]=
	{
			BL_GET_VER              ,
			BL_GET_HELP             ,
			BL_GET_CID              ,
			BL_GET_RDP_STATUS       ,
			BL_GO_TO_ADDR           ,
			BL_FLASH_ERASE          ,
			BL_MEM_WRITE            ,
			BL_EN_R_W_PROTECT       ,
			BL_MEM_READ             ,
			BL_READ_SECTOR_P_STATUS ,
			BL_OTP_READ             ,
			BL_DIS_R_W_PROTECT
	};
	voidSendAck(sizeof(Local_u8BLGetHelp));

	HAL_UART_Transmit(&huart2, Local_u8BLGetHelp, sizeof(Local_u8BLGetHelp), HAL_MAX_DELAY);

}
void BL_voidHandlerGetCIDCMD()
{
	uint16_t Localu16DeviceID= (uint16_t)DBGMCU_IDCODE&0xFFF;

	voidSendAck(2U);

	HAL_UART_Transmit(&huart2, (uint8_t *)&Localu16DeviceID,2, HAL_MAX_DELAY);

}
void BL_voidHandlerGetRDPStateCMD()
{
	uint8_t Mess = (OPTIONBYTES_RDP&0xFF00)>>8;

	voidSendAck(1U);

	HAL_UART_Transmit(&huart2,&Mess, 1U, HAL_MAX_DELAY);
}
void BL_voidHandlerGoTOADDCMD(uint8_t * const CMDpacket)
{

	voidSendAck(1U);

	uint32_t Local_u32DesAdd=0U;

	Local_u32DesAdd=(*(uint32_t *)&CMDpacket[2]);

	uint8_t Local_u8AddValidState=u8_ValidateAdd(Local_u32DesAdd);

	HAL_UART_Transmit(&huart2, &Local_u8AddValidState, 1U, HAL_MAX_DELAY);

	if(Local_u8AddValidState == Add_Valid)
	{

		Void_BL_JMP_TO_ADD(Local_u32DesAdd);
	}
	else
	{
		/*Nothing*/
	}
}
void BL_voidHandlerFlashEraseCMD(uint8_t * const CMDpacket)
{

	uint8_t EraseState=0U;
	uint8_t SecNum = CMDpacket[2];
	uint8_t NumSec = CMDpacket[3];

	voidSendAck(1U);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	EraseState=u8_ExeFlashERASE(SecNum,NumSec);

	HAL_UART_Transmit(&huart2,&EraseState,1U,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
}
void BL_voidHandlerMemWriteCMD(uint8_t * const CMDpacket)
{

	uint32_t Local_u8MemAdd = (*(uint32_t *)&CMDpacket[2]);

	uint8_t Local_u8AddValidState=u8_ValidateAdd(Local_u8MemAdd);

	uint8_t Local_u8WritingState = Writing_Suc;

	voidSendAck(1U);/*Sending ACK & Reply length*/

	if(Local_u8AddValidState == Add_Valid)
	{
		uint8_t Local_u8PayloadLen = CMDpacket[6];

		Local_u8WritingState = u8_ExeMemWrite(&CMDpacket[7],Local_u8MemAdd,Local_u8PayloadLen);
	}
	else
	{
		Local_u8WritingState = Writing_Err;
	}

	HAL_UART_Transmit(&huart2, &Local_u8WritingState, 1U, HAL_MAX_DELAY);
}
void BL_voidHandlerEnRW_protectCMD(uint8_t * const CMDpacket)
{

	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	HAL_FLASH_OB_Unlock();

	voidSendAck(0U);

	*(__IO uint8_t *)OPTCR_BYTE1_ADDRESS = 0xFF; // level 1
	*(__IO uint8_t *)OPTCR_BYTE2_ADDRESS = (~CMDpacket[2])&CMDpacket[3];

	HAL_FLASH_OB_Lock();
}
void BL_voidHandlerMemReadCMD(uint8_t * const CMDpacket)
{
	uint8_t Local_Len = CMDpacket[6];

	uint8_t Local_u8Uart_Reply[255]={0};

	uint32_t Locat_u32_BaseAdd=CMDpacket[2];

	voidSendAck(Local_Len);

	uint8_t i = 0;

	for(i=0;i<Local_Len;i++)
	{
		Local_u8Uart_Reply[i]= *((uint8_t *)(Locat_u32_BaseAdd+i));
	}
	HAL_UART_Transmit(&huart2,Local_u8Uart_Reply,(uint16_t)(i+i),HAL_MAX_DELAY);
}
void BL_voidHandlerReadSecPROStateCMD()
{
	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	HAL_FLASH_OB_Unlock();

	voidSendAck(1U);

	uint8_t Protect_state = 0U;

	Protect_state=((*(__IO uint8_t *)OPTCR_BYTE2_ADDRESS)>>16)&0xff;

	HAL_FLASH_OB_Lock();

	HAL_UART_Transmit(&huart2,&Protect_state,1U,HAL_MAX_DELAY);
}
void BL_voidHandlerOTPReadCMD()
{

	uint8_t Local_u8_OptionBYTES_RED[8]={0};

	uint8_t i =0;

	voidSendAck(8U);

	for(i=0;i<8;i++)
	{
		Local_u8_OptionBYTES_RED[i]= *((uint8_t*)(0x1FFFC000U+i));
	}

	HAL_FLASH_OB_Lock();

	HAL_UART_Transmit(&huart2,Local_u8_OptionBYTES_RED,8U,HAL_MAX_DELAY);

}
void BL_voidHandlerDISRW_protectCMD()
{

	FLASH_WaitForLastOperation(HAL_MAX_DELAY);

	HAL_FLASH_OB_Unlock();

	*(__IO uint8_t *)OPTCR_BYTE1_ADDRESS = 0xAA;//Level 0
	*(__IO uint8_t *)OPTCR_BYTE2_ADDRESS = 0x00;

	HAL_FLASH_OB_Lock();

	voidSendAck(0U);
}
