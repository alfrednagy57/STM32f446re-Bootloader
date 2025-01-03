
#ifndef INC_BL_PRV_H_
#define INC_BL_PRV_H_

#define Add_Valid  0x9F
#define Add_NValid 0xA0

#define BL_VERION 1U

#define BL_ACK		0xA5
#define BL_NACK		0x7F

#define FLASH_SEC2_BASE_ADD 0x8008000U

#define DBGMCU_IDCODE	(*(volatile uint32_t *const)0xE0042000U)

#define OPTIONBYTES_RDP (*(volatile uint32_t *const)0x1FFFC000U)

#define BL_MASS_ERASE_CODE 0xFFU

#define Writing_Suc 0U
#define Writing_Err 1U


static void voidSendAck(const uint8_t ReplyLen);
void voidSendNAck(void);
static uint8_t u8_ValidateAdd(uint32_t Copy_u32ADD);
static void Void_BL_JMP_TO_ADD(uint32_t ADD);
static uint8_t u8_ExeFlashERASE(uint8_t SecNum,uint8_t NoOfSec);
static uint8_t u8_ExeMemWrite(const uint8_t *DataBuff,uint32_t BaseAdd,uint8_t PayloadLen);

#endif /* INC_BL_PRV_H_ */
