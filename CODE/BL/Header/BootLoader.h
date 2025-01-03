#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

/*
 * BL Commands
 * */
#define BL_GET_VER               0x51
#define BL_GET_HELP              0x52
#define BL_GET_CID               0x53
#define BL_GET_RDP_STATUS        0x54
#define BL_GO_TO_ADDR            0x55
#define BL_FLASH_ERASE           0x56
#define BL_MEM_WRITE             0x57
#define BL_EN_R_W_PROTECT        0x58
#define BL_MEM_READ              0x59
#define BL_READ_SECTOR_P_STATUS  0x5A
#define BL_OTP_READ              0x5B
#define BL_DIS_R_W_PROTECT       0x5C
//#define BL_MY_NEW_COMMAND        0x5D

/*
 * len details of the command
 * */
#define MAX_BL_GET_VER_LEN                 6
#define MAX_BL_GET_HELP_LEN                6
#define MAX_BL_GET_CID_LEN                 6
#define MAX_BL_GET_RDP_STATUS_LEN          6
#define MAX_BL_GO_TO_ADDR_LEN              10
#define MAX_BL_FLASH_ERASE_LEN             8
#define MAX_BL_MEM_WRITE_LEN               11
#define MAX_BL_EN_R_W_PROTECT_LEN          8
#define MAX_BL_READ_SECTOR_P_STATUS_LEN    6
#define MAX_BL_DIS_R_W_PROTECT_LEN         6
#define MAX_BL_MY_NEW_COMMAND_LEN          8

void BL_voidHandlerGetVerCMD();
void BL_voidHandlerGetHelpCMD();
void BL_voidHandlerGetCIDCMD();
void BL_voidHandlerGetRDPStateCMD();
void BL_voidHandlerGoTOADDCMD(uint8_t * const CMDpacket);
void BL_voidHandlerFlashEraseCMD(uint8_t * const CMDpacket);
void BL_voidHandlerMemWriteCMD(uint8_t * const CMDpacket);
void BL_voidHandlerEnRW_protectCMD(uint8_t * const CMDpacket);
void BL_voidHandlerMemReadCMD(uint8_t * const CMDpacket);
void BL_voidHandlerReadSecPROStateCMD();
void BL_voidHandlerOTPReadCMD();
void BL_voidHandlerDISRW_protectCMD();

uint8_t u8_VerifyCrc(uint8_t * const Copyu8_dataARR,uint8_t LEN_Arr,uint32_t u32_CopyHostCRC);

#endif /* INC_BOOTLOADER_H_ */









