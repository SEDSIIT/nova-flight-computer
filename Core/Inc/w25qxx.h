#ifndef _W25QXX_H
#define _W25QXX_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include "main.h"

	// Standard SPI instruction addresses
	#define W25QXX_WRITE_ENABLE 0x06
	#define W25QXX_VOLATILE_SR_WRITE_ENABLE 0x50
	#define W25QXX_WRITE_DISABLE 0x04
	#define W25QXX_READ_STATUS_REGISTER_1 0x05
	#define W25QXX_READ_STATUS_REGISTER_2 0x35
	#define W25QXX_READ_STATUS_REGISTER_3 0x15
	#define W25QXX_WRITE_STATUS_REGISTER_1 0x01
	#define W25QXX_WRITE_STATUS_REGISTER_2 0x31
	#define W25QXX_WRITE_STATUS_REGISTER_3 0x11
	#define W25QXX_PAGE_PROGRAM 0x02
	#define W25QXX_SECTOR_ERASE 0x20
	#define W25QXX_BLOCK_ERASE_SMALL 0x52
	#define W25QXX_BLOCK_ERASE_LARGE 0xD8
	#define W25QXX_CHIP_ERASE 0xC7
	#define W25QXX_ERASE_PROGRAM_SUSPEND 0x75
	#define W25QXX_ERASE_PROGRAM_RESUME 0x7A
	#define W25QXX_POWER_DOWN 0xB9
	#define W25QXX_READ_DATA 0x03
	#define W25QXX_FAST_READ 0x0B
	#define W25QXX_RELEASE_POWERDOWN 0xAB
	#define W25QXX_DEVICE_ID 0x90
	#define W25QXX_JEDEC_ID 0x9F
	#define W25QXX_READ_UNIQUE_ID 0x4B
	#define W25QXX_READ_SFDP_REGISTER 0x5A
	#define W25QXX_ERASE_SECURITY_REGISTERS 0x44
	#define W25QXX_PROGRAM_SECURITY_REGISTERS 0x42
	#define W25QXX_READ_SECUIRTY_REGISTERS 0x48
	#define W25QXX_ENABLE_QPI 0x38
	#define W25QXX_ENABLE_RESET 0x66
	#define W25QXX_RESET 0x99

	#define W25QXX_DUMMY_BYTE 0xA5

	typedef enum
	{
		W25Q10 = 1,
		W25Q20,
		W25Q40,
		W25Q80,
		W25Q16,
		W25Q32,
		W25Q64,
		W25Q128,
		W25Q256,
		W25Q512,

	} W25QXX_ID_t;

	typedef struct
	{
		W25QXX_ID_t ID;
		uint8_t UniqID[8];
		uint16_t PageSize;
		uint32_t PageCount;
		uint32_t SectorSize;
		uint32_t SectorCount;
		uint32_t BlockSize;
		uint32_t BlockCount;
		uint32_t CapacityInKiloByte;
		uint8_t StatusRegister1;
		uint8_t StatusRegister2;
		uint8_t StatusRegister3;
		uint8_t Lock;

	} w25qxx_t;

	extern w25qxx_t w25qxx;
	//############################################################################
	// in Page,Sector and block read/write functions, can put 0 to read maximum bytes
	//############################################################################
	bool W25qxx_Init(void);

	void W25qxx_EraseChip(void);
	void W25qxx_EraseSector(uint32_t SectorAddr);
	void W25qxx_EraseBlock(uint32_t BlockAddr);

	uint32_t W25qxx_PageToSector(uint32_t PageAddress);
	uint32_t W25qxx_PageToBlock(uint32_t PageAddress);
	uint32_t W25qxx_SectorToBlock(uint32_t SectorAddress);
	uint32_t W25qxx_SectorToPage(uint32_t SectorAddress);
	uint32_t W25qxx_BlockToPage(uint32_t BlockAddress);

	bool W25qxx_IsEmptyPage(uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize);
	bool W25qxx_IsEmptySector(uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize);
	bool W25qxx_IsEmptyBlock(uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize);

	void W25qxx_WriteByte(uint8_t pBuffer, uint32_t Bytes_Address);
	void W25qxx_WritePage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize);
	void W25qxx_WriteSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize);
	void W25qxx_WriteBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize);

	void W25qxx_ReadByte(uint8_t *pBuffer, uint32_t Bytes_Address);
	void W25qxx_ReadBytes(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
	void W25qxx_ReadPage(uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize);
	void W25qxx_ReadSector(uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize);
	void W25qxx_ReadBlock(uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize);
//############################################################################
#ifdef __cplusplus
}
#endif

#endif
