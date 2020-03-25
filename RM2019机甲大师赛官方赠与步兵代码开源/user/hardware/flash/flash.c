#include "flash.h"
#include "stm32f4xx.h"

static uint32_t GetSector(unsigned int Address);
static uint32_t Get_Next_Flash_Address(uint32_t Address);
static int8_t FLASH_Erase_Muli_Sector(uint32_t start_Address, uint32_t end_Address, uint8_t VoltageRange);

int8_t flash_write_single_address(uint32_t Address, uint32_t *buf, uint32_t len)
{
    uint32_t uwSector = 0;

    uint32_t uwAddress = 0;
    uint32_t uwEndAddress = 0;
    uint32_t uwSectorCounter = 0;
    uint32_t *data_buf;
    uint32_t data_len;
    __IO uint32_t uwData32 = 0;
    __IO uint32_t uwMemoryProgramStatus = 0;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    uwSector = GetSector(Address);

    uwSectorCounter = uwSector;

    if (FLASH_EraseSector(uwSectorCounter, VoltageRange_3) != FLASH_COMPLETE)
    {

        return -1;
    }

    uwAddress = Address;
    uwEndAddress = Get_Next_Flash_Address(Address);
    data_buf = buf;
    data_len = 0;
    while (uwAddress <= uwEndAddress)
    {
        if (FLASH_ProgramWord(uwAddress, *data_buf) == FLASH_COMPLETE)
        {
            uwAddress = uwAddress + 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {

            return -1;
        }
    }

    FLASH_Lock();

    uwAddress = Address;
    uwMemoryProgramStatus = 0;
    data_buf = buf;
    data_len = 0;

    while (uwAddress < uwEndAddress)
    {
        uwData32 = *(__IO uint32_t *)uwAddress;

        if (uwData32 != *data_buf)
        {
            uwMemoryProgramStatus++;
        }

        uwAddress = uwAddress + 4;
        data_buf++;
        data_len++;
        if (data_len == len)
        {
            break;
        }
    }

    if (uwMemoryProgramStatus)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int8_t flash_write_muli_address(uint32_t start_Address, uint32_t end_Address, uint32_t *buf, uint32_t len)
{

    uint32_t uwAddress = 0;

    uint32_t *data_buf;
    uint32_t data_len;

    __IO uint32_t uwData32 = 0;
    __IO uint32_t uwMemoryProgramStatus = 0;

    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_Erase_Muli_Sector(start_Address, start_Address, VoltageRange_3);

    uwAddress = start_Address;

    data_buf = buf;
    data_len = 0;
    while (uwAddress <= end_Address)
    {

        if (FLASH_ProgramWord(uwAddress, *data_buf) == FLASH_COMPLETE)
        {
            uwAddress = uwAddress + 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {

            return -1;
        }
    }

    FLASH_Lock();

    uwAddress = start_Address;

    data_buf = buf;
    data_len = 0;
    uwMemoryProgramStatus = 0;

    while (uwAddress < end_Address)
    {
        uwData32 = *(__IO uint32_t *)uwAddress;

        if (uwData32 != *data_buf)
        {
            uwMemoryProgramStatus++;
        }

        uwAddress = uwAddress + 4;
        data_buf++;
        data_len++;
        if (data_len == len)
        {
            break;
        }
    }

    if (uwMemoryProgramStatus)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}
void flash_read(uint32_t Address, uint32_t *buf, uint32_t len)
{
    uint32_t i = 0;
    __IO uint32_t uwData32;
    uint32_t uwAddress = 0;
    uwAddress = Address;
    for (i = 0; i < len; i++)
    {
        uwData32 = *(__IO uint32_t *)uwAddress;
        *buf = uwData32;
        uwAddress = uwAddress + 4;
        buf++;
    }
}

static int8_t FLASH_Erase_Muli_Sector(uint32_t start_Address, uint32_t end_Address, uint8_t VoltageRange)
{

    uint32_t uwStartSector = 0;
    uint32_t uwEndSector = 0;

    uint32_t uwSectorCounter = 0;

    uwStartSector = GetSector(start_Address);
    uwEndSector = GetSector(end_Address);

    uwSectorCounter = uwStartSector;
    while (uwSectorCounter <= uwEndSector)
    {

        if (FLASH_EraseSector(uwSectorCounter, VoltageRange) != FLASH_COMPLETE)
        {

            return -1;
        }

        if (uwSectorCounter == FLASH_Sector_11)
        {
            uwSectorCounter += 40;
        }
        else
        {
            uwSectorCounter += 8;
        }
    }
    return 0;
}

#define FLASH_WRP_SECTORS (OB_WRP_Sector_0 | OB_WRP_Sector_1)
__IO uint32_t SectorsWRPStatus = 0xFFF;

void write_protect(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    SectorsWRPStatus = FLASH_OB_GetWRP() & FLASH_WRP_SECTORS;

    if (SectorsWRPStatus == 0x00)
    {
        return;
    }
    else
    {

        FLASH_OB_Unlock();

        FLASH_OB_WRPConfig(FLASH_WRP_SECTORS, ENABLE);

        status = FLASH_OB_Launch();

        if (status != FLASH_COMPLETE)
        {
            return;
        }

        FLASH_OB_Lock();
    }
}

void write_relieve_protect(void)
{
    FLASH_Status status = FLASH_COMPLETE;

    SectorsWRPStatus = FLASH_OB_GetWRP() & FLASH_WRP_SECTORS;

    if (SectorsWRPStatus == 0x00)
    {

        FLASH_OB_Unlock();

        FLASH_OB_WRPConfig(FLASH_WRP_SECTORS, DISABLE);
        status = FLASH_OB_Launch();

        if (status != FLASH_COMPLETE)
        {
            return;
        }

        FLASH_OB_Lock();
    }
}

static uint32_t GetSector(uint32_t Address)
{
    uint32_t sector = 0;

    if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_Sector_0;
    }
    else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_Sector_1;
    }
    else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_Sector_2;
    }
    else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_Sector_3;
    }
    else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_Sector_4;
    }
    else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_Sector_5;
    }
    else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_Sector_6;
    }
    else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_Sector_7;
    }
    else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_Sector_8;
    }
    else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_Sector_9;
    }
    else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_Sector_10;
    }

    else if ((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
    {
        sector = FLASH_Sector_11;
    }

    else if ((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
    {
        sector = FLASH_Sector_12;
    }
    else if ((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
    {
        sector = FLASH_Sector_13;
    }
    else if ((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
    {
        sector = FLASH_Sector_14;
    }
    else if ((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
    {
        sector = FLASH_Sector_15;
    }
    else if ((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
    {
        sector = FLASH_Sector_16;
    }
    else if ((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
    {
        sector = FLASH_Sector_17;
    }
    else if ((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
    {
        sector = FLASH_Sector_18;
    }
    else if ((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
    {
        sector = FLASH_Sector_19;
    }
    else if ((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
    {
        sector = FLASH_Sector_20;
    }
    else if ((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
    {
        sector = FLASH_Sector_21;
    }
    else if ((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
    {
        sector = FLASH_Sector_22;
    }
    else /*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_Sector_23;
    }
    return sector;
}

static uint32_t Get_Next_Flash_Address(uint32_t Address)
{
    uint32_t sector = 0;

    if ((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else if ((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
    else if ((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
    else if ((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
    else if ((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }

    else if ((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
    {
        sector = ADDR_FLASH_SECTOR_12;
    }

    else if ((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
    {
        sector = ADDR_FLASH_SECTOR_13;
    }
    else if ((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
    {
        sector = ADDR_FLASH_SECTOR_14;
    }
    else if ((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
    {
        sector = ADDR_FLASH_SECTOR_15;
    }
    else if ((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
    {
        sector = ADDR_FLASH_SECTOR_16;
    }
    else if ((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
    {
        sector = ADDR_FLASH_SECTOR_17;
    }
    else if ((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
    {
        sector = ADDR_FLASH_SECTOR_18;
    }
    else if ((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
    {
        sector = ADDR_FLASH_SECTOR_19;
    }
    else if ((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
    {
        sector = ADDR_FLASH_SECTOR_20;
    }
    else if ((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
    {
        sector = ADDR_FLASH_SECTOR_21;
    }
    else if ((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
    {
        sector = ADDR_FLASH_SECTOR_22;
    }
    else if ((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
    {
        sector = ADDR_FLASH_SECTOR_23;
    }
    else /*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}
