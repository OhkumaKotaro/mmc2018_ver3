#include "flash.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "maze_info.h"

extern maze_t maze;

// flash use address ( sector11 )
const uint32_t start_address = 0x80E0000; //sentor11 start address
const uint32_t end_adress = 0x80FFFFF;


///////////////////////////////////////////////////////////////////////
// ereaseFlash
// [argument] nothing
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////

void eraseFlash(void)
{
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS; // セクタを選ぶ
    erase.Sector = FLASH_SECTOR_11; // セクタ11を指定
    erase.NbSectors = 1; // 消すセクタの数(今回はひとつ)
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 3.3Vで消去
    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&erase, &pageError); // HAL_FLASHExの関数で消去
}
///////////////////////////////////////////////////////////////////////
// writeFlash
// [argument] address, *data, size
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void writeFlash(uint32_t address, uint8_t *data, uint32_t size)
{
    HAL_FLASH_Unlock(); // フラッシュをアンロック
    eraseFlash(); // セクタ11を消去
    while (size > 0)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, address, *data);// 1byteずつフラッシュに書き込む
        ++address;
        ++data;
        --size;
    }
   HAL_FLASH_Lock(); // フラッシュをロック
}
///////////////////////////////////////////////////////////////////////
// loadFlash
// [argument] address, *data, size
// [Substitutiong] struct
// [return] nothing
///////////////////////////////////////////////////////////////////////
void loadFlash(uint32_t address, uint8_t *data, uint32_t size)
{
    memcpy(data, (uint8_t *)address, size);// addressの内容をコピー
}

void writeMaze(void){
    writeFlash(start_address,(uint8_t*)&maze,sizeof(maze_t));
}

void loadMaze(void){
    loadFlash(start_address,(uint8_t*)&maze,sizeof(maze_t));
}