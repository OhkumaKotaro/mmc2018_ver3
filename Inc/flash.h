/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __flash_H
#define __flash_H
#ifdef __cplusplus
 extern "C" {
#endif

#include "stdint.h"

// flash use address ( sector11 )
extern const uint32_t start_address; //sentor11 start address
extern const uint32_t end_adress;

void eraseFlash(void);
void writeFlash(uint32_t address, uint8_t *data, uint32_t size);
void loadFlash(uint32_t address, uint8_t *data, uint32_t size);
void writeMaze(void);
void loadMaze(void);


#ifdef __cplusplus
}
#endif
#endif /*__flash_H */