#ifndef AX_ENCODER_EXTENDED_H
#define AX_ENCODER_EXTENDED_H
#include <stdint.h>
void AX_ENCODER_ExtendedInit(unsigned channel);
int32_t AX_ENCODER_ReadExtended(unsigned channel);
void AX_ENCODER_SetExtended(unsigned channel,uint16_t value);
#endif
