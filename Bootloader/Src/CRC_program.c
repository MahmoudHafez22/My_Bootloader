#include <stdint.h>
#include "Stm32F446xx.h"
#include "CRC_interface.h"
#include "ErrType.h"


uint32_t CRC_u32tCalculate(uint32_t *Copt_pu32Data,uint32_t Copy_u32Length)
{
	uint32_t Local_u32Iterator , Local_u32CRCVal ;

	for(Local_u32Iterator=0 ; Local_u32Iterator < Copy_u32Length ; Local_u32Iterator++)
	{
		CRC->DR = Copt_pu32Data[Local_u32Iterator];
	}
	Local_u32CRCVal = CRC->DR ;

	return Local_u32CRCVal ;
}

void CRC_voidReset(void)
{
	CRC->CR |=1 ;
}

