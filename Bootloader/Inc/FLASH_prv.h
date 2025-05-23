#ifndef FLASH_PRV_H_
#define FLASH_PRV_H_

#define FLASH_TIMEOUT						0xFFFFFFFFU

#define FLASH_UNLOCK_FLASH_KEY1				0x45670123
#define FLASH_UNLOCK_FLASH_KEY2				0xCDEF89AB


#define FLASH_LOCK_VAL					    1U
#define FLASH_LOCK_SHIFT				    31U

#define FLASH_BUSY_MASK					    0x00080000
#define FLASH_BUSY_SHIFT				    16U

#define FLASH_IS_BUSY					    1U
#define FLASH_IS_READY					    0U

#define FLASH_PARALLEL_SHIFT			    8U

#define FLASH_SECTOR_ERASE_EN_SHIFT		    1U
#define FLASH_MASS_ERASE_EN_SHIFT		    2U

#define FLASH_ERASE_SNB_SHIFT			    3U
#define FLASH_ERASE_STRT_SHIFT			    16U

#define FLASH_EOP_SHIFT					    0U

#define FLASH_UNLOCK_OTP_KEY1				0x08192A3B
#define FLASH_UNLOCK_OTP_KEY2				0x4C5D6E7F

#define FLASH_SPRMOD_SHIFT					31U
#define FLASH_nWRP_SHIFT					16U
#define FLASH_nWRP_MASK						0xFF


#define FLASH_OTPSTRT_SHIFT					1U

typedef enum
{
	READ_PROTECT,
	WRITE_PROTECT
}Flash_ProtectMode_T;

#endif /* FLASH_PRV_H_ */
