/*
 * RCC_register.h
 *
 *  Created on: Mar 6, 2024
 *      Author: user
 */

#ifndef STM32F446XX_H_
#define STM32F446XX_H_

/**********************************          Various Memories Base Addresses          **********************************/
#define FLASH_BASE_ADDRESS          0x08000000UL
#define SRAM_BASE_ADDRESS           0x20000000UL
#define ROM_BASE_ADDRESS            0x1FFF0000UL

/**********************************          CORE Peripheral Base Addresses          **********************************/
#define SYSTIC_BASE_ADDRESS            0xE000E010UL
#define NVIC_BASE_ADDRESS              0xE000E100UL
#define SCB_BASE_ADDRESS               0xE000E008UL

/**********************************          AHB1 Peripheral Base Addresses          **********************************/

#define RCC_BASE_ADDRESS                  0x40023800U

#define GPIOA_BASE_ADDRESS                0x40020000U
#define GPIOB_BASE_ADDRESS                0x40020400U
#define GPIOC_BASE_ADDRESS                0x40020800U
#define GPIOD_BASE_ADDRESS                0x40020C00U
#define GPIOE_BASE_ADDRESS                0x40021000U
#define GPIOF_BASE_ADDRESS                0x40021400U
#define GPIOG_BASE_ADDRESS                0x40021800U
#define GPIOH_BASE_ADDRESS                0x40021C00U

#define DMA1_BASE_ADDRESS                 0x40026000U
#define DMA2_BASE_ADDRESS				  0x40026400U

#define FLASH_INTERFACE_BASE_ADDRESS		0x40023C00UL      /*FLASH INTERFACE REGISTER BASE ADDRESS*/
#define CRC_BASE_ADDRESS		            0x40023000UL	   /*CYCLIC REDUNDANCY CHECH BASE ADDRESS*/

/**********************************          AHB2 Peripheral Base Addresses          **********************************/




/**********************************          AHB3 Peripheral Base Addresses          **********************************/



/**********************************          APB2 Peripheral Base Addresses          **********************************/

#define USART5_BASE_ADDRESS					0x40005000UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 5*/
#define USART4_BASE_ADDRESS					0x40004C00UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 4*/
#define USART3_BASE_ADDRESS					0x40004800UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 3*/
#define USART2_BASE_ADDRESS					0x40004400UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 2*/


/**********************************          APB2 Peripheral Base Addresses          **********************************/
#define SYSCFG_BASE_ADDRESS                  0x40013800U
#define EXTI_BASE_ADDRESS                    0x40013C00U

#define USART6_BASE_ADDRESS					0x40011400UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 6*/
#define USART1_BASE_ADDRESS					0x40011000UL	 /*UNIVERSAL SYNCH/ASYNCH RECEIVER TRANSMITTER 1*/

/**********************************          RCC register definition Structure       **********************************/
typedef struct
{
	volatile uint32_t CR;            /* RCC clock control register ;*/
	volatile uint32_t PLLCFGR;       /* RCC PLL configuration register ;*/
	volatile uint32_t CFGR;          /* RCC clock configuration register ;*/
	volatile uint32_t CIR;           /* RCC clock interrupt register ;*/
	volatile uint32_t AHB1RSTR;      /* RCC AHB1 peripheral reset register ;*/
	volatile uint32_t AHB2RSTR;      /* RCC AHB2 peripheral reset register ;*/
	volatile uint32_t AHB3RSTR;      /* RCC AHB3 peripheral reset register ;*/
	uint32_t Reserved0;              /* Reserved, 0x1c*/
	volatile uint32_t APB1RSTR;      /* RCC APB1 peripheral reset register ;*/
	volatile uint32_t APB2RSTR;      /* RCC APB2 peripheral reset register ;*/
	uint32_t Reserved1[2];           /* Reserved, 0x28-0x2C                                           */
	volatile uint32_t AHB1ENR;       /* RCC AHB1 peripheral clock enable register ;*/
	volatile uint32_t AHB2ENR;       /* RCC AHB2 peripheral clock enable register ;*/
	volatile uint32_t AHB3ENR;       /* RCC AHB3 peripheral clock enable register ;*/
	uint32_t Reserved2;              /* Reserved, 0x3c*/
	volatile uint32_t APB1ENR;       /* RCC APB1 peripheral clock enable register*/
	volatile uint32_t APB2ENR;       /* RCC APB2 peripheral clock enable register*/
	uint32_t Reserved3[2];           /* Reserved, 0x48-0x4C                                           */
	volatile uint32_t AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode register,  */
	volatile uint32_t AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode register,  */
	volatile uint32_t AHB3LPENR;     /* RCC AHB3 peripheral clock enable in low power mode register,  */
	uint32_t      Reserved4;     	 /* Reserved, 0x5C                                                */
	volatile uint32_t APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode register,  */
	volatile uint32_t APB2LPENR;     /* RCC APB2 peripheral clock enable in low power mode register,  */
	uint32_t      Reserved5[2];  	 /* Reserved, 0x68-0x6C                                           */
	volatile uint32_t BDCR;          /* RCC Backup domain control register,                           */
	volatile uint32_t CSR;           /* RCC clock control & status register,                          */
	uint32_t      Reserved6[2];  	 /* Reserved, 0x78-0x7C                                           */
	volatile uint32_t SSCGR;         /* RCC spread spectrum clock generation register,                */
	volatile uint32_t PLLI2SCFGR;    /* RCC PLLI2S configuration register,                            */
	volatile uint32_t PLLSAICFGR;    /* RCC PLLSAI configuration register,                            */
	volatile uint32_t DCKCFGR;       /* RCC Dedicated Clocks configuration register,                  */
	volatile uint32_t CKGATENR;      /* RCC Clocks Gated ENable Register,                             */
	volatile uint32_t DCKCFGR2;      /* RCC Dedicated Clocks configuration register 2,                */

}RCC_Reg_t;
//RCC_Reg_t *RCC  = (RCC_Reg_t*)(0x40023800) ;
#define RCC                   ((RCC_Reg_t*)RCC_BASE_ADDRESS)


/**********************************          GPIO register definition Structure      **********************************/

typedef struct
{
    volatile uint32_t  MODER ;                          /*GPIO PORT Mode Register*/
    volatile uint32_t  OTYPER ;                         /*GPIO PORT Output TypeRegister*/
    volatile uint32_t  OSPEEDR;                        /*GPIO PORT Output Speed Register*/
    volatile uint32_t  PUPDR;                          /*GPIO PORT Pull up/down Register*/
    volatile uint32_t  IDR;                            /*GPIO PORT Input Data Register*/
    volatile uint32_t  ODR;                            /*GPIO PORT Output data Register*/
    volatile uint32_t  BSRR;                           /*GPIO PORT Bit Set/Reset Register*/
    volatile uint32_t  LCKR;                           /*GPIO PORT Lock Register*/
    volatile uint32_t  AFR[2];                         /*GPIO PORT Alternet Function Register*/
}GPIO_RegDef_t;


/**********************************          GPIO Peripheral definitions             **********************************/

#define GPIOA                    ((GPIO_RegDef_t*)GPIOA_BASE_ADDRESS)
#define GPIOB                    ((GPIO_RegDef_t*)GPIOB_BASE_ADDRESS)
#define GPIOC                    ((GPIO_RegDef_t*)GPIOC_BASE_ADDRESS)
#define GPIOD                    ((GPIO_RegDef_t*)GPIOD_BASE_ADDRESS)
#define GPIOE                    ((GPIO_RegDef_t*)GPIOE_BASE_ADDRESS)
#define GPIOF                    ((GPIO_RegDef_t*)GPIOF_BASE_ADDRESS)
#define GPIOG                    ((GPIO_RegDef_t*)GPIOG_BASE_ADDRESS)
#define GPIOH                    ((GPIO_RegDef_t*)GPIOH_BASE_ADDRESS)


/**********************************          SYSTICK register definition Structure       **********************************/

typedef struct{
	volatile uint32_t STK_CTRL;
	volatile uint32_t STK_LOAD;
	volatile uint32_t STK_VAL;
	volatile uint32_t STK_CALIB;
}SYSTIC_REG_t;

#define SYSTIC           ((SYSTIC_REG_t*)SYSTIC_BASE_ADDRESS)

#define STK_CTRL_COUNTFLAG              16u
#define STK_CTRL_CLKSOURCE              2u
#define STK_CTRL_TICKINT                1u
#define STK_CTRL_ENABLE                 0u

/**********************************          NVIC register definition Structure       **********************************/
typedef struct{
	volatile uint32_t ISER[8];
	uint32_t RESERVED0[24];
	volatile uint32_t ICER[8];
	uint32_t RESERVED1[24];
	volatile uint32_t ISPR[8];
	uint32_t RESERVED2[24];
	volatile uint32_t ICPR[8];
	uint32_t RESERVED3[24];
	volatile uint32_t IABR[8];
	uint32_t RESERVED4[56];
	volatile uint32_t IPR[240];
	volatile uint32_t RESERVED5[643];
	volatile uint32_t STIR;
}NVIC_RegDef_t;

#define NVIC                     ((NVIC_RegDef_t*)NVIC_BASE_ADDRESS)


/**********************************          SCB register definition Structure       **********************************/
typedef struct{
	volatile uint32_t  ACTLR ;
	uint32_t RESERVED0[829] ;
	volatile uint32_t CPUID ;
	volatile uint32_t ICSR ;
	volatile uint32_t VTOR ;
	volatile uint32_t AIRCR ;
	volatile uint32_t SCR ;
	volatile uint32_t CCR ;
	volatile uint32_t SHPR1 ;
	volatile uint32_t SHPR2 ;
	volatile uint32_t SHPR3 ;
	volatile uint32_t SHCSR ;
	volatile uint32_t MMSR ;
	volatile uint32_t BFSR ;
	volatile uint32_t UFSR ;
	volatile uint32_t HFSR ;
	uint32_t RESERVED1 ;
	volatile uint32_t MMAR ;
	volatile uint32_t BFAR ;
	volatile uint32_t AFSR ;

}SCB_RegDef_t;

#define SCB                      ((SCB_RegDef_t*)SCB_BASE_ADDRESS)

/**********************************          SYSCFG register definition Structure       **********************************/
typedef struct{
	volatile uint32_t SYSCFG_MEMRMP;
	volatile uint32_t SYSCFG_PMC;
	volatile uint32_t SYSCFG_EXTICR[4];
	volatile uint32_t SYSCFG_CMPCR;
	volatile uint32_t SYSCFG_CFGR;

}SYSCFG_RegDef_t;


#define SYSCFG               ((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDRESS)

/**********************************          EXTI register definition Structure       **********************************/
typedef struct{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
}EXTI_RegDef_t;

#define EXTI                     ((EXTI_RegDef_t*)EXTI_BASE_ADDRESS)

/**********************************          DMA register definition Structure      **********************************/
typedef struct{
	volatile uint32_t DMA_SxCR;
	volatile uint32_t DMA_SxNDTR;
	volatile uint32_t DMA_SxPAR;
	volatile uint32_t DMA_SxM0AR;
	volatile uint32_t DMA_SxM1AR;
	volatile uint32_t DMA_SxFCR;

}DMA_StreamReg_t;
typedef struct{
	volatile uint32_t DMA_ISR[2];
	volatile uint32_t DMA_IFCR[2];

	volatile DMA_StreamReg_t STREAM_REG[8] ;
}DMA_RegDef_t;

#define DMA1                     ((DMA_RegDef_t*)DMA1_BASE_ADDRESS)
#define DMA2                     ((DMA_RegDef_t*)DMA2_BASE_ADDRESS)


/***************USART Register Definition Structure****************/
typedef struct{

	volatile uint32_t USART_SR;
	volatile uint32_t USART_DR;
	volatile uint32_t USART_BRR;
	volatile uint32_t USART_CR1;
	volatile uint32_t USART_CR2;
	volatile uint32_t USART_CR3;
	volatile uint32_t USART_GTPR;

}USART_Reg_t;
/***************USART Register Definition ****************/
#define USART1	((USART_Reg_t*)USART1_BASE_ADDRESS)
#define USART2	((USART_Reg_t*)USART2_BASE_ADDRESS)
#define USART3	((USART_Reg_t*)USART3_BASE_ADDRESS)
#define USART4	((USART_Reg_t*)USART4_BASE_ADDRESS)
#define USART5	((USART_Reg_t*)USART5_BASE_ADDRESS)
#define USART6	((USART_Reg_t*)USART6_BASE_ADDRESS)

/******************** CRC Register Definition Structure *****************/
typedef struct
{
	volatile uint32_t DR;
	volatile uint32_t IDR;
	volatile uint32_t CR;

}CRC_RegDef_T;

#define CRC			((CRC_RegDef_T *)(CRC_BASE_ADDRESS))

/******************** CRC Register Definition Structure *****************/
typedef struct
{
	volatile uint32_t ACR;
	volatile uint32_t KEYR;
	volatile uint32_t OPTKEYR;
	volatile uint32_t SR;
	volatile uint32_t CR;
	volatile uint32_t OPTCR;
}FLASH_Interface_RegDef_T;

#define FLASH_INTERFACE		((FLASH_Interface_RegDef_T *)(FLASH_INTERFACE_BASE_ADDRESS))


#endif /* STM32F446XX_H_ */
