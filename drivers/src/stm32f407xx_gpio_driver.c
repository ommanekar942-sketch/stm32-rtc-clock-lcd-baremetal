
#include "stm32f407xx_gpio_driver.h"

// API supported by this driver

// peripheral clock setup
void GPIO_PeriClockControl(GPIO_REGDEF_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
				{
					GPIOA_PCLK_DI();
				}
				else if(pGPIOx == GPIOB)
				{
					GPIOB_PCLK_DI();
				}
				else if(pGPIOx == GPIOC)
				{
					GPIOC_PCLK_DI();
				}
				else if(pGPIOx == GPIOD)
				{
					GPIOD_PCLK_DI();
				}
				else if(pGPIOx == GPIOE)
				{
					GPIOE_PCLK_DI();
				}
				else if(pGPIOx == GPIOF)
				{
					GPIOF_PCLK_DI();
				}
				else if(pGPIOx == GPIOG)
				{
					GPIOG_PCLK_DI();
				}
				else if(pGPIOx == GPIOH)
				{
					GPIOH_PCLK_DI();
				}
				else if(pGPIOx == GPIOI)
				{
					GPIOI_PCLK_DI();
				}
	}
}

// Initialize and de-initialize
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{

	// peripheral clock enable
	 GPIO_PeriClockControl(pGPIO_Handle->pGPIOx, ENABLE);

	uint32_t temp=0;   //temp register
	// 1.Configure the mode of GPIO pin
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
	// non interrupt mode
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->MODER |= temp;
    }

	else
	{
		// interrupt mode
		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			// 1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			// 1. configure the RTSR
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 1. configure the RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR

		//SYSCFG is required to-->
		//Connect a specific GPIO pin (like PB3, PC13) to a specific EXTI line (0–15)
		//Without SYSCFG, the EXTI lines won't know which GPIO port to respond to.

		uint8_t temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASE_ADDR_TO_CODE(pGPIO_Handle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2*4);

		// 3. enable the EXTI interrupt delivery using IMR --> This step enables the interrupt delivery for that EXTI line to the NVIC.
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp=0;

	// 2. configure the speed
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;
	temp=0;

	// 3. configure the PuPd setting
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~(0x3 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->PUPDR |= temp;
	temp=0;

	// 4. configure the OPTYPE
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~(0x1 << pGPIO_Handle-> GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	temp=0;

	// 5. configure the alt type functionality
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIO_Handle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2) );
		pGPIO_Handle->pGPIOx->AFR[temp1] |= (pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );
	}
}

void GPIO_DeInit(GPIO_REGDEF_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if(pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if(pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}
}

// data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_REGDEF_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber ,uint8_t Value)
{
	if(Value ==  GPIO_PIN_SET ) {
		pGPIOx -> ODR |= (1<<PinNumber);
	}
	else
	{
		pGPIOx -> ODR &= ~(1<<PinNumber);

	}
}

void GPIO_WriteToOutputPort(GPIO_REGDEF_t *pGPIOx, uint16_t Value)
{
	pGPIOx -> ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_REGDEF_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx -> ODR ^= (1<<PinNumber);
}

// IRQ configuration and ISR handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		if(IRQNumber<=31)
		{
			// ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber>31 && IRQNumber<64)
		{
			// ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}
		else if(IRQNumber>64 && IRQNumber<96)
		{
			// ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
	}
	else
	{
		if(IRQNumber<=31)
				{
					// ICER0 register
			        *NVIC_ICER0 |= (1 << IRQNumber);
				}
				else if(IRQNumber>31 && IRQNumber<64)
				{
					// ICER1 register
					*NVIC_ICER1 |= (1 << IRQNumber%32);
				}
				else if(IRQNumber>64 && IRQNumber<96)
				{
					// ICER2 register
					*NVIC_ICER2 |= (1 << IRQNumber%64);
				}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQpriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber%4;
	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx ) |= (IRQpriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)    // IRQ handling knows which PinNumber get triggered
{
	// clear EXTI PR register corresponding to Pin Number
	if(EXTI->PR & (1 << PinNumber))
	{
		// clear
		EXTI->PR |= (1 << PinNumber);
	}

	//  Why this?

	//The peripheral sets a pending flag when interrupt occurs.

	// If we don’t clear it, the CPU will keep entering the IRQ again and again.
}
