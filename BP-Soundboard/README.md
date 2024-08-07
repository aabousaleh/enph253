This is notes for Michael so he remembers how to do things.
# USART: BP - AD2 Communications
## Setup Function
Add this or else no idle event detection will happen. Not sure why this wasn't in the FUCKING IOC FILE FOR FUCK SAKE.
```
__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
```

## Callback Function
```
void HAL_UARTEx_RxEventCallback (UART_HandleTypeDef * huart, uint16_t Size)
{

	//HAL_UART_Transmit(&huart1, data, 5, 0xFFFF);
	if (huart->Instance == USART1) {
		uint8_t validData = 0;
		//uint16_t dacValue0;
		//uint16_t dacValue1;
		if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_TC){
			if(inputBuffer[3] == 'A'){
				validData = 1;
				//dacValue0 = (inputBuffer[0] << 4) & ((inputBuffer[1] & 240) >> 4);
				//dacValue1 = ((inputBuffer[1] & 15) << 8) * inputBuffer[2];
			}
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, inputBuffer, UART_INPUT_BUFFER_SIZE);
		}
		if(HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE){
			HAL_UART_DMAStop(&huart1);
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, inputBuffer, UART_INPUT_BUFFER_SIZE);
		}

		if(validData == 1){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		}
	}
}
```
# SPI: BP - DAC Communication:
Currently, the SPI bus runs at 9MPs. This could be problematic in a more noisy environment.
## Example Writing High to DAC0 and DAC1
```
uint8_t dacHigh[] = {0x00,0x0F,0xFF,0x08,0x0F,0xFF};
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET);       //Latch previous output
HAL_SPI_Transmit_DMA(&hspi1, dacHigh, sizeof(dacHigh));
```
## Callback Function
```
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  //Latch out previous data
    }
}
```
