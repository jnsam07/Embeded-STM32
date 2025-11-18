/* USER CODE BEGIN PV */
uint8_t RxBuffer[1];
/* USER CODE END PV */


/* USER CODE BEGIN WHILE */
while (1)
{
	if (HAL_UART_Receive(&huart2, RxBuffer, 1, 1) == HAL_OK)
	{
		if (RxBuffer[0] >= 'A' && RxBuffer[0] <= 'Z')
		{
			RxBuffer[0] = RxBuffer[0] - 'A' + 'a';
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		}
		else if (RxBuffer[0] >= 'a' && RxBuffer[0] <= 'z')
		{
			RxBuffer[0] = RxBuffer[0] - 'a' + 'A';
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		}
		else if (RxBuffer[0] >= '1' && RxBuffer[0] <= '9')
		{
			RxBuffer[0] = 10 - (RxBuffer[0] - '0') + '0';
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		}
		else if (RxBuffer[0] == '0')
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		else if (RxBuffer[0] == '\r')
		{
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
			RxBuffer[0] = '\n';
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		}
		else
		{
			RxBuffer[0] = '?';
			HAL_UART_Transmit(&huart2, RxBuffer, 1, 1);
		}
	}
}
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
/* USER CODE END 3 */