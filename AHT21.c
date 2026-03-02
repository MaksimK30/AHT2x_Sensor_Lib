#include "AHT21.h"

AHT* AHT21Init(I2C_HandleTypeDef *port, uint8_t address) {
	
	//Если не удалось выделить память для объекта датчика - вернём NULL
	AHT* sensor = (AHT*)malloc(sizeof(AHT));	
	if(sensor == NULL){
		return NULL; 
	}
	
	//Если не удалось выделить память для буфера - вернём NULL,
	// освободим память
	sensor->buff = (uint8_t*)calloc(8, 1);
	if(sensor->buff == NULL){
		free(sensor);
		return NULL; 
	}

	//Если не удалось выделить память для указателя на статус ответа - вернём NULL,
	// освободим память
	sensor->ret = (HAL_StatusTypeDef*)malloc(sizeof(HAL_StatusTypeDef));
	if(sensor->ret == NULL){	
		free(sensor->buff);
		free(sensor);
		return NULL;
	}
	
	//Пробуем связаться с датчиком
	*sensor->ret = HAL_I2C_IsDeviceReady(port, address, 2,
			120);
	
	//Ошибка при попытке связаться с датчиком
	if (*sensor->ret != HAL_OK) {	
		free(sensor->buff);
		free(sensor->ret);
		free(sensor);
		return NULL;
	}
	
	sensor->buff[0] = AHT21_CONNECT_BYTE;
	*sensor->ret = HAL_I2C_Master_Transmit(port, address, sensor->buff, 1, 100);
	
	//Ошибка запроса соединения
	if (*sensor->ret != HAL_OK) {
		free(sensor->buff);
		free(sensor->ret);
		free(sensor);
		return NULL;
	}
	
	*sensor->ret = HAL_I2C_Master_Receive(port, address, sensor->buff, 1, 100);
	
	sensor->buff[0] = sensor->buff[0] & 0x18;
	
	//Соединение не удалось
	if (sensor->buff[0] != 0x18) {	
		free(sensor->buff);
		free(sensor->ret);
		free(sensor);		
		return NULL;
	}
	
	sensor->humidity = 0;
	sensor->temperature = 0;
	sensor->port = port;
	sensor->address = address;
	sensor->lastError = AHT21_OK;
	
	return sensor;
}

AHT21_STATUS readInfo(AHT *sensor){
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0X33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 3, 100);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = AHT21_TX_ERROR;
		return AHT21_TX_ERROR;
	}	
	
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 6, 100);

	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {		
		sensor->lastError = AHT21_RX_ERROR;
		return AHT21_RX_ERROR;
	}
	
	//Считаем влажность
	sensor->humidity = (sensor->buff[1] << 12) | (sensor->buff[2] << 4) | (sensor->buff[3] >> 4);
	sensor->humidity = (sensor->humidity * 100);
	sensor->humidity = sensor->humidity / 0x100000;

	//Считаем температуру
	sensor->temperature = ((sensor->buff[3] & 0xF) << 16) | (sensor->buff[4] << 8) | (sensor->buff[5]);
	sensor->temperature = (sensor->temperature * 200);
	sensor->temperature = sensor->temperature / 0x100000;
	sensor->temperature = sensor->temperature - 50;
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

void AHT21Deleter(AHT* sensor){
	free(sensor->buff);
	free(sensor->ret);
	free(sensor);
}

/////////////////////////////NON-BLOCKING MODE WITH INTERRUPTIONS/////////////////////////////

AHT21_STATUS sendInfoRequestIT(AHT *sensor){
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0X33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = AHT21_TX_ERROR;
		return AHT21_TX_ERROR;
	}		
	
	//Возвращаем флаг успешного выполнения	
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

AHT21_STATUS parseInfoRequestIT(AHT *sensor){
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive_IT(sensor->port, sensor->address, sensor->buff, 6);

	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {		
		sensor->lastError = AHT21_RX_ERROR;
		return AHT21_RX_ERROR;
	}
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}
/////////////////////////////NON-BLOCKING MODE WITH DMA/////////////////////////////

AHT21_STATUS sendInfoRequestDMA(AHT *sensor){
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0X33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = AHT21_TX_ERROR;
		return AHT21_TX_ERROR;
	}		
	
	//Возвращаем флаг успешного выполнения	
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

AHT21_STATUS parseInfoRequestDMA(AHT *sensor){
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive_DMA(sensor->port, sensor->address, sensor->buff, 6);

	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {		
		sensor->lastError = AHT21_RX_ERROR;
		return AHT21_RX_ERROR;
	}
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}