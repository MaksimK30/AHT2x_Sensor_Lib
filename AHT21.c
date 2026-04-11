#include "AHT21.h"

AHT21* AHT21_Init(I2C_HandleTypeDef *port, uint8_t address) {
	
	//Если не удалось выделить память для объекта датчика - вернём NULL
	AHT21 *sensor = (AHT21*)malloc(sizeof(AHT21));	
	if(sensor == NULL){
		return NULL; 
	}
	
	//Если не удалось выделить память для буфера - вернём NULL,
	// освободим память
	sensor->buff = (uint8_t*)calloc(8, sizeof(uint8_t));
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
	
	sensor->buff[0] = AHT21_CONNECT_BYTE;
	*sensor->ret = HAL_I2C_Master_Transmit(port, address, sensor->buff, 1, HAL_MAX_DELAY);
	//Ошибка запроса соединения
	if (*sensor->ret != HAL_OK) {
		//В случае, если проверка не прошла, может помочь 
		// повторная инициализация байтами 0xBE, 0x08, 0x00.
		free(sensor->buff);
		free(sensor->ret);
		free(sensor);
		return NULL;
	}
	
	*sensor->ret = HAL_I2C_Master_Receive(port, address, sensor->buff, 1, 200);
	
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

AHT21_STATUS AHT21_ReadInfo(AHT21 *sensor){
	uint8_t status;
  uint32_t timeout;
	
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0x33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 3, 100);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = AHT21_TX_ERROR;
		return AHT21_TX_ERROR;
	}	
	
	HAL_Delay(100);
	
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 7, 100);
	
	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {		
		sensor->lastError = AHT21_RX_ERROR;
		return AHT21_RX_ERROR;
	}
	 
	//Вычисляем CRC, проверяем ответ
	if(AHT21_CalcCRC8(sensor) != sensor->buff[6]){
		sensor->lastError = AHT21_CRC_ERROR;
		return AHT21_CRC_ERROR;
	}
	
	//Считаем влажность
	uint32_t raw_hum = ((uint32_t)sensor->buff[1] << 12) | ((uint32_t)sensor->buff[2] << 4) | ((uint32_t)sensor->buff[3] >> 4);
	sensor->humidity = ((float)raw_hum / 1048576.0f) * 100.0f;

	//Считаем температуру
	uint32_t raw_temp = ((sensor->buff[3] & 0xF) << 16) | (sensor->buff[4] << 8) | (sensor->buff[5]);
	sensor->temperature = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;	
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

void AHT21_Deleter(AHT21 *sensor){
	free(sensor->buff);
	free(sensor->ret);
	free(sensor);
	sensor = NULL;
}

void AHT21_ReadInfoFromBuffer(AHT21 *sensor){
	//Вычисляем CRC, проверяем ответ
	if(AHT21_CalcCRC8(sensor) != sensor->buff[6]){
		sensor->lastError = AHT21_CRC_ERROR;
		return;
	}
	
	//Считаем влажность
	uint32_t raw_hum = ((uint32_t)sensor->buff[1] << 12) | ((uint32_t)sensor->buff[2] << 4) | ((uint32_t)sensor->buff[3] >> 4);
	sensor->humidity = ((float)raw_hum / 1048576.0f) * 100.0f;

	//Считаем температуру
	uint32_t raw_temp = ((sensor->buff[3] & 0xF) << 16) | (sensor->buff[4] << 8) | (sensor->buff[5]);
	sensor->temperature = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;	
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
}

uint8_t AHT21_CalcCRC8(AHT21 *sensor) {
    uint8_t crc = 0xFF;
    const uint8_t polynomial = 0x31; // x^8 + x^5 + x^4 + 1
    
    for (uint8_t i = 0; i < 6; i++) {
        crc ^= sensor->buff[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
/////////////////////////////NON-BLOCKING MODE WITH INTERRUPTIONS/////////////////////////////

AHT21_STATUS AHT21_SendInfoRequest_IT(AHT21 *sensor){
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0X33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = *sensor->ret == HAL_BUSY ? AHT21_BUSY : AHT21_TX_ERROR;
		return sensor->lastError;
	}				
	
	//Возвращаем флаг успешного выполнения	
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

AHT21_STATUS AHT21_ParseInfoRequest_IT(AHT21 *sensor){
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive_IT(sensor->port, sensor->address, sensor->buff, 7);

	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = *sensor->ret == HAL_BUSY ? AHT21_BUSY : AHT21_RX_ERROR;
		return sensor->lastError;
	}		
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}
/////////////////////////////NON-BLOCKING MODE WITH DMA/////////////////////////////

AHT21_STATUS AHT21_SendInfoRequest_DMA(AHT21 *sensor){
	//Записываем команду
	sensor->buff[0] = 0xAC;
	sensor->buff[1] = 0x33;
	sensor->buff[2] = 0x00;
	
	//Отсылаем запрос
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если ошибка - вернуть статус ошибки и 
	//установить его в структуре датчика
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = *sensor->ret == HAL_BUSY ? AHT21_BUSY : AHT21_TX_ERROR;
		return sensor->lastError;
	}		
	
	//Возвращаем флаг успешного выполнения	
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}

AHT21_STATUS AHT21_ParseInfoRequest_DMA(AHT21 *sensor){
	//Обрабатываем данные
	*sensor->ret = HAL_I2C_Master_Receive_DMA(sensor->port, sensor->address, sensor->buff, 7);

	//Отсылаем запрос
	if (*sensor->ret != HAL_OK) {
		sensor->lastError = *sensor->ret == HAL_BUSY ? AHT21_BUSY : AHT21_RX_ERROR;
		return sensor->lastError;
	}		
	
	//Возвращаем флаг успешного выполнения
	sensor->lastError = AHT21_OK;
	return AHT21_OK;
}