#include "I2C.h"

#if BMP280_I2C
	void I2C_Conf(uint16_t SCK_speed)
	{
		GPIO_InitTypeDef GPIOInit;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
		 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

		GPIO_StructInit(&GPIOInit);

		//Definition of pins for I2C protocol GPIOB_PIN6 - SCL, GPIOB_PIN7 - SDA
		GPIOInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIOInit.GPIO_Mode = GPIO_Mode_AF_OD;
		GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
		 GPIO_Init(GPIOB, &GPIOInit);


		//Konfiguracja I2C1
		I2C_InitTypeDef I2CInit;
		I2C_StructInit(&I2CInit);
		//Tryb pracy uk³adu
		I2CInit.I2C_Mode = I2C_Mode_I2C;
		//Stosunek trwania stanu wysokiego do niskiego linii SCK
		I2CInit.I2C_DutyCycle = I2C_DutyCycle_2;
		//Adres I2C po stronie mikrokontrolera jeœli pracuje jako slave
		I2CInit.I2C_OwnAddress1 = 0x00;
		//Wlaczenie potwierdzenia transmisji
		I2CInit.I2C_Ack = I2C_Ack_Enable;
		 //Czêstotliwoœc SCK 400 kHz
		I2CInit.I2C_ClockSpeed = 100 * SCK_speed;

		I2C_Init(I2C1, &I2CInit);
		I2C_Cmd(I2C1, ENABLE);
	}
#endif

#if BMP280_I2C
	//Ustawienie adresu pamiêci
	void I2C_ADDRES(uint8_t SLA, uint32_t addr)
	{
	 //Rozpoczêcie komunikacji
	 I2C_GenerateSTART(I2C1, ENABLE);
	 //Czekanie a¿ polecenie zostanie wykonane
	 while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);
	 //W tym kroku wys³any jest identyfikator uk³adu
	 I2C_Send7bitAddress(I2C1, SLA, I2C_Direction_Transmitter);
	 //Odczekanie na odpowiedŸ
	 while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS);

	 //Wys³anie adresu komórki pamiêci do/z jakiej bêd¹ zapisywane/odczytywane dane.
	 I2C_SendData(I2C1, addr);
	 while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);

	}
#endif

#if BMP280_I2C
	//Odczyt z pamiêci
	void I2C_READ(uint8_t SLA, uint32_t addr,  int size, void* data)
	{
		  //Deklaracja zmiennych
		  int i;
		  uint8_t* buffer = (uint8_t*)data;

		  //Wybór adresu urz¹dzenia
		  I2C_ADDRES(SLA, addr);

		  //Wys³anie sygna³u start, nastêpnie czekanie na odpowiedŸ
		  I2C_GenerateSTART(I2C1, ENABLE);
		  while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS);

		  //Odbiór tylko jednego bajtu, wlacz potwierdzenie
		  I2C_AcknowledgeConfig(I2C1, ENABLE);
		  //Wys³anie adresu uk³adu, ustawienie i2c jako master w trybie received mode
		  I2C_Send7bitAddress(I2C1, SLA, I2C_Direction_Receiver);
		  //Odczekanie na EV6
		  while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS);

		  for (i = 0; i < size - 1; i++)
		  {
		   while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
		   buffer[i] = I2C_ReceiveData(I2C1);
		  }
		  //Odbiór pojedyñczego bajtu
		  I2C_AcknowledgeConfig(I2C1, DISABLE);

		  //Wygenerowanie sygna³u stop
		  I2C_GenerateSTOP(I2C1, ENABLE);
		  //Odczekanie na sygna³
		  while(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS);
		  //Odczytaj dane z rejestru
		  buffer[i] = I2C_ReceiveData(I2C1);
	}
#endif


#if BMP280_I2C
	//Zapis do pamiêci
	void I2C_WRITE(uint8_t SLA, uint32_t addr, int size, const void* data)
	{
	  int i;
	  const uint8_t* buffer = (uint8_t*)data;

	  //Wybór adresu urz¹dzenia
	  I2C_ADDRES(SLA, addr);

	  //Przejœcie przez ca³y buffor z danymi
	  for (i = 0; i < size; i++)
	  {
	   I2C_SendData(I2C1, buffer[i]);
	   //while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS);
	   while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS);
	  }
	  //Wygenerowanie sygna³u Stop
	  I2C_GenerateSTOP(I2C1, ENABLE);
	}

#endif
