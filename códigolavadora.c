/*
 * lavadora.c
 *
 *  Created on: , 2024
 *      Author: valeriealvarez
 */


#include <stdint.h>
#include "stm32l053xx.h"  // LIBRERIA  UNIFICADA

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif


void timer2();
void timer6();
void timer21();


uint32_t delay=0x00;
volatile uint8_t lavado = 0;
volatile uint8_t clock_base_counter = 0x00;
volatile uint8_t seconds_unit = 0x00;
void print_bcd_7_segment_decoder_CC(uint8_t);

// USART
void delay_ms(uint16_t n);
void USART2_write(uint8_t ch);
void USART2_putstring(uint8_t* StringPtr);
void USART2_putstring_E(uint8_t* StringPtr);
uint8_t USART2_read(void);



void EXTI4_15_IRQHandler(void) {
    if (EXTI->PR & (1 << 5)) { // Check if the interrupt is from PA5
        EXTI->PR |= (1 << 5); // Clear the interrupt pending bit
        lavado = 1; // Set lavado to 1
    }
}

int main(void){


	//ENABLE HSI 16MHZ
		//HSI ON
	RCC->CR |=(1<<0);
		//HSI16 AS SYSCLK
	RCC->CFGR |=(1<<0);

	//ENABLE GPIOA CLK
	RCC->IOPENR |=(1<<0);
	//ENABLE GPIOB CLK
	RCC->IOPENR |=(1<<1);
	//ENABLE GPIOC CLK
	RCC->IOPENR |=(1<<2);

    /* Configuración de pines para USART2 */
    GPIOA->MODER &= ~(1<<4); // PA2 como Alternate Function
    GPIOA->MODER &= ~(1<<6); // PA3 como Alternate Function
    GPIOA->AFR[0] |= 1<<10; // PA2 mapeado como AF4
    GPIOA->AFR[0] |= 1<<14; // PA3 mapeado como AF4

    /* Configuración del periférico USART2 */
    RCC->APB1ENR |= 1<<17; // Habilitar USART2
    USART2->BRR = 1666; // Configurar baud rate para 9600 16Mhz
   // USART2->BRR = 218; // Configurar baud rate para 9600 2.097mhz
    USART2->CR1 |= (1<<2)|(1<<3); // Habilitar transmisor y receptor
    USART2->CR1 |= 1<<0; // Habilitar USART2
	//ENABLE SYSCFG CLK
	RCC->APB2ENR |=(1<<0);

	//GPIO CONFIG
	//PC9 AS OUTPUT
	GPIOC->MODER &=~(1<<19);
		//PC8 AS OUTPUT
	GPIOC->MODER &=~(1<<17);
//	GPIOC->ODR |=(1<<8);
//	GPIOC->ODR &=~(1<<8);
		//PC6 AS OUTPUT
	GPIOC->MODER &=~(1<<13);
		//PC5 AS OUTPUT
	GPIOC->MODER &=~(1<<11);
	//PC3 AS OUTPUT
	GPIOC->MODER &=~(1<<7);
		//PA5 AS INPUT
	GPIOA->MODER &=~(1<<10);
	GPIOA->MODER &=~(1<<11);


    // Configure PA5 for external interrupt
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PA;
    EXTI->IMR |= (1 << 5); // Unmask interrupt for line 5
    EXTI->FTSR |= (1 << 5); // Trigger on falling edge

    // Enable EXTI4_15 interrupt
    NVIC_EnableIRQ(EXTI4_15_IRQn);

    //DISPLAY

	GPIOB->MODER &=~(1<<17);	//PB8 COMO SALIDA CON EL SEGMENTO A
	GPIOB->MODER &=~(1<<5);		//PB2 COMO SALIDA CON EL SEGMENTO B
	GPIOB->MODER &=~(1<<7);		//PB3 COMO SALIDA CON EL SEGMENTO C
	GPIOB->MODER &=~(1<<9);		//PB4 COMO SALIDA CON EL SEGMENTO D
	GPIOB->MODER &=~(1<<13);	//PB6 COMO SALIDA CON EL SEGMENTO E
	GPIOB->MODER &=~(1<<15);	//PB7 COMO SALIDA CON EL SEGMENTO F
	GPIOB->MODER &=~(1<<19);	//PB9 COMO SALIDA CON EL SEGMENTO H

	GPIOA->MODER &=~(1<<1);		//PA0 COMO HABILITADOR DEL COMUN DEL DISPLAY 0


	timer2();
	timer6();
	timer21();
	USART2_putstring_E("PRECIONE INICIO PARA QUE EMPIECE EL LAVADO");
	print_bcd_7_segment_decoder_CC(0x00);
					 					 		GPIOA->ODR ^=(1<<0);
	while(1){

        // 1s
		 if (lavado == 1) {

				 	if((TIM2->SR & 0x00000001)){ //CHECK UIF == 1
				 		TIM2->SR &=~1;			//CLEAR UIF SI NO LA LIMPIO NUNCA SE VA A LIMPIAR
				 				//GPIOC->ODR ^=(1<<9);
				 					clock_base_counter=clock_base_counter + 1;
				 					if(clock_base_counter == 10){
				 						seconds_unit = seconds_unit + 1;
				 						clock_base_counter = 0;
				 						//GPIOC->ODR ^=(1<<10);    // base del contador que cuando sume 10 secuenciencias del TIM2 sea 1 segundo.
					}
				 	}
				 	else if(seconds_unit == 1 ){

				 					 		GPIOC->ODR ^=(1<<9);   // CICLO DE LLENADO
				 					 		print_bcd_7_segment_decoder_CC(0x01);
				 					 		GPIOA->ODR ^=(1<<0);

				 	                	}

				 	else if(seconds_unit == 3 ){
				 		// USART2_putstring_E("CICLO RINSE");
				 		if((TIM21->SR & 0x00000001)){
				 		TIM21->SR &=~1;
				 		GPIOC->ODR &=~(1<<9);   //APAGA CICLO DE LLENADO
				 		GPIOC->ODR ^=(1<<8);      //ENCIENDE CICLO DE ENJUAGE
				 		print_bcd_7_segment_decoder_CC(0x02);
				 						 					 		GPIOA->ODR ^=(1<<0);

                	}
				 	}
				 		else if(seconds_unit == 4) {
				 		//	 USART2_putstring_E("CICLO RINSE");
				 		if((TIM21->SR & 0x00000001)){
				 						 		TIM21->SR &=~1;
				 					 GPIOC->ODR &=~(1<<8);   //APAGA CICLO DE LLENADO
				 						 		GPIOC->ODR ^=(1<<6);      //ENCIENDE CICLO DE ENJUAGE
				 						 		print_bcd_7_segment_decoder_CC(0x02);
				 						 						 					 		GPIOA->ODR ^=(1<<0);
				 	}
				 		}
				 		else if(seconds_unit == 5 ){
				 						 		if((TIM21->SR & 0x00000001)){
				 						 		TIM21->SR &=~1;
				 						 		GPIOC->ODR &=~(1<<9);   //APAGA CICLO DE LLENADO
				 						 		GPIOC->ODR ^=(1<<8);      //ENCIENDE CICLO DE ENJUAGE
				 						 		print_bcd_7_segment_decoder_CC(0x02);
				 						 						 					 		GPIOA->ODR ^=(1<<0);
				 		                	}
				 						 	}
				 						 		else if(seconds_unit == 6) {
				 						 		if((TIM21->SR & 0x00000001)){
				 						 						 		TIM21->SR &=~1;
				 						 					 GPIOC->ODR &=~(1<<8);   //APAGA CICLO DE LLENADO
				 						 						 		GPIOC->ODR ^=(1<<6);      //ENCIENDE CICLO DE ENJUAGE
				 						 						 		print_bcd_7_segment_decoder_CC(0x02);
				 						 						 						 					 		GPIOA->ODR ^=(1<<0);
				 						 	}
				 						 		}

				 	else if(seconds_unit == 7 ){
				 		// USART2_putstring_E("REMOJO");
				 					 		GPIOC->ODR &=~(1<<8);
				 					 		GPIOC->ODR &=~(1<<6);
				 					 		print_bcd_7_segment_decoder_CC(0x03);
				 					 						 					 		GPIOA->ODR ^=(1<<0);

				 					 	}
					else if(seconds_unit == 8) {
						// USART2_putstring_E("VACIADO DE TANQUE");
								 						 					 GPIOC->ODR &=~(1<<6);   //APAGA CICLO DE LLENADO
								 						 						 		GPIOC->ODR ^=(1<<5);      //ENCIENDE vaciado de agua
								 						 						 		print_bcd_7_segment_decoder_CC(0x04);
								 						 						 						 					 		GPIOA->ODR ^=(1<<0);
								 						 		}
					else if(seconds_unit == 9) {
					//	 USART2_putstring_E("EXPRIMIENDO");
						if((TIM21->SR & 0x00000001)){
										 						 		TIM21->SR &=~1;

										 						 		GPIOC->ODR ^=(1<<8);      //ENCIENDE CICLO CENTRIFUGADO

										 						 						 					 		GPIOA->ODR ^=(1<<0);
													 						 					GPIOC->ODR &=~(1<<5);   //APAGA CICLO VACIADO
							w						 						 						 //		GPIOC->ODR ^=(1<<8);      //ENCIENDE CENTRIFUGADO
													 						 						 		print_bcd_7_segment_decoder_CC(0x05);
													 						 						 						 					 		GPIOA->ODR ^=(1<<0);
													 						 		}

					}

				 	else if (seconds_unit == 13){
				 		USART2_putstring_E("FIN DEL LAVADO, PUEDE ABRIR TAPA");
				 		GPIOC->ODR &=~(1<<8);
				 		print_bcd_7_segment_decoder_CC(0x0F);
				 						 					 		GPIOA->ODR ^=(1<<0);
				 		seconds_unit = 0;
				 		lavado =0;
				 		clock_base_counter = 0;
				 	}

}
}

}


void timer2(){

	//TIMER 2 CONFIG 1s
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<0);	//AQUI ES DONDE VIVE EL TIMER2

		//LOAD PRESCALER
   // TIM2->PSC = 5000-1;
  TIM2->PSC = 1600-1; //ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS
		//LOAD ARR
	//TIM2->ARR = 2000-1;
  TIM2->ARR = 5000-1;   //SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM2->CNT = 0;
		//COUNTER ENABLE
	TIM2->CR1 |=(1<<0);

}

void timer6(){

	//TIMER 6 CONFIG 500ms
		//ENABLE TIMER2
	RCC->APB1ENR |=(1<<4);	//AQUI ES DONDE VIVE EL TIMER2

		//LOAD PRESCALER
//TIM6->PSC = 5000-1;
TIM6->PSC = 1600-1;   //ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS
		//LOAD ARR
	//TIM6->ARR = 1000-1;
    TIM6->ARR = 6900-1;   //SE RESTA UNO PARA QUE AVANCE LOS CICLOS

		//DIRECTION [0: (DEFAULT) UP COUNTER, 1: DOWN COUNTER]
	//TIM2->CR1 |=(1<<0);
		//COUNT TO 0
	TIM6->CNT = 0;
		//COUNTER ENABLE
	TIM6->CR1 |=(1<<0);
}

void timer21(){

	//TIMER 21 CONFIG 250ms
		//ENABLE TIMER21
	RCC->APB2ENR |=(1<<2);	//AQUI ES DONDE VIVE EL TIMER 21

		//LOAD PRESCALER
	//TIM21->PSC = 5000-1;
    TIM21->PSC = 1600-1; //ESTE VALOR ES DEL EJEMPLO VISTO EN CLASE PARA UP COUNTER, EL -1 SIRVE PARA PASAR DE CICLOS


		//LOAD ARR
    // TIM21->ARR = 140-1;
     TIM21->ARR = 5000-1; //SE RESTA UNO PARA QUE AVANCE LOS CICLOS
    // TIM21->CR1 &= ~(1<<4); //Default, upcounter
     	TIM21->CR1 |= 1<<4; //Downcounter
		//COUNT TO 0
	TIM21->CNT = 0;
		//COUNTER ENABLE
	TIM21->CR1 |=(1<<0);

}
uint8_t USART2_read(void)
{
    while(!(USART2->ISR & 0x0020)){}
    return USART2->RDR;
}

void USART2_write(uint8_t ch)
{
    while (!(USART2->ISR & 0x0080)){}
    USART2->TDR = ch;
}

void USART2_putstring(uint8_t* StringPtr)
{
    while(*StringPtr != 0x00)
    {
        USART2_write(*StringPtr);
        StringPtr++;
    }
}

void USART2_putstring_E(uint8_t* StringPtr)
{
    while(*StringPtr != 0x00)
    {
        USART2_write(*StringPtr);
        StringPtr++;
    }
    USART2_write(0x0D); // CR
    USART2_write(0x0A); // NL
}


void print_bcd_7_segment_decoder_CC(uint8_t val){




	if(val==0x00){						//ENCIENDE EL NUMERO 0 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<8;	//SEGMENTO A
		GPIOB->ODR |=0x01<<2;	//SEGMENTO B
		GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		GPIOB->ODR |=0x01<<4;	//SEGMENTO D
		GPIOB->ODR |=0x01<<6;	//SEGMENTO E
		GPIOB->ODR |=0x01<<7;	//SEGMENTO F
	}else if(val==0x01){				//ENCIENDE EL NUMERO 1 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<2;	//SEGMENTO B
		GPIOB->ODR |=0x01<<3;	//SEGMENTO C
	}else if(val==0x02){				//ENCIENDE EL NUMERO 2 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<8;	//SEGMENTO A
		GPIOB->ODR |=0x01<<2;	//SEGMENTO B
		GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		GPIOB->ODR |=0x01<<6;	//SEGMENTO E
		GPIOB->ODR |=0x01<<4;	//SEGMENTO D
	}else if(val==0x03){				//ENCIENDE EL NUMERO 3 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<8;	//SEGMENTO A
		GPIOB->ODR |=0x01<<2;	//SEGMENTO B
		GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		GPIOB->ODR |=0x01<<4;	//SEGMENTO D
	}else if(val==0x04){				//ENCIENDE EL NUMERO 4 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<7;	//SEGMENTO F
		GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		GPIOB->ODR |=0x01<<2;	//SEGMENTO B
		GPIOB->ODR |=0x01<<3;	//SEGMENTO C

	}else if(val==0x05){				//ENCIENDE EL NUEMRO 5 EN EL DISPLAY
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<8;	//SEGMENTO A
		GPIOB->ODR |=0x01<<7;	//SEGMENTO F
		GPIOB->ODR |=0x01<<9;	//SEGMENTO G
		GPIOB->ODR |=0x01<<3;	//SEGMENTO C
		GPIOB->ODR |=0x01<<4;	//SEGMENTO D

	}else if(val==0x0F){				//ENCIENDE LA LETRA F
		GPIOB->BSRR=(0x3DC<<16);	//RESETEA TODOS LOS BITS A 0
		GPIOB->ODR |=0x01<<8;	//SEGMENTO A
		GPIOB->ODR |=0x01<<6;	//SEGMENTO E
		GPIOB->ODR |=0x01<<7;	//SEGMENTO F
		GPIOB->ODR |=0x01<<9;	//SEGMENTO G

}
}
