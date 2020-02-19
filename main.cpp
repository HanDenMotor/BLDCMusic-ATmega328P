/*
 * stepVVVF-1.cpp
 *
 * Created: 2017/11/21 21:10:10
 * stepVVVF Serial_system
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 24000000 //CPU�N���b�N16Mhz
#include <util/delay.h>
#include <stdlib.h>

#define IDA 0x04//������ID
#define IDB 0x05//������ID
#define DATA 3 //�f�[�^�� ID������

#define SPEED 38400//�V���A���ʐM�̑��x


void setSquareWave(int freq,uint8_t velo,uint8_t mt); //��`�g�̎��g���ݒ�
void serialRead();



volatile uint8_t vect = 0;//�p�x
volatile uint8_t duty_h = 0;//duty125�ȏ�

volatile uint8_t flag = 0;//�������Ẵf�[�^���ǂ�����flag
volatile uint8_t bit = 0;//�ǂݍ��ݒ���bit
volatile int freqencyA = 0,freqencyB = 0;
volatile uint8_t velocityA = 0, velocityB = 0;//��]���x
volatile uint8_t serialBuf[DATA] = {};//�V���A���f�[�^�̃o�b�t�@�[
volatile uint8_t serial_MT = 0;//�V���A���f�[�^����M�������[�^
volatile uint8_t serial_flag = 0;//�V���A���f�[�^����M�������[�^

volatile double duty = 0;


uint8_t BL1_pulseON_B[6] = {0x38,0x13,0x23,0x34,0x15,0x29};//U��V,W��V,W��U,V��U,V��W,U��W


//uint8_t BL1_pulseOFF_B[6] = {0x18,0x12,0x22,0x24,0x05,0x09};//U��V,W��V,W��U,V��U,V��W,U��W
uint8_t BL1_pulseOFF_B[6] = {0x30,0x11,0x21,0x30,0x11,0x21};//U��V,W��V,W��U,V��U,V��W,U��W

	
uint8_t BL2_pulseON_D[6] = {0x1C,0xD0,0xC4,0x34,0xB0,0x8C};//U��V,W��V,W��U,V��U,V��W,U��W


//uint8_t BL2_pulseOFF_D[6] = {0x18,0x50,0x44,0x24,0xA0,0x88};//U��V,W��V,W��U,V��U,V��W,U��W
uint8_t BL2_pulseOFF_D[6] = {0x14,0x90,0x84,0x14,0x90,0x84};//U��V,W��V,W��U,V��U,V��W,U��W

	
volatile uint8_t state = 0,MT1_ON = 0;
volatile uint8_t state2 = 0,MT2_ON = 0;
volatile int stop_cnt = 0;
volatile int stop_cnt2 = 0;
uint8_t  stop_flag = 0,state_ps = 0;
uint8_t  stop_flag2 = 0,state2_ps = 0;


#define DUTY_NUM 4
#define FREQ_NUM 2

int main(void)
{
	 DDRD = 0xFC;//PWM�s�����o�͐ݒ�
	 DDRB = 0x3F;
	 DDRC = 0x18;
	 
	

	 //��`�g�����^�C�}�[�ݒ�
	 TCCR0A = 0x01; //PWM�o�͒�~�@����PWM OCR0A max
	 TCCR0B = 0x0C; // 256����
	 
	 
	 TCCR2A = 0x01; //PWM�o�͒�~�@����PWM OCR2A max
	 TCCR2B = 0x0E; // 256����
	 
	 //PWM����Timer1
	 TCCR1A = 0x03;//10bit����PWM
	 TCCR1B = 0x09;//�����Ȃ�
	 
	 
	 //�V���A���ʐM�ݒ�
	UBRR0 =  ((long)F_CPU/((long)SPEED * 16)) - 1;

	
	UCSR0A = 0x80;
	UCSR0B = 0x98;
	UCSR0C = 0x06;
	
	sei();
	
	
	
    /* Replace with your application code */
    while (1) 
    {
		//if(UCSR0A & 0x80) PORTB = 0x02;
		if(serial_flag){
			if(serial_MT == 1){//�V���A���f�[�^��M��
				
				/*int duty = 5;
				if(freqencyA > 4200) freqencyA /= 8;
				else if(freqencyA > 2100) freqencyA /= 4;
				else if(freqencyA > 1050) freqencyA /= 2;
				
				duty = freqencyA / 45 + 12;
				if(duty > 65) duty = 65;
				
				duty = 255 - duty;*/
				
				if(velocityA > 60) velocityA = 60;
				velocityA = 255 - velocityA;
				
				stop_cnt = 0;
				stop_flag = 0;
				
				setSquareWave(freqencyA / FREQ_NUM,velocityA ,1);
				serial_MT = 0;//�t���O��߂�
			}else if(serial_MT == 2){//�V���A���f�[�^��M��
			
				/*int duty = 5;
				if(freqencyB > 4200) freqencyB /= 8;
				else if(freqencyB > 2100) freqencyB /= 4;
				else if(freqencyB > 1050) freqencyB /= 2;
				duty = freqencyB / 45 + 12 ;
				if(duty > 65) duty = 65;*/
				
				if(velocityB > 60) velocityB = 60;
				velocityB = 255 - velocityB;
				
				duty = 255 - duty;
				stop_cnt2 = 0;
				stop_flag2 = 0;

				setSquareWave(freqencyB / FREQ_NUM,velocityB ,2);
				serial_MT = 0;//�t���O��߂�
			}
			serial_flag = 0;
		}
		
		if(stop_flag){
			if(state_ps != state){
				stop_cnt = 0;
				stop_flag = 0;
				TIMSK0 = 0x00;//���荞�ݖ���
			}
			//state_ps = state;
			
		}
		
		if(stop_flag2){
			if(state_ps != state){
				stop_cnt2 = 0;
				stop_flag2 = 0;
				TIMSK2 = 0x00;//���荞�ݖ���
			}
			
			//state2_ps = state2;
		}
		
    }
}

void setSquareWave(int freq,uint8_t velo,uint8_t mt) {
  /*�}�C�R����16Mhz 64���� ���@250kHz
     �񓯊���duty99.9%
     �����͍ő�duty99%
     �񓯊���PWM�؂�ւ���20��
  */
  
   duty = velo;//128�i�K��256�i�K�ɕύX

	
	if (freq != 0) { //����
	
		if(mt == 1){
			TIMSK0 = 0x00;//���荞�ݒ�~
			PORTB = BL1_pulseOFF_B[state];//�o��L
			PORTC |= 0x10;
			MT1_ON = 1;
			
			if((freq > 188) && (freq <= 750)){
				TCCR0B = 0x0C;//256����
				OCR0A = (unsigned int)(47058 / freq) ; //���荞�ݎ��g��
				OCR1A = velo * DUTY_NUM;
				TIMSK0 = 0x01;//���荞�� ���
				TIMSK1 |= 0x03;//PWM���荞�݁@����CompA
			}else if((freq > 48) && (freq <= 188)){
				TCCR0B = 0x0D;//1024����
				OCR0A = (unsigned int)(11730 / freq) ; //���荞�ݎ��g��
				OCR1A = velo * DUTY_NUM;
				TIMSK0 = 0x01;//���荞�� ���
				TIMSK1 |= 0x03;//PWM���荞�݁@����CompA
			}else if((freq > 750) && (freq <= 2000)){
				TCCR0B = 0x0B;//64����
				OCR0A = (unsigned int)(190476 / freq) ; //���荞�ݎ��g��
				OCR1A = velo * DUTY_NUM;
				TIMSK0 = 0x01;//���荞�� ���
				TIMSK1 |= 0x03;//PWM���荞�݁@����CompA
			}
			
		}else if(mt == 2){
			TIMSK2 = 0x00;//���荞�ݒ�~
			PORTD =  BL2_pulseOFF_D[state2];
			PORTC |= 0x08;
			MT2_ON = 1;
			
			if((freq > 188) && (freq <= 750)){
				TCCR2B = 0x0E;//256����
				OCR2A = (unsigned int)(47058 / freq) ; //���荞�ݎ��g��
				OCR1B = velo * DUTY_NUM;
				TIMSK2 = 0x01;//���荞�� ���
				TIMSK1 |= 0x05;//PWM���荞�݁@����CompB
			}else if((freq > 48) && (freq <= 188)){
				TCCR2B = 0x0F;//1024����
				OCR2A = (unsigned int)(11730 / freq) ; //���荞�ݎ��g��
				OCR1B = velo * DUTY_NUM;
				TIMSK2 = 0x01;//���荞�� ���
				TIMSK1 |= 0x05;//PWM���荞�݁@����CompB
			}else if((freq > 750) && (freq <= 2000)){
				TCCR2B = 0x0C;//64����
				OCR2A = (unsigned int)(190476 / freq) ; //���荞�ݎ��g��
				OCR1B = velo * DUTY_NUM;
				TIMSK2 = 0x01;//���荞�� ���
				TIMSK1 |= 0x05;//PWM���荞�݁@����CompB
			}
			
		}	
					

	 } else  { //��~
		if(mt == 1){
			TIMSK0 = 0x01;//OVF���荞��
			stop_flag = 1;
			PORTC &= ~0x10;
			PORTB = BL1_pulseOFF_B[state];//�o��L
			MT1_ON = 0;
			TIMSK1 &= ~0x02;
			
			state_ps = state;//���݊p�x
		 }else if(mt == 2){
			TIMSK2 = 0x01;//OVF���荞��
			stop_flag2 = 1;
			PORTC &= ~0x08;
			PORTD = BL2_pulseOFF_D[state2];
			MT2_ON = 0;
			TIMSK1 &= ~0x04;
			
			state2_ps = state2;
		}
	}
}


ISR(TIMER0_OVF_vect){//�^�C�}�[0�@�i�p
	PORTB = BL1_pulseOFF_B[state];
	PORTD = BL2_pulseOFF_D[state2];
	sei();
	state++;
	if(state == 6){
		state = 0;
	}
}


ISR(TIMER2_OVF_vect){//�^�C�}�[2�@�i�p
	PORTB = BL1_pulseOFF_B[state];
	PORTD = BL2_pulseOFF_D[state2];
	sei();
	state2++;
	if(state2 == 6){
		state2 = 0;
	}
}

ISR(TIMER1_OVF_vect){//PWMON
	//sei();
	PORTB = BL1_pulseOFF_B[state];
	PORTD = BL2_pulseOFF_D[state2];
}
ISR(TIMER1_COMPA_vect){
	//sei();
	//MT1
	//PORTB &= ~0x3F;//�o��OFF
	PORTB = BL1_pulseON_B[state];//�o��
	
}


ISR(TIMER1_COMPB_vect){
	//sei();
	//MT2
	//PORTD &= ~0xFC;//�o��OFF
	PORTD = BL2_pulseON_D[state2];
	
}



ISR(USART_RX_vect){//�V���A����M������
	
	uint8_t data = 0; //��M�f�[�^
	data = UDR0;
	
	PORTB = BL1_pulseOFF_B[state];
	PORTD = BL2_pulseOFF_D[state2];
	sei();


	if(data & 0x80){//�ŏ��bit��1�̎�ID�w��
		
		
		if((data & 0x7F) == IDA) {
			
			flag = 1;
			serial_MT = 1;//�V���A����M�ς�
		}
		else if((data & 0x7F) == IDB) {
			
			flag = 1;
			serial_MT = 2;
		}
		else flag = 0;
		bit = 0;//bit reset
		
	}else if(flag){//�ŏ��bit��0�Ŏ������Ă�ID�̎�
		serialBuf[bit] = data;
		bit++;
		
		if(bit == DATA){//��M�ς݂��f�[�^���Ɠ����ɂȂ�� 

			 flag = 0;//�t���O��߂�
			 if(serial_MT == 1){
				 freqencyA = (serialBuf[0] & 0x7F) << 7;//���g���ɕϊ�
				 freqencyA += serialBuf[1] & 0x7F;
				 velocityA = (serialBuf[2] & 0x7F) * 2;//���̋��x 128�i�K��256�i�K
			 }else if(serial_MT == 2){
				 freqencyB = (serialBuf[0] & 0x7F) << 7;//���g���ɕϊ�
				 freqencyB += serialBuf[1] & 0x7F;
				 velocityB = (serialBuf[2] & 0x7F) * 2;//���̋��x
			 }
			serial_flag = 1;//��M����On
				

		}
	}
}





