#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include "i2cmaster.h"

#define FREQ_CPU  7372800 //cpu frequency
unsigned long USART_BAUDRATE = 19200; //bitstream rate
#define BAUD_PRESCALE (((FREQ_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define DEV_ACC 0x30 //accelerometer address in i2c bus
#define DEV_MAG 0x3C

#define CAL_N 10 //how many accelerometer reads will be used when calibrating device
void (*boot)( void ) = (void *)0x3C00;
typedef struct  xyz {
	signed long x;
	signed long y;
	signed long z;
} __attribute__ ((__packed__)) XYZ;

XYZ cal_vector = {0}; //Initial vector got after callibration
XYZ acc_vector = {0};
XYZ acc_vector_v = {0};

unsigned char cal_done=0;
unsigned short cal_n_done=0;

void init_usart(void) {
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0); 
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);

	UBRR0H = (unsigned char)(BAUD_PRESCALE>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALE;
}

void send_char(char data) {
    while ( (UCSR0A & (1 << UDRE0)) == 0) {};
      UDR0 = data;
}

void send_string(char* StringPtr) {
   while (*StringPtr) 
      send_char(*StringPtr++);
}

signed long get_signed(unsigned long d) {
	signed long sd;
	d&=0xFFFF;
	if(d>=32768) {
		sd = ( d^0xFFFF) * -1;
		return sd;
	}
	return d;
}

// Read accelerometer data
void update_acc_data() {
	signed long buf[3]={0};
	unsigned short tmp;
	memset(&acc_vector,0,sizeof(acc_vector));
	unsigned char *c, *z;
	for(char i=0;i<10;i++) {
		for(char j=0;j<3;j++) {
			z=&tmp;
			
			i2c_start_wait(0x30);
			i2c_write(0x28+j*2);
			i2c_rep_start(0x30+I2C_READ);
			z[0] = i2c_readNak();
			i2c_stop();
			
			i2c_start_wait(0x30);
			i2c_write(0x28+j*2+1);
			i2c_rep_start(0x30+I2C_READ);
			z[1] = i2c_readNak();
			i2c_stop();
			
			buf[j]+=get_signed(tmp);
		}		
	}
	
	for(char j=0;j<3;j++) buf[j]/=10;
	
	acc_vector.x = buf[0];
	acc_vector.y = buf[1];
	acc_vector.z = buf[2];	
}

// When calibrating, the accelerometer shouldn't be moving or rotating
void calibrate() {
	if(acc_vector.x==0 && acc_vector.y==0 && acc_vector.z==0) return;
	cal_vector.x += acc_vector.x;
	cal_vector.y += acc_vector.y;
	cal_vector.z += acc_vector.z;

	cal_n_done++;
	
	if(cal_n_done!=CAL_N) return;

	cal_vector.x /= CAL_N;
	cal_vector.y /= CAL_N;
	cal_vector.z /= CAL_N;
	
	cal_done=1;
}

void recalculate() {
	// subtracting the calibration vector from the current vector
	acc_vector.x -= cal_vector.x;
	acc_vector.y -= cal_vector.y;
	acc_vector.z -= cal_vector.z;

	acc_vector_v.x+=acc_vector.x;
	acc_vector_v.y+=acc_vector.y;
	acc_vector_v.z+=acc_vector.z;
}

ISR(TIMER1_COMPA_vect)  {
	char buf[32];

	update_acc_data();

	if(!cal_done) {
		calibrate();
		return;
	}
	
	recalculate();

	acc_vector_v.x=acc_vector_v.x*180/32768;
	acc_vector_v.y=acc_vector_v.y*180/32768;
	acc_vector_v.z=acc_vector_v.z*180/32768;
		
	send_string("A:");
	ltoa(acc_vector_v.x, buf, 10);
	send_string(buf);
	send_string(" ");
	ltoa(acc_vector_v.y, buf, 10);
	send_string(buf);
	send_string(" ");
	ltoa(acc_vector_v.z, buf, 10);
	send_string(buf);
	send_string("\r\n");
		
	memset(&acc_vector_v, 0, sizeof(acc_vector_v));
}

int main(void) {
	char buf[64];
	init_usart();
	for(int i=0;i<20000;i++);
	send_string("Starting..\r\n");
    i2c_init();
	for(int i=0;i<20000;i++);
	i2c_start_wait(0x30);

	i2c_write(0x20);
	i2c_write(0x27);
	i2c_stop();

	i2c_start_wait(0x30);
	i2c_write(0x23);
    i2c_rep_start(0x30+I2C_READ);
	send_string("Configuring accelerometer..\r\n");
	//Setting accelerometer parameters

	unsigned int ddd=i2c_readNak();
	itoa(ddd, buf, 10);
	send_string(buf);
	send_string("\r\n");
	i2c_stop();

	TCCR1B |= (1 << WGM12); // timer1 set to CTC mode
    TIMSK1 |= (1 << OCIE1A); // enabling CTC interrupt
    sei(); // activating all interrupts

	//timer overflow number = cpu freq / freq divider / required freq - 1
	OCR1A   = 1249;
    TCCR1B |= ((1 << CS10) | (1 << CS11)); // setting cpu freq divider

	while(1) {}

	return 0;
}