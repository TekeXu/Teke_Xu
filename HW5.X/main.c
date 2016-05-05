#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "ILI9163C.h"
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0xFFFF // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7       // chip select pin
//#define PI 3.14159
//void initSPI1(void);
void initI2C2(void);
//char SPI1_IO(char write);
//void setVoltage(unsigned char channel, unsigned char voltage);
//void makewave(void);
void i2c_master_start(void);              // send a START signal
void i2c_master_restart(void);            // send a RESTART signal
void i2c_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c_master_recv(void);      // receive a byte of data
void i2c_master_ack(int val);             // send an ACK (0) or NACK (1)
void i2c_master_stop(void);               // send a stop
void init_ctrl1(void);
void init_ctrl2(void);
void init_ctrl3(void);
//void init_OC(void);
void I2C_read_multiple(char address, char reg_addr, unsigned char * data, char length);
void display_character(unsigned short x, unsigned short y, unsigned short color, char c);



//int value = 0;
//int sinewave[100];
//int triangle_wave[200];
int j,ii,jj;
unsigned char data_array[14];
unsigned short temp, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;

int main() {
    //int value = 0;
    //char r;
    //makewave();
    //int count1 = 0;
    //int count2 = 0;
    char message[100];
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;     // ouput
    //TRISBbits.TRISB4 = 1;     // input
    LATAbits.LATA4 = 1;       // intialize LED on

    SPI1_init();
    LCD_init();
    initI2C2();
    init_ctrl1();
    init_ctrl2();
    init_ctrl3();

    __builtin_enable_interrupts();
    // LCD test here
    LCD_clearScreen(WHITE);
    sprintf(message, "Hello World 1337!");
    int k = 0;
    while(message[k]){
      display_character(28 + k * 5,32,BLACK,message[k]); 
      k++;
    }


    //display_character(28,32,BLACK,'H');
    //display_character(33,32,BLACK,'e');
    //display_character(38,32,BLACK,'l');
    //display_character(43,32,BLACK,'l');
    //display_character(48,32,BLACK,'o');





    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		// remember the core timer runs at half the CPU speed
        /*_CP0_SET_COUNT(0);                   // set core timer to 0
        while(_CP0_GET_COUNT() < 480000){     // wait 1ms / 0.001s
            ;
        }*/
        ;
        //I2C_read_multiple(0x6B, 0x20, data_array, 14);

    }
    
}
// {0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
void display_character(unsigned short x, unsigned short y, unsigned short color, char c){
  char temp;
  char c_ascii;
  c_ascii = c - 0x20;
  for(ii = 0; ii < 5; ii++){
     for(jj = 0;jj < 8; jj++){
       temp = ASCII[c_ascii][ii];  // temp = 0x7f = 01111111, 00001000, 00001000, 00001000, 01111111
       if((temp >> (7 - jj)) & 1){
        LCD_drawPixel(x + ii, y + (7-jj), color);
       }

     }
  }

}




void initI2C2(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    I2C2BRG = 233;          //some number for 100kHz;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
                                  // PGD = 104ns, Fsck = 100kHz, Pblck = 48MHz.
    I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

void init_ctrl1(void){
  i2c_master_start();
  i2c_master_send(0x6B << 1 | 0);   // chip address & indicate write
  i2c_master_send(0x10);   // addr of ctrl_1 register
  i2c_master_send(0x80);   // send the value to the register, enable accelerometer
  i2c_master_stop();
}

void init_ctrl2(void){
  i2c_master_start();
  i2c_master_send(0x6B << 1 | 0);   // chip address & indicate write
  i2c_master_send(0x11);   // addr of ctrl_2 register
  i2c_master_send(0x80);   // send the value to the register, enable agyroscope
  i2c_master_stop();
}

void init_ctrl3(void){
  i2c_master_start();
  i2c_master_send(0x6B << 1 | 0);   // chip address & indicate write
  i2c_master_send(0x12);   // addr of ctrl_3 register
  i2c_master_send(0x04);   // send the value to the register, enable if_inc
  i2c_master_stop();
}

void I2C_read_multiple(char address, char reg_addr, unsigned char * data, char length){
  i2c_master_start();
  i2c_master_send(address << 1 | 0);   // chip address & indicate write
  i2c_master_send(reg_addr);   // addr of OUT_TEMP_L register
  i2c_master_restart();   // make the restart bit, so we can begin reading
  i2c_master_send(address << 1 | 1);   // chip address & indicate reading
  for(j = 0; j < length - 1; j++){
    *(data + j) = i2c_master_recv();
    i2c_master_ack(0); // make the ack so the slave knows we got it
  }
  *(data + length) = i2c_master_recv();
  i2c_master_ack(1);
  i2c_master_stop(); // make the stop bit
  temp = data_array[0] << 8 | data_array[1];
  gyroX = data_array[2] << 8 | data_array[3];
  gyroY = data_array[4] << 8 | data_array[5];
  gyroZ = data_array[6] << 8 | data_array[7];
  accelX = data_array[8] << 8 | data_array[9];
  accelY = data_array[10] << 8 | data_array[11];
  accelZ = data_array[12] << 8 | data_array[13];
}







// Start a transmission on the I2C bus
void i2c_master_start(void) {
    I2C2CONbits.SEN = 1;            // send the start bit
    while(I2C2CONbits.SEN) { ; }    // wait for the start bit to be sent
}

void i2c_master_restart(void) {     
    I2C2CONbits.RSEN = 1;           // send a restart 
    while(I2C2CONbits.RSEN) { ; }   // wait for the restart to clear
}

void i2c_master_send(unsigned char byte) { // send a byte to slave
  I2C2TRN = byte;                   // if an address, bit 0 = 0 for write, 1 for read
  while(I2C2STATbits.TRSTAT) { ; }  // wait for the transmission to finish
  if(I2C2STATbits.ACKSTAT) {        // if this is high, slave has not acknowledged
    // ("I2C2 Master: failed to receive ACK\r\n");
  }
}

unsigned char i2c_master_recv(void) { // receive a byte from the slave
    I2C2CONbits.RCEN = 1;             // start receiving data
    while(!I2C2STATbits.RBF) { ; }    // wait to receive the data
    return I2C2RCV;                   // read and return the data
}

void i2c_master_ack(int val) {        // sends ACK = 0 (slave should send another byte)
                                      // or NACK = 1 (no more bytes requested from slave)
    I2C2CONbits.ACKDT = val;          // store ACK/NACK in ACKDT
    I2C2CONbits.ACKEN = 1;            // send ACKDT
    while(I2C2CONbits.ACKEN) { ; }    // wait for ACK/NACK to be sent
}

void i2c_master_stop(void) {          // send a STOP:
  I2C2CONbits.PEN = 1;                // comm is complete and master relinquishes bus
  while(I2C2CONbits.PEN) { ; }        // wait for STOP to complete
}



