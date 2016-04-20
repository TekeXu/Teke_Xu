#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include <math.h>
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
#define PI 3.14159
void initSPI1(void);
void initI2C2(void);
void initExpander(void);
char SPI1_IO(char write);
void setVoltage(unsigned char channel, unsigned char voltage);
void makewave(void);
void i2c_master_start(void);              // send a START signal
void i2c_master_restart(void);            // send a RESTART signal
void i2c_master_send(unsigned char byte); // send a byte (either an address or data)
unsigned char i2c_master_recv(void);      // receive a byte of data
void i2c_master_ack(int val);             // send an ACK (0) or NACK (1)
void i2c_master_stop(void);               // send a stop
void setExpander(char pin, char level);
char getExpander(void);


//int value = 0;
int sinewave[100];
int triangle_wave[200];

int main() {
    //int value = 0;
    char r;
    makewave();
    int count1 = 0;
    int count2 = 0;
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
    TRISBbits.TRISB4 = 1;     // input

    LATAbits.LATA4 = 1;       // intialize LED on
    initSPI1();
    initI2C2();
    initExpander();
    
    __builtin_enable_interrupts();
    
    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		// remember the core timer runs at half the CPU speed
        _CP0_SET_COUNT(0);                   // set core timer to 0
        while(_CP0_GET_COUNT() < 24000){     // wait 1ms / 0.001s
            ;
        }
        while(_CP0_GET_COUNT() < 24000){     // wait 1ms / 0.001s
            ;
        }
        while(_CP0_GET_COUNT() < 24000){     // wait 1ms / 0.001s
            ;
        }
        while(_CP0_GET_COUNT() < 24000){     // wait 1ms / 0.001s
            ;
        }
        setVoltage(0,sinewave[count1]);
        setVoltage(1,triangle_wave[count2]);
        count1++;
        count2++;
        if(count1 == 100){
          count1 = 0;
        }
        if(count2 == 200){
          count2 = 0;
        }
        r = getExpander();
        setExpander(0,r);

    }// while loop
        //CS = 0;                                 // listen to me
        //SPI1_IO(0x38); // most significant byte of address
        //SPI1_IO(0x00);         // the least significant address byte
        //CS = 1;   
    
}


void initSPI1(void){
  // set up the chip select pin as an output
  // the chip select pin is used by the sram to indicate
  // when a command is beginning (clear CS to low) and when it
  // is ending (set CS high)
  //TRISBbits.TRISB14 = 0;    // output master clock
  TRISBbits.TRISB7 = 0;     // set RB7 output : chip selection
  TRISBbits.TRISB8 = 0;     // set RB8 output : SDO1
                            // set RB9 output : SDI1(no need)
  CS = 1;                   // disabled the slave
  RPB8Rbits.RPB8R = 0b0011; // assign SDO1 to PIN 17

  // Master - SPI4, pins are: SDI4(F4), SDO4(F5), SCK4(F13).  
  // we manually control SS4 as a digital output (F12)
  // since the pic is just starting, we know that spi is off. We rely on defaults here
 
  // setup spi4
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x3;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 4

}


void initI2C2(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    I2C2BRG = 233;          //some number for 100kHz;            // I2CBRG = [1/(2*Fsck) - PGD]*Pblck - 2 
                                    // look up PGD for your PIC32
                                  // PGD = 104ns, Fsck = 100kHz, Pblck = 48MHz.
    I2C2CONbits.ON = 1;               // turn on the I2C2 module
}

char SPI1_IO(char write){
    SPI1BUF = write;
    while(!SPI1STATbits.SPIRBF){
        ;
    }
    return SPI1BUF;
}

void initExpander(void){    // GP0-3 outputs and GP4-7 inputs
  i2c_master_start();
  i2c_master_send(0x20 << 1 | 0);   // GPIO address & indicate write
  i2c_master_send(0x00);   // addr of I/O direction register
  i2c_master_send(0xF0);   // send the value to the register. 0-3outputs 4-7 inputs
  i2c_master_stop();
}

void setVoltage(unsigned char channel, unsigned char voltage){    //channel 0 for voutA, 1 for voutB
    unsigned short value = 0;
    if(channel == 0){
      value = (0b0011 << 12) + (voltage << 4);
        CS = 0;                                 // listen to me
        SPI1_IO((value & 0xFF00) >> 8 ); // most significant byte of address
        SPI1_IO(value & 0x00FF);         // the least significant address byte
        CS = 1;                          // end
    }
    if (channel == 1){
      value = (0b1011 << 12) + (voltage << 4);
        CS = 0;
        SPI1_IO((value & 0xFF00) >> 8 ); // most significant byte of address
        SPI1_IO(value & 0x00FF);         // the least significant address byte
        CS = 1;
    }
}

char getExpander(void){ // read from GP7
  i2c_master_start();
  i2c_master_send(0x20 << 1 | 0);
  i2c_master_send(0x09);  // the register to read from (GPIO)
  i2c_master_restart();   // make the restart bit, so we can begin reading
  i2c_master_send(0x20 << 1 | 1); // indicate reading
  unsigned char read = i2c_master_recv() >> 7;     // save the value returned. the value of GP7
  i2c_master_ack(1); // make the ack so the slave knows we got it
  i2c_master_stop(); // make the stop bit
  return read;
}

void setExpander(char pin, char level){ // set GP0
  i2c_master_start();
  i2c_master_send(0x20 << 1 | 0);   // GPIO address & indicate write
  i2c_master_send(0x0A);   // addr of OLAT register
  if(level == 1){
    i2c_master_send(0x01);   // send '1' to GP0, indicating high level.
  }
  if(level == 0){
    i2c_master_send(0x00);
  }
  i2c_master_stop();
}

void makewave(void){
  int i;
  for(i = 0; i < 100; i++){
    sinewave[i] = (int)(128.0 + 127.5 * sin(PI * 0.02 * i));
  }
  for(i = 0; i < 200; i++){
    triangle_wave[i] = (int)(1.28 * i);
  }
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

