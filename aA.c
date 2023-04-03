<------ LED SCREEN NAME PRINT ---->

#pragma config FOSC = HS 
#pragma config WDTE = OFF 
#pragma config PWRTE = OFF
#pragma config BOREN = ON 
#pragma config LVP = ON 
#pragma config CPD = OFF 
#pragma config WRT = OFF 
#pragma config CP = OFF 

#include <xc.h>
#define _XTAL_FREQ 20000000

#define RS RD0 //D
#define RW RD1 //D
#define EN RD2 //D

void lcd_data(unsigned char data){
	 PORTC = data;
	 RS = 1;
	 RW = 0;
	 EN = 1;
	 __delay_ms(100);
	 EN = 0;
}
void lcd_command(unsigned char cmd){
	 PORTC = cmd;
	 RS = 0;
	 RW = 0;
	 EN = 1;
	 __delay_ms(100);
	 EN = 0;
}
void lcd_string(const unsigned char *str, unsigned char num){
	unsigned char i;
	for(i=0; i<num; i++) {
	lcd_data(str[i]);
	}
}
void lcd_init(){
	 lcd_command(0x38);
	 lcd_command(0x06);
	 lcd_command(0x0c);
	 lcd_command(0x01);
}
void main(void) {
	 TRISC = 0x00;
	 TRISD0 = 0;
	 TRISD1 = 0;
	 TRISD2 = 0;

	 lcd_init();

 while(1){
	 lcd_command(0x83);
	 lcd_string("ICT000",6);
	 lcd_command(0xc3);
	 lcd_string("********",8);
	 __delay_ms(100);
	 lcd_command(0x01);
 }
	return;
}


<------ Motion detection using PIR Motion Sensor ---->

#include <xc.h>
#define _XTAL_FREQ 20000000
#pragma config FOSC = HS 
#pragma config WDTE = OFF 
#pragma config PWRTE = OFF 
#pragma config BOREN = ON 
#pragma config LVP = ON 
#pragma config CPD = OFF 
#pragma config WRT = OFF
#pragma config CP = OFF

#define PIR RD5
#define buzzer RB2
#define led RB3
#define RS RD0
#define RW RD1
#define EN RD2

void lcd_data(unsigned char data){
	 PORTC = data;
	 RS = 1;
	 RW = 0;
	 EN = 1;
	 __delay_ms(100);
	 EN = 0;
}
void lcd_command(unsigned char cmd){
	 PORTC = cmd;
	 RS = 0;
	 RW = 0;
	 EN = 1;
	 __delay_ms(100);
	 EN = 0;
}
void lcd_string(const unsigned char *str, unsigned char num){
	unsigned char i;
		for(i=0; i<num; i++)
		{
		lcd_data(str[i]);
	}
}
void lcd_init(){
	 lcd_command(0x38);
	 lcd_command(0x06);
	 lcd_command(0x0c);
	 lcd_command(0x01);
}
void main(void) {
	 TRISB2 = 0;
	 TRISB3 = 0;
	 TRISD5 = 1;
	 TRISC = 0x00;
	 TRISD0 = 0;
	 TRISD1 = 0;
	 TRISD2 = 0;

	 lcd_init();

	 while(1)
	 {
	 if(PIR==1)
	 {
	 buzzer = 1;
	 led = 1;

	 lcd_command(0x83);
	 lcd_string("Motion",8);
	 lcd_command(0xc3);
	 lcd_string("Detected",8);
	 __delay_ms(100);
	 lcd_command(0x01);
	 }
	 else
	 {
	 buzzer = 0;
	 led = 0;

	 lcd_command(0x83);
	 lcd_string("Motion",8);
	 lcd_command(0xc3);
	 lcd_string("NotDetected",11);
	 __delay_ms(100);
	 lcd_command(0x01);
	 }
 }
return;
}

******COMPONATS******

BUZZER
LED-ORANGE
LM016L
LOGICTOGGLR

disply
RS == RD0
RW == RD1
EN == RD2

D0 == RC0
''
D7 == RC7

RB2 --> BUZZER GROUND
RB3 --> LED GROUND
RD5 --> PIR SENSOR O

PIR SENSOR
V == POWER
G == GROUND
TestPin == LOGICTOGGLE



<------ Ultrasonic Sensor ---->

#define _XTAL_FREQ 20000000
#define RS RD2
#define EN RD3
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7
#define Trigger RB1 //34 is Trigger
#define Echo RB2//35 is Echo 

#include <xc.h>

#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF        // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


void Lcd_SetBit(char data_bit) //Based on the Hex value Set the Bits of the Data Lines
{

if(data_bit& 1) 
D4 = 1;
else
D4 = 0;

if(data_bit& 2)

D5 = 1;

else

D5 = 0;
if(data_bit& 4)
D6 = 1;
else
D6 = 0;
if(data_bit& 8) 
D7 = 1;
else
D7 = 0;
}

 

void Lcd_Cmd(char a)

{
RS = 0;           
Lcd_SetBit(a); //Incoming Hex value
EN  = 1;         
        __delay_ms(4);
        EN  = 0;         
}

 

void Lcd_Clear()
{
Lcd_Cmd(0); //Clear the LCD
Lcd_Cmd(1); //Move the curser to first position
}

 

void Lcd_Set_Cursor(char a, char b)

{

char temp,z,y;

if(a== 1)

{
 temp = 0x80 + b - 1; //80H is used to move the curser
z = temp>>4; //Lower 8-bits
y = temp & 0x0F; //Upper 8-bits
Lcd_Cmd(z); //Set Row
Lcd_Cmd(y); //Set Column
}

else if(a== 2)

{
temp = 0xC0 + b - 1;
z = temp>>4; //Lower 8-bits
y = temp & 0x0F; //Upper 8-bits
Lcd_Cmd(z); //Set Row
Lcd_Cmd(y); //Set Column
}

}

 

void Lcd_Start()

{
  Lcd_SetBit(0x00);
  for(int i=1065244; i<=0; i--)  NOP();  
  Lcd_Cmd(0x03);
__delay_ms(5);
  Lcd_Cmd(0x03);
__delay_ms(11);
  Lcd_Cmd(0x03); 
  Lcd_Cmd(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  Lcd_Cmd(0x02); //02H is used for Return home -> Clears the RAM and initializes the LCD
  Lcd_Cmd(0x08); //Select Row 1
  Lcd_Cmd(0x00); //Clear Row 1 Display
  Lcd_Cmd(0x0C); //Select Row 2
  Lcd_Cmd(0x00); //Clear Row 2 Display
  Lcd_Cmd(0x06);

}

 

void Lcd_Print_Char(char data){
   char Lower_Nibble,Upper_Nibble;
   Lower_Nibble = data&0x0F;
   Upper_Nibble = data&0xF0;
   RS = 1;             // => RS = 1
   Lcd_SetBit(Upper_Nibble>>4);             //Send upper half by shifting by 4
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP(); 
   EN = 0;
   Lcd_SetBit(Lower_Nibble); //Send Lower half
   EN = 1;
   for(int i=2130483; i<=0; i--)  NOP();
   EN = 0;

}

 

void Lcd_Print_String(char *a){

int i;

for(i=0;a[i]!='\0';i++)
  Lcd_Print_Char(a[i]);  //Split the string using pointers and call the Char function 
}
int time_taken;
int distance;
char t1,t2,t3,t4,t5;
char d1,d2,d3;

int main()

{

    TRISD = 0x00; //PORTD declared as output for interfacing LCD

    TRISB0 = 1;        //DEfine the RB0 pin as input to use as interrupt pin
    TRISB1 = 0; //Trigger pin of US sensor is sent as output pin
    TRISB2 = 1; //Echo pin of US sensor is set as input pin       
    TRISB3 = 0; //RB3 is output pin for LED
    
    T1CON=0x20;

    Lcd_Start();

    while(1){ 
        TMR1H =0; TMR1L =0; //clear the timer bits

        Trigger = 1; 
        __delay_us(10);           
        Trigger = 0;  
        
        while (Echo==0);
            TMR1ON = 1;
        while (Echo==1);
            TMR1ON = 0;

        time_taken = (TMR1L | (TMR1H<<8)); 
        distance= (0.0272*time_taken)/2;

        time_taken = time_taken * 0.8;
        t1 = (time_taken/1000)%10;
        t2 = (time_taken/1000)%10;
        t3 = (time_taken/100)%10;
        t4 = (time_taken/10)%10;
        t5 = (time_taken/1)%10;
        d1 = (distance/100)%10;
        d2 = (distance/10)%10;
        d3 = (distance/1)%10;

        Lcd_Set_Cursor(1,1);
        Lcd_Print_String("Time_taken:");
        Lcd_Print_Char(t1+'0');
        Lcd_Print_Char(t2+'0');
        Lcd_Print_Char(t3+'0');
        Lcd_Print_Char(t4+'0');
        Lcd_Print_Char(t5+'0');
        Lcd_Set_Cursor(2,1);
        Lcd_Print_String("distance:");
        Lcd_Print_Char(d1+'0');
        Lcd_Print_Char(d2+'0');
        Lcd_Print_Char(d3+'0');      
    }

    return 0;

}


******COMPONATS******
PIC16F877
HCSR04
LM016L
POT

disply
RS == RD2
EN == RD3

D4 == RC4
''
D7 == RC7

HCSR04
GND == GROUND 
TR == RB1
ECHO == RB2
VCC == POWER

<------ trafic light ------>

#include <xc.h>
#define _XTAL_FREQ 20000000
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

void main(void) {
    
    TRISB0 = 0;
    TRISB1 = 0;
    TRISB2 = 0;
    
    while(1){
        PORTBbits.RB0=1;
        __delay_ms(500);
        PORTBbits.RB0=0;
        __delay_ms(500);
        PORTBbits.RB1=1;
        __delay_ms(500);
        PORTBbits.RB1=0;
        __delay_ms(500);
        PORTBbits.RB2=1;
        __delay_ms(500);
        PORTBbits.RB2=0;
        __delay_ms(500);
    }
    return;
}

******COMPONATS******

led 3 RB0-RB3


<------ 7 segment multyplexing ------>


#include <xc.h>
#define _XTAL_FREQ 20000000

// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

unsigned char segment[]= {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x6f};

int i;
int j;
void main(void) {
    TRISB=0x00;
    TRISC=0x00;
    PORTB=0x00;
    PORTC=0x00;
    
    for(i=1; i<=10; i++){
        for(j=0; j<10; j++){
            PORTB = segment[j];
            __delay_ms(500);
        } PORTC= segment[i]; 
    }
        
    return;
}

******COMPONATS******
7SEG-MPX1-CC
PIC16F877A

RC0 == LED FIRST
''
RC7 == LED LAST

RB0 == LED FIRST
''
RB7 == LED LAST



<------ Simulate a circuit with 8 LEDs ------>

#include <xc.h>
#define _XTAL_FREQ 8000000

#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON // Low-Voltage (Single-Supply) In-Circuit Serial Programming
Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data EEPROM
code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write protection off;
all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code protection off)
void main(void) {
 TRISC = 0;
 while(1){
 PORTC = 0x01;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x02;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x04;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x08;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x10;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x20;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x40;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);

 PORTC = 0x80;
 __delay_ms(1000);
 PORTC = 0x00;
 __delay_ms(100);
 }
 return;
}

******COMPONATS******
LED BLUE 1 == RC0
''
LRD BLUR 8 == 

<------ push Buttont ------>

#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON // Low-Voltage (Single-Supply) In-Circuit Serial
Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming
enabled)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data
EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write
protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code
protection off)
#include <xc.h>
#define _XTAL_FREQ 20000000
void main(void) {
 TRISB2 = 1;
 TRISC1 = 0;

 if(PORTBbits.RB2==1){
 PORTCbits.RC1=1;
 }
 else{
 PORTCbits.RC1=0;
 }
 return;

******COMPONATS******
led == RC1
BUTTON == RB2



<------ Stepper Motor ------>

#pragma config FOSC = HS // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = ON // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = ON // Low-Voltage (Single-Supply) In-Circuit Serial
Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming
enabled)
#pragma config CPD = OFF // Data EEPROM Memory Code Protection bit (Data
EEPROM code protection off)
#pragma config WRT = OFF // Flash Program Memory Write Enable bits (Write
protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF // Flash Program Memory Code Protection bit (Code
protection off)
#include <xc.h>
#define _XTAL_FREQ 80000000
void rotate_motor() {

 PORTC = 0b00000001;
 __delay_ms(1000);

 PORTC = 0b00000010;
 __delay_ms(1000);

 PORTC = 0b00000011;
 __delay_ms(1000);

 PORTC = 0b00000100;
 __delay_ms(1000);

 PORTC = 0b00000101;
 __delay_ms(1000);

 PORTC = 0b00000110;
 __delay_ms(1000);

 PORTC = 0b00000111;
 __delay_ms(1000);

 PORTC = 0b00001000;
 __delay_ms(1000);
}

void main(void){
 TRISC = 0x00;
 PORTC = 0x00;

 while(1){
 rotate_motor();
 }
 return;
}

******COMPONATS******
MOTER-STEPPE
16F877A
ULN2824

MOTER-STEPPE
ULN2824 1C || 3C
ULN2824 POWER || POWER
ULN2824 2C || 4C

U2
RC0 == B1 
''
RC7 == B8

<------ temparage senser ------>

#include <xc.h>
#include "config.h"
#include <stdint.h>
#include <stdio.h>
//=================================
//--------[ Definitiotns ]---------
#define _XTAL_FREQ 4000000
#define LCD_EN_Delay 500
#define LCD_DATA_PORT_D TRISD
#define LCD_RS_D TRISD2
#define LCD_EN_D TRISD1
#define RS RD2
#define EN RD1
#define D4 RD4
#define D5 RD5
#define D6 RD6
#define D7 RD7
//=================================
//-----[ Globals ]-----
uint16_t AN0RES=0;
float Temperature, Voltage;
char* line1 = " Temperature Is ";
char* TempSTR[16];
//=================================
//-----[ Functions Prototypes ]----
void ADC_Init();
uint16_t ADC_Read(uint8_t);
void LCD_Init();
void LCD_Clear();
void LCD_CMD(char);
void LCD_DATA(char);
void LCD_Set_Cursor(char, char);
void LCD_Write_Char(char);
void LCD_Write_String(char*);
//=================================
//---------[ Main Routine]---------
void main(void) {
    
    LCD_Init();
    LCD_Clear();
    ADC_Init();
    while(1)
    {
        // Read The ADC
        AN0RES = ADC_Read(0); // Read Analog Channel 0
        // Calculate The Temperature
        Voltage = AN0RES * 0.0048828;
        Temperature = Voltage / 0.01;
        // Convert The Temperature From Float To String
        sprintf(TempSTR, "     %.3fc", Temperature);
        // Print Out The Temperature
        LCD_Set_Cursor(1,1);
        LCD_Write_String(line1);
        LCD_Set_Cursor(2,1);
        LCD_Write_String(TempSTR);
        __delay_ms(10);
    }
    return;
}
//=================================
//--------[ ADC Routines ]---------
void ADC_Init()
{
    ADCON0 = 0x41;  // Turn ADC ON, Select AN0 Channel, ADC Clock = Fosc/8
    ADCON1 = 0x80;  // All 8 Channels Are Analog, Result is "Right-Justified"
                    // ADC Clock = Fosc/8
}
uint16_t ADC_Read(uint8_t ANC)
{
    if(ANC<0 || ANC>7)    // Check Channel Number Validity
    { return 0;}
    ADCON0 &= 0x11000101; // Clear The Channel Selection Bits
    ADCON0 |= ANC<<3;     // Select The Required Channel (ANC)
                          // Wait The Aquisition Time
    __delay_us(30);       // The Minimum Tacq = 20us, So That should be enough
    GO_DONE = 1;          // Start A/D Conversion
    while(ADCON0bits.GO_DONE); // Polling GO_DONE Bit
                               // Provides Delay Until Conversion Is Complete
    return ((ADRESH << 8) + ADRESL); // Return The Right-Justified 10-Bit Result
}
//=================================
//--------[ LCD Routines ]---------
void LCD_DATA(char Data)
{
    if(Data& 1) 
        D4 = 1;
    else
        D4 = 0;

    if(Data& 2)
        D5 = 1;
    else
        D5 = 0;

    if(Data& 4)
        D6 = 1;
    else
        D6 = 0;

    if(Data& 8) 
        D7 = 1;
    else
        D7 = 0;
}
void LCD_CMD(char a)
{
    RS = 0;           
    LCD_DATA(a); //Incoming Hex value
    EN  = 1;         
    __delay_ms(4);
    EN  = 0;         
}
void LCD_Clear()
{
    LCD_CMD(0); //Clear the LCD
    LCD_CMD(1); //Move the cursor to first position
}
void LCD_Set_Cursor(char a, char b)
{
    char Temp,z,y;
    if(a== 1)
    {
      Temp = 0x80 + b - 1;
        z = Temp>>4; //Lower 8-bits
        y = Temp & 0x0F; //Upper 8-bits
        LCD_CMD(z); //Set Row
        LCD_CMD(y); //Set Column
    }
    else if(a== 2)
    {
        Temp = 0xC0 + b - 1;
        z = Temp>>4; //Lower 8-bits
        y = Temp & 0x0F; //Upper 8-bits
        LCD_CMD(z); //Set Row
        LCD_CMD(y); //Set Column
    }
}
void LCD_Init()
{
  // IO Pin Configurations
  LCD_DATA_PORT_D = 0x00;
  LCD_RS_D = 0;
  LCD_EN_D = 0;
  // The Procedure As Described in Datasheet
  LCD_DATA(0x00);
  __delay_us(LCD_EN_Delay); 
  LCD_CMD(0x03);
  __delay_ms(5);
  LCD_CMD(0x03);
  __delay_ms(11);
  LCD_CMD(0x03); 
  LCD_CMD(0x02); //Clears the RAM and initializes the LCD
  LCD_CMD(0x02); //Clears the RAM and initializes the LCD
  LCD_CMD(0x08); //Select Row 1
  LCD_CMD(0x00); //Clear Row 1 Display
  LCD_CMD(0x0C); //Select Row 2
  LCD_CMD(0x00); //Clear Row 2 Display
  LCD_CMD(0x06);
}
void LCD_Write_Char(char data)
{
   char Lower_Nibble,Upper_Nibble;
   Lower_Nibble = data&0x0F;
   Upper_Nibble = data&0xF0;
   RS = 1;        
   LCD_DATA(Upper_Nibble>>4);  
   EN = 1;
   __delay_us(LCD_EN_Delay); 
   EN = 0;
   __delay_us(LCD_EN_Delay); 
   LCD_DATA(Lower_Nibble); 
   EN = 1;
   __delay_us(LCD_EN_Delay); 
   EN = 0;
   __delay_us(LCD_EN_Delay); 
}
void LCD_Write_String(char *a)
{
    int i;
    for(i=0;a[i]!='\0';i++)
       LCD_Write_Char(a[i]);
}

******COMPONATS******
LM016L
PIC16F877A
LM35


LM35
1 POWER
2 RA0
3 GROUND

LM016L
VSS GROUND
VDD POWER
VEE
RS RD2
RW GROUND
EN RD1

D4 RD4
''
D7 RD7

<------  ------>
******COMPONATS******

<------  ------>
******COMPONATS******

<------  ------>
******COMPONATS******