#include <EFM8LB1.h>
#include <stdio.h>
#include <string.h>

#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L // Baud rate of UART to match Python Controller exactly

#define CMD_NONE 0
#define CMD_FORWARD 1
#define CMD_BACKWARD 2
#define CMD_LEFT 3
#define CMD_RIGHT 4
#define CMD_STOP 5

#define LCD_RS P1_7
#define LCD_E P2_0
#define LCD_D4 P1_3
#define LCD_D5 P1_2
#define LCD_D6 P1_1
#define LCD_D7 P1_0
#define CHARS_PER_LINE 16

#define IR_OUT P2_1
#define IR_START 0xAA
volatile bit ir_carrier_on = 0; 

// function prototypes
void PORT_Init(void);
void Timer2_Init(void);
void IR_send_mark(unsigned int us);
void IR_send_space(unsigned int us);
void IR_send_bit(bit b);
void IR_send_byte(unsigned char value);
void IR_send_command(unsigned char cmd);
void LCD_4BIT(void);
void LCDprint(char *string, unsigned char line, bit clear);
void waitms(unsigned int ms);

void PORT_Init(void) {
  P2MDOUT |= 0x02;  // P2.1 push-pull for IR/LED output
  IR_OUT = 0;
}

char _c51_external_startup(void) {
  SFRPAGE = 0x00;
  WDTCN = 0xDE; 
  WDTCN = 0xAD; 
  SFRPAGE = 0x10;
  PFE0CN = 0x20; 
  SFRPAGE = 0x00;
  
  CLKSEL = 0x00;
  CLKSEL = 0x00;
  while ((CLKSEL & 0x80) == 0);
  CLKSEL = 0x03;
  CLKSEL = 0x03;
  while ((CLKSEL & 0x80) == 0);

  P0MDOUT |= 0x10;         // Enable UART0 TX as push-pull output
  P2MDOUT |= 0b_0000_0010; // P2.1 in push-pull mode
  XBR0 = 0x01;             // Enable UART0 on P0.4(TX) and P0.5(RX)
  XBR1 = 0X00;
  XBR2 = 0x40; // Enable crossbar and weak pull-ups

  // Configure UART0 for 115200 baud
  SCON0 = 0x10; // IMPORTANT: 0x10 sets REN=1 (Receiver Enabled)
  CKCON0 |= 0b_0000_0000;
  TH1 = 0x100 - ((SYSCLK / BAUDRATE) / (2L * 12L));
  TL1 = TH1;
  TMOD &= ~0xf0;
  TMOD |= 0x20;
  TR1 = 1; // Start Timer1
  TI = 1;  // Indicate TX0 ready
  return 0;
}

void Timer2_Init(void) {
  CKCON0 |= 0b_0001_0000;
  TMR2CN0 = 0x00; 
  TMR2RL = 0x10000L - (SYSCLK / (2L * 38000L));
  TMR2 = TMR2RL; 
  ET2 = 1; 
  TR2 = 0; 
}

void Timer2_ISR(void) interrupt INTERRUPT_TIMER2 {
  TF2H = 0;
  if (ir_carrier_on) IR_OUT = !IR_OUT; 
  else               IR_OUT = 0; 
}

void Timer3us(unsigned char us) {
  unsigned char i;
  CKCON0 |= 0b_0100_0000;
  TMR3RL = (-(SYSCLK) / 1000000L);
  TMR3 = TMR3RL;                   
  TMR3CN0 = 0x04;          
  for (i = 0; i < us; i++) {
    while (!(TMR3CN0 & 0x80));                 
    TMR3CN0 &= ~(0x80); 
  }
  TMR3CN0 = 0; 
}

void waitms(unsigned int ms) {
  unsigned int j;
  unsigned char k;
  for (j = 0; j < ms; j++)
    for (k = 0; k < 4; k++)
      Timer3us(250);
}

void IR_send_mark(unsigned int us) {
  ir_carrier_on = 1;
  TR2 = 1;
  while (us >= 200) {
    Timer3us(200);
    us -= 200;
  }
  if (us > 0) Timer3us((unsigned char)us);
  ir_carrier_on = 0;
  TR2 = 0;
  IR_OUT = 0;
}

void IR_send_space(unsigned int us) {
  ir_carrier_on = 0;
  TR2 = 0;
  IR_OUT = 0;
  while (us >= 200) {
    Timer3us(200);
    us -= 200;
  }
  if (us > 0) Timer3us((unsigned char)us);
}

void IR_send_bit(bit b) {
  if (b) {
    IR_send_mark(1200);
    IR_send_space(600);
  } else {
    IR_send_mark(600);
    IR_send_space(1200);
  }
}

void IR_send_byte(unsigned char value) {
  unsigned char i;
  for (i = 0; i < 8; i++) {
    IR_send_bit(value & 0x01);
    value >>= 1;
  }
}

void IR_send_command(unsigned char cmd) {
  IR_send_mark(2400);
  IR_send_space(600);
  IR_send_byte(IR_START);
  IR_send_byte(cmd);
  IR_send_space(3000);
}

void LCD_pulse(void) {
  LCD_E = 1;
  Timer3us(40);
  LCD_E = 0;
}

void LCD_byte(unsigned char x) {
  ACC = x; 
  LCD_D7 = ACC_7; LCD_D6 = ACC_6; LCD_D5 = ACC_5; LCD_D4 = ACC_4;
  LCD_pulse();
  Timer3us(40);
  ACC = x; 
  LCD_D7 = ACC_3; LCD_D6 = ACC_2; LCD_D5 = ACC_1; LCD_D4 = ACC_0;
  LCD_pulse(); 
}

void WriteData(unsigned char x) {
  LCD_RS = 1; 
  LCD_byte(x);
  waitms(2);
}

void WriteCommand(unsigned char x) {
  LCD_RS = 0; 
  LCD_byte(x);
  waitms(5);
}

void LCD_4BIT(void) {
  LCD_E = 0; 
  waitms(20);
  WriteCommand(0x33);
  WriteCommand(0x33);
  WriteCommand(0x32); 
  WriteCommand(0x28);
  WriteCommand(0x0c);
  WriteCommand(0x01); 
  waitms(20);         
}

void LCDprint(char *string, unsigned char line, bit clear) {
  int j;
  WriteCommand(line == 2 ? 0xc0 : 0x80);
  waitms(5);
  for (j = 0; string[j] != 0; j++)
    WriteData(string[j]); 
  if (clear)
    for (; j < CHARS_PER_LINE; j++)
      WriteData(' '); 
}

// -------------------------------------------------------------
// The Master UART Bridge Loop
// -------------------------------------------------------------
void main(void) {
  char cmd_char = 'S';
  unsigned char cmd_byte = CMD_STOP;
  
  LCD_4BIT();
  PORT_Init();
  Timer2_Init();
  EA = 1; 

  LCDprint("CV BRIDGE READY!", 1, 1);
  LCDprint("STOP", 2, 1);

  while (1) {
    // 1. Check if the Laptop's Python Script sent us a character (RX Interrupt Flag)
    if (RI) {
        cmd_char = SBUF0;   // Capture the character ('F', 'B', etc.)
        RI = 0;             // Reset the interrupt flag immediately
        
        // 2. Map the Python ASCII Character to our specific IR Payload bytes
        if      (cmd_char == 'F') cmd_byte = CMD_FORWARD;
        else if (cmd_char == 'B') cmd_byte = CMD_BACKWARD;
        else if (cmd_char == 'L') cmd_byte = CMD_LEFT;
        else if (cmd_char == 'R') cmd_byte = CMD_RIGHT;
        else                      cmd_byte = CMD_STOP;

        // 3. Print the new state to the LCD
        switch(cmd_char) {
            case 'F': LCDprint("FORWARD", 2, 1); break;
            case 'B': LCDprint("BACKWARD", 2, 1); break;
            case 'L': LCDprint("LEFT", 2, 1); break;
            case 'R': LCDprint("RIGHT", 2, 1); break;
            default:  LCDprint("STOP", 2, 1); break;
        }
    }

    // 4. Fire the currently cached command out the IR LED.
    // This perfectly bridges the 0.5s python debounce to a constant 40ms stream
    // for the TSOP_331 transceiver to decode.
    IR_send_command(cmd_byte);
    waitms(40);
  }
}
