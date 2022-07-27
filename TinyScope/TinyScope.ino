#include <U8g2lib.h>
#include "logo.h"
//#include <U8x8lib.h>


//////////////////////////////////////////////
//Pins


#define ENC_BUT     2
#define ENC_INT     3
#define ENC_DAT     4

#define MUX_ADDR_A0 8
#define MUX_ADDR_A1 9

#define ANALOG_IN   A0


//////////////////////////////////////////////

#define BMP_BUF               128

#define ADC_MAX               1023.0
#define SCOPE_HEIGHT          64.0
#define SCOPE_WIDTH           128
#define ALPHA                 SCOPE_HEIGHT / ADC_MAX //conversion

#define TRIG_VALUE      32 //useful to sync signal

//U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
U8G2_SH1106_128X64_NONAME_F_HW_I2C disp(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8X8_SSD1306_128X64_NONAME_HW_I2C disp(/* reset=*/ U8X8_PIN_NONE);
uint8_t x;

union memBuf { // a little trick to reduce memory usage
  uint16_t y[128];
  uint8_t bmp[BMP_BUF];
} buf;


//float t = 0.0;
uint8_t syncState; //for trigger

uint16_t SR_time = 10; // ---> samplerate
uint8_t channelScope; //There 4 channels


void setup() {


  pinMode(ENC_BUT, INPUT_PULLUP);
  pinMode(ENC_INT, INPUT_PULLUP);
  pinMode(ENC_DAT, INPUT_PULLUP);


  pinMode(MUX_ADDR_A0, OUTPUT);
  pinMode(MUX_ADDR_A1, OUTPUT);
  
  disp.begin();


  //Serial.println("hi");
  
  //animated logo display 
  for (uint8_t frame = 0; frame < 12; frame++)
  {
    for (uint16_t j = 0; j < 8; j++)
    {
      for (uint16_t i = 0; i < BMP_BUF; i++)
      {
        buf.bmp[i] = pgm_read_byte(&logo[frame*1024 + j * 128 + i]);
      }
      disp.drawBitmap(0, j * 8, 16, 8, buf.bmp);
    }
    disp.updateDisplay();
    delay(50);
  }

  delay(3000);

  ADCSRA = (ADCSRA & 0xF8) | 0x02; //little hack to increase ADC sample rate :o

  //interrupt setup
  attachInterrupt(digitalPinToInterrupt(ENC_INT), encoderInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_BUT), buttonInterrupt, FALLING);


  
}




void encoderInterrupt(void)
{
  if(digitalRead(ENC_DAT)) //clockwise ?
  {
    SR_time++;
    if(SR_time > 2000)
      SR_time = 2000;
  }
  else // or counter clockwise
  {
    SR_time--;
    if(SR_time < 1)
      SR_time = 1;
  }
}

void buttonInterrupt(void)
{
  channelScope++; //another little hack if pins used are PB0 & PB1
  if(channelScope > 4)
  {
    channelScope = 0;
  }
  //switch multiplexer I/O
  digitalWrite(MUX_ADDR_A0, channelScope & 1);
  digitalWrite(MUX_ADDR_A1, channelScope & 2);
}




void loop() {

  for (x = 0; x < SCOPE_WIDTH; x++) //buffer read
  {
    buf.y[x] = analogRead(ANALOG_IN);
    delayMicroseconds(SR_time);
  }

  for (x = 0; x < SCOPE_WIDTH; x++) //conversion and write on graphic buffer
  {
    buf.y[x] *= ALPHA;
    disp.drawPixel(x, buf.y[x]);
  }




  
  //update display
  disp.updateDisplay();
  disp.clearBuffer();
  //display "measured area"
  disp.drawHLine(63, 0, map(SR_time, 1, 2000, 1, 63));
  disp.drawHLine(63-map(SR_time, 1, 2000, 1, 63), 0, map(SR_time, 1, 2000, 1, 63));
  //disp.clear(); //to long to use this
  syncState = 0;
  while (syncState < 3) //sync signal
  {
    buf.y[0] = analogRead(ANALOG_IN) * ALPHA;

    //Maybe think about an optimisation here ?
    if (buf.y[0] > TRIG_VALUE && syncState == 0)
    {
      syncState++;
    }
    if (buf.y[0] < TRIG_VALUE && syncState == 1)
    {
      syncState++;
    }
    if (buf.y[0] > TRIG_VALUE && syncState == 2)
    {
      syncState++;
    }
  }








}
