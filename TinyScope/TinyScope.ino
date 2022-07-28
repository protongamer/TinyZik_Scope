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

#define TEXT_FONT_USED             u8g2_font_squeezed_b7_tr
#define TEXT_CURSOR_X              5
#define TEXT_CURSOR_Y              7


#define BMP_BUF               128 //buffer to print frame
#define ENC_INC               10 //samplerate increment/decrement

#define ADC_MAX               650 
#define ADC_MIN               203
#define SCOPE_HEIGHT          64
#define SCOPE_WIDTH           128
//#define ALPHA                 (float)SCOPE_HEIGHT / (float)ADC_MAX //conversion

#define TRIG_VALUE      28 //useful to sync signal (middle of the screen... you got it I suppose ?)

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SH1106_128X64_NONAME_F_HW_I2C disp(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8X8_SSD1306_128X64_NONAME_HW_I2C disp(/* reset=*/ U8X8_PIN_NONE);
uint8_t x;

union memBuf { // a little trick to reduce memory usage
  uint16_t y[128];
  uint8_t bmp[BMP_BUF];
} buf;


//float t = 0.0;
uint8_t syncState; //for trigger

volatile uint16_t SR_time = 10; // ---> samplerate
volatile uint8_t channelScope = 0; //There 4 channels
uint8_t trigTimeOut;

char textBuf[5];

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
        buf.bmp[i] = pgm_read_byte(&logo[frame * 1024 + j * 128 + i]);
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

  disp.setFont(TEXT_FONT_USED);

}




void encoderInterrupt(void)
{
  if (digitalRead(ENC_DAT)) //clockwise ?
  {
    SR_time += ENC_INC;
    if (SR_time > 2000)
      SR_time = 2000;
  }
  else // or counter clockwise
  {
    SR_time -= ENC_INC;
    if (SR_time < ENC_INC)
      SR_time = ENC_INC;
  }
}

void buttonInterrupt(void)
{
  channelScope++; //another little hack if pins used are PB0 & PB1
  if (channelScope >= 4)
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
    buf.y[x] = map(buf.y[x], ADC_MIN, ADC_MAX, 0, SCOPE_HEIGHT);
  }

  for (x = 1; x < SCOPE_WIDTH; x++) //conversion and write on graphic buffer
  {
    disp.drawLine(x-1, 63-buf.y[x-1], x, 63-buf.y[x]);
  }

  //update display
  
 
  
  disp.setCursor(TEXT_CURSOR_X,TEXT_CURSOR_Y);
  sprintf(textBuf, "CH%d", channelScope+1);
  disp.print(textBuf);
  disp.updateDisplay();
  disp.clearBuffer();
  //display "measured area"
  disp.drawHLine(63, 0, map(SR_time, ENC_INC, 2000, ENC_INC, 63));
  disp.drawHLine(63 - map(SR_time, ENC_INC, 2000, ENC_INC, 63), 0, map(SR_time, ENC_INC, 2000, ENC_INC, 63));
  //disp.clear(); //to long to use this
  syncState = 0;
  trigTimeOut = 0;
  while ((syncState < 3) && (trigTimeOut < 200)) //sync signal
  {
    buf.y[0] = map(analogRead(A0), ADC_MIN, ADC_MAX, 0, SCOPE_HEIGHT);

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
    trigTimeOut++;
  }








}
