//************************************************************************************************************//
  
//********************* THE 8x16 RGB LED MATRIX USING BIT ANGLE MODULATION METHOD ********************//

//************************************************************************************************************//
#include <SPI.h>
#include "font8x8.h"
#include "bitmap.h"

#define blank_pin 3   // Defines actual BIT of PortD for blank - is Arduino UNO pin 3, MEGA pin 5
#define latch_pin 2   // Defines actual BIT of PortD for latch - is Arduino UNO pin 2, MEGA pin 4
#define clock_pin 13  // used by SPI, must be 13 SCK 13 on Arduino UNO, 52 on MEGA
#define data_pin 11   // used by SPI, must be pin MOSI 11 on Arduino UNO, 51 on MEGA

#define RowA_Pin 4
#define RowB_Pin 5
#define RowC_Pin 6
#define RowD_Pin 7

byte red[4][16];
byte green[4][16];
byte blue[4][16];

int level=0;//keeps track of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things


#define myPI      3.14159265358979323846
#define myDPI     1.2732395
#define myDPI2    0.40528473
#define dist(a, b, c, d) sqrt(double((a - c) * (a - c) + (b - d) * (b - d)))

//*********** Defining the Matrix *************

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G & B (4096 colours)
const  byte Size_Y = 16;    //Number of Layers Y axis (levels/Layers)
const  byte Size_X = 8;     //Number of LEDs X axis (Left to right across front)
//***************************************************Layer*********************************************************//

#define COLOUR_WHEEL_LENGTH 256

uint8_t colourR[COLOUR_WHEEL_LENGTH];
uint8_t colourG[COLOUR_WHEEL_LENGTH];
uint8_t colourB[COLOUR_WHEEL_LENGTH];
int16_t ColPos = 0;
uint16_t colourPos;
uint8_t R, G, B;
byte myred, mygreen, myblue;

/**   An RGB color template */
struct Color
{
  unsigned char red, green, blue;

  Color(int r, int g, int b) : red(r), green(g), blue(b) {}
  Color() : red(0), green(0), blue(0) {}
};

const Color redcolor        = Color(0x0F, 0x00, 0x00);
const Color orangecolor     = Color(0x0F, 0x0F, 0x00);
const Color yellowcolor     = Color(0x0F, 0x09, 0x00);
const Color greencolor      = Color(0x00, 0x0F, 0x00);
const Color bluecolor       = Color(0x00, 0x00, 0x0F);

#define RED     0x0F,0x00,0x00
#define ORANGE  0x0F,0x04,0x00
#define YELLOW  0x0F,0x09,0x00
#define GREEN   0x00,0x0F,0x00
#define TEAL    0x00,0x0F,0x04
#define BLUE    0x00,0x00,0x0F
#define PURPLE  0x0F,0x00,0x0F
#define WHITE   0x0F,0x0F,0x0F
#define CLEAR   0x00,0x00,0x00

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 10;

//pinMode (2, OUTPUT); // turn off PWM and set PortD bit 4 as output
//pinMode (3, OUTPUT); // turn off PWM and set PortD bit 5 as output
pinMode(latch_pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(RowA_Pin, OUTPUT);
pinMode(RowB_Pin, OUTPUT);
pinMode(RowC_Pin, OUTPUT);
pinMode(RowD_Pin, OUTPUT);

SPI.begin();
interrupts();

fill_colour_wheel();
}

void loop()
{

  clearfast();
  char scrolltext_1[]="    * Welcome to my 'RGB Matrix 16x8' *   ";
  clearfast();
  fillTable(TEAL);
  delay(3000);
  clearfast();
  fillTable(PURPLE);
  delay(3000);
  clearfast();
  fillTable(GREEN);
  delay(3000);
  clearfast();
  fillTable(BLUE);
  delay(3000);
  clearfast();
  fillTable(YELLOW);
  delay(3000);
  clearfast();
  plasma();
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 304, 8, INSTRUCTABLES, 50); 
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 168, 8, RANDOMPLASMA, 50); 
  clearfast();
  randomPlasma();
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 144, 8, FILLDAZZLE, 50); 
  clearfast();
  FillDazzle_Vcolorwheel(); 
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 200, 8, COLORMORPH, 50); 
  clearfast();
  colorMorphTable(30);
  clearfast();
  HScrollBigImageL_colorwheel(0, 0, 224, 8, COLORWHEEL, 50); 
  clearfast();   
  fillTable_colorwheel();
  hScroll(0, redcolor, bluecolor, scrolltext_1);
  clearfast(); 
}

void LED(int X, int Y, int R, int G, int B)
{
  X = constrain(X, 0, 15); 
  Y = constrain(Y, 0, 7);
  
  R = constrain(R, 0, 15);
  G = constrain(G, 0, 15); 
  B = constrain(B, 0, 15);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][X], Y, bitRead(R, BAM));

    bitWrite(green[BAM][X], Y, bitRead(G, BAM));

    bitWrite(blue[BAM][X], Y, bitRead(B, BAM));
  }

}

void rowScan(byte row)
{
  
  if (row & 0x01) PORTD |= 1<<RowA_Pin;
    else PORTD &= ~(1<<RowA_Pin);
  
  if (row & 0x02) PORTD |= 1<<RowB_Pin;
    else PORTD &= ~(1<<RowB_Pin);

  if (row & 0x04) PORTD |= 1<<RowC_Pin;
    else PORTD &= ~(1<<RowC_Pin);

  if (row & 0x08) PORTD |= 1<<RowD_Pin;
    else PORTD &= ~(1<<RowD_Pin);
}

ISR(TIMER1_COMPA_vect){
  
PORTD |= ((1<<blank_pin));
if(BAM_Counter==8)
BAM_Bit++;
else
if(BAM_Counter==24)
BAM_Bit++;
else
if(BAM_Counter==56)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:
      
      //Blue        
        myTransfer(blue[0][level]);      
      //Green        
        myTransfer(green[0][level]);     
      //Red     
        myTransfer(red[0][level]);
      break;
    case 1:

      //Blue       
        myTransfer(blue[1][level]);
      //Green
        myTransfer(green[1][level]);
      //Red
        myTransfer(red[1][level]);       
      break;
    case 2:
      
      //Blue      
        myTransfer(blue[2][level]);       
      //Green
        myTransfer(green[2][level]);      
      //Red
        myTransfer(red[2][level]);
      break;
    case 3:
      
      //Blue     
        myTransfer(blue[3][level]);     
      //Green
        myTransfer(green[3][level]);      
      //Red
        myTransfer(red[3][level]);
        
  if(BAM_Counter==120){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(level);

PORTD |= 1<<latch_pin;
PORTD &= ~(1<<latch_pin);
delayMicroseconds(2); 
PORTD &= ~(1<<blank_pin);
//delayMicroseconds(5);
level++;
if(level==16)
level=0;
pinMode(blank_pin, OUTPUT);

}

inline static uint8_t myTransfer(uint8_t C_data){
  SPDR = C_data;
  asm volatile("nop"); 
  asm volatile("nop");
}


void clearfast ()
{
for (unsigned char j=0; j<16; j++)
        {
        red[0][j]   = 0;
        red[1][j]   = 0;
        red[2][j]   = 0;
        red[3][j]   = 0;
        green[0][j] = 0;
        green[1][j] = 0;
        green[2][j] = 0;
        green[3][j] = 0;
        blue[0][j] = 0;
        blue[1][j] = 0;
        blue[2][j] = 0;
        blue[3][j] = 0;
        }
}

void fillTable(byte R, byte G, byte B)
{
    for (byte x=0; x<16; x++)
    {
      for (byte y=0; y<8; y++)
      {
        LED(x, y, R, G, B);
      }
    }
}

void fillTable_colorwheel(){  // This subroutine fills the cube with a colour
  uint8_t R, G, B;
  for (byte inter=0; inter<10; inter++)
  {
    for (byte x=0; x<16; x++)
    {
      for (byte y=0; y<8; y++)
      {
        get_colour(colourPos + 8*x, &R, &G, &B);
        LED(x, y, R, G, B);      
      }
      increment_colour_pos(8);
      delay(50);
    }
    delay(500);
    //clearfast();
  for (byte y=0; y<8; y++)
    {    
    for (byte x=0; x<16; x++)
    {

        get_colour(colourPos + 8* inter, &R, &G, &B);
        LED(15-x, 7-y, R, G, B);      
      }
      increment_colour_pos(8);
      delay(50);
    }
  delay(1000);
}
}

//*******************************************************MK4*****************************************************//

//FAST SINE APPROX
float mySin(float x){
  float sinr = 0;
  uint8_t g = 0;

  while(x > myPI){
    x -= 2*myPI; 
    g = 1;
  }

  while(!g&(x < -myPI)){
    x += 2*myPI;
  }

  sinr = myDPI*x - myDPI2*x*myAbs(x);
  sinr = 0.225*(sinr*myAbs(sinr)-sinr)+sinr;

  return sinr;
}

//FAST COSINE APPROX
float myCos(float x){
  return mySin(x+myPI/2);
}

float myTan(float x){
  return mySin(x)/myCos(x);
}

//SQUARE ROOT APPROX
float mySqrt(float in){
  int16_t d = 0;
  int16_t in_ = in;
  float result = 2;
  
  for(d = 0; in_ > 0; in_ >>= 1){
    d++;
  }
  
  for(int16_t i = 0; i < d/2; i++){
    result = result*2;
  }
  
  for(int16_t i = 0; i < 3; i++){
    result = 0.5*(in/result + result);
  }
  
  return result;
}

//MAP NUMBERS TO NEW RANGE
float myMap(float in, float inMin, float inMax, float outMin, float outMax){
  float out;
  out = (in-inMin)/(inMax-inMin)*(outMax-outMin) + outMin;
  return out;
}

//ROUND A NUMBER
int16_t myRound(float in){
  int8_t s = in/myAbs(in);
  return (int16_t)(s*(myAbs(in) + 0.5));
}

//ABSOLUTE VALUE
float myAbs(float in){
  return (in)>0?(in):-(in);
} 

void fill_colour_wheel(void) 
{
  float red, green, blue;
  float c, s;
  int32_t phase = 0;
  int16_t I = 0;

  while (phase < COLOUR_WHEEL_LENGTH) 
  {
    s = (1 << BAM_RESOLUTION)*mySin(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));
    c = (1 << BAM_RESOLUTION)*myCos(myPI*(3 * phase - I*COLOUR_WHEEL_LENGTH) / (2 * COLOUR_WHEEL_LENGTH));

    red = (I == 0 ? 1 : 0)*s + (I == 1 ? 1 : 0)*c;
    green = (I == 1 ? 1 : 0)*s + (I == 2 ? 1 : 0)*c;
    blue = (I == 2 ? 1 : 0)*s + (I == 0 ? 1 : 0)*c;

    colourR[phase] = red;
    colourG[phase] = green;
    colourB[phase] = blue;

    if (++phase >= (1 + I)*COLOUR_WHEEL_LENGTH / 3) 
      I++;
  }
}

void get_colour(int16_t p, uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (p >= COLOUR_WHEEL_LENGTH)
    p -= COLOUR_WHEEL_LENGTH;

  *R = colourR[p];
  *G = colourG[p];
  *B = colourB[p];
}

void get_next_colour(uint8_t *R, uint8_t *G, uint8_t *B)
{
  if (++ColPos >= COLOUR_WHEEL_LENGTH)
    ColPos -= COLOUR_WHEEL_LENGTH;

  *R = colourR[ColPos];
  *G = colourG[ColPos];
  *B = colourB[ColPos];
}

void increment_colour_pos(uint8_t i)
{
  colourPos += i;
  while (colourPos >= COLOUR_WHEEL_LENGTH)
  {
    colourPos -= COLOUR_WHEEL_LENGTH;
  }
}


void colorMorph(int time) {
  int red, green, blue;
  int keepColorTime = time * 150;
  
  delay(keepColorTime);
  // RED + GREEN
  for(int green = 0; green <= 15; green++) {
    fillTable(15, green, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN - RED
  for(int red = 15; red >= 0; red --) {
    fillTable(red, 15, 0);
    delay(time);
  }
  delay(keepColorTime);
  // GREEN + BLUE
  for(int blue = 0; blue <= 15; blue++) {
    fillTable(0, 15, blue);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE - GREEN
  for(int green = 15; green >= 0; green --) {
    fillTable(0, green, 15);
    delay(time);
  }
  delay(keepColorTime);
  // BLUE + RED
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 0,15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - BLUE + GREEN
  green = 0;
  for(int blue = 15; blue >= 0; blue --) {
    fillTable(15, green, blue);
    delay(time);
    green++;
  }
  delay(keepColorTime);
  // GREEN - RED + BLUE
  blue = 0;
  for(int red = 15; red >= 0; red --) {
    fillTable(red, 15, blue);
    delay(time);
    blue++;
  }
  delay(keepColorTime);
  // GREEN + RED + BLUE
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 15, 15);
    delay(time);
  }
  delay(keepColorTime);
  // RED - GREEN - BLUE
  blue = 15;
  for(int green = 15; green >= 0; green --) {
    fillTable(15, green, blue);
    delay(time);
    blue--;
  }
}

void colorMorphTable(int time) 
{
  for(int red = 0; red <= 15; red++) {
    fillTable(red, 0,0);
    delay(time);
  }
  colorMorph(time);
  for(int red = 15; red >= 0; red--) {
    fillTable(red, 0,0);
    delay(time);
  }
  delay(500);
}


Color getPixel(int X, int Y) {
  // Check parameters
  Color pixelColor;
  X = constrain(X, 0, 15);
  Y = constrain(Y, 0, 7);
  
  // RED

  bitWrite(pixelColor.red, 0, bitRead(red[0][X], Y));
  bitWrite(pixelColor.red, 1, bitRead(red[1][X], Y));
  bitWrite(pixelColor.red, 2, bitRead(red[2][X], Y));
  bitWrite(pixelColor.red, 3, bitRead(red[3][X], Y));


  // GREEN

  bitWrite(pixelColor.green, 0, bitRead(green[0][X], Y));
  bitWrite(pixelColor.green, 1, bitRead(green[1][X], Y));
  bitWrite(pixelColor.green, 2, bitRead(green[2][X], Y));
  bitWrite(pixelColor.green, 3, bitRead(green[3][X], Y));


  // BLUE

  bitWrite(pixelColor.blue, 0, bitRead(blue[0][X], Y));
  bitWrite(pixelColor.blue, 1, bitRead(blue[1][X], Y));
  bitWrite(pixelColor.blue, 2, bitRead(blue[2][X], Y));
  bitWrite(pixelColor.blue, 3, bitRead(blue[3][X], Y));

  return pixelColor;
}

void fade(uint8_t x, uint8_t y)
{
Color pixelColor;
    pixelColor = getPixel(x,y);
    if(pixelColor.red > 0)
      pixelColor.red--;
    if(pixelColor.green > 0)
      pixelColor.green--;
    if(pixelColor.blue > 0)
      pixelColor.blue--;
    LED(x, y, pixelColor.red, pixelColor.green, pixelColor.blue);    
}

bool Fly_colorwheel( int16_t start, int16_t end, int16_t offs_ortho) {
   //  static uint8 ctr;
    int16_t i;
        for ( i = start; i <= end; i++ ) {
            //manage_color();
            get_colour(colourPos + 2*(offs_ortho + start + end), &R, &G, &B);
            LED(i, offs_ortho, R, G, B);
            //delay(10);
            if ( ( i-1 )  >= 0 ) // we don't want to go outside the buffer
            LED(( i - 1 ), offs_ortho, 0, 0, 0);   // Clear the previous Voxel before updating the current one
            delay(30);
            increment_colour_pos(2);
        }
 }

 void FillDazzle_Hcolorwheel() {
    int16_t i, ii, j;   
        for ( ii = 0; ii <= 7; ii++ ) {         // Y-offset
            for ( i = 15; i >= 0; i-- ) {        // X-offset
                // pop back to A_Animate
                // params: plane, start, end, ortho-offset, perp-offset
                Fly_colorwheel(0, i, ii);
            }
        }
    }

 void FillDazzle_Vcolorwheel() {
    int16_t i, ii, j;   
    for ( i = 15; i >= 0; i-- ) {        // X-offset
        for ( ii = 0; ii <= 7; ii++ ) {         // Y-offset
                // pop back to A_Animate
                // params: plane, start, end, ortho-offset, perp-offset
                Fly_colorwheel(0, i, ii);
            }
        }
    }

 void FillDazzle_Mix() {
    int16_t i, ii, j;   
        for ( ii = 0; ii <= 7; ii++ ) {         // Y-offset
            for ( i = 15; i >= 0; i-- ) {        // X-offset
                // pop back to A_Animate
                // params: plane, start, end, ortho-offset, perp-offset
                Fly_colorwheel(0, i, ii);
                Fly_colorwheel(0, i, 7-ii);
            }
        }
        delay(3000);
    }

void plasma() {
int time = 0;
  
  for(int loopIt = 0; loopIt < 500; loopIt++)
    { 
    for(int y = 0; y <= 7; y++)
    {
      for(int x = 0; x <= 15; x++)
      {
        uint32_t color =
        (
              64.0 + (64.0 * sin(sqrt(double(x+time ))))
            + 64.0 - (64.0 * sin(sqrt(double(y+time))))
            + 64.0 - (64.0 * sin(sqrt(double(x+y+8+time))))
           //+ 64.0 + (64.0 * sin(sqrt(double((x+4)^2 +(y+2)^2+ time))))
        ) ;
        get_colour(int(color)%256, &R, &G, &B);
        LED(x, y, R, G, B);
        }
      }   
    time++;
  } 
}   

// 
void randomPlasma() {
    
   double time = 0;
          
    for(int looper = 0; looper < 250; looper++)
    {
        //time = millis()%100;
        //time = looper;
        time = looper;

        for(int x = 0; x < 16; x++)
        {
          for(int y = 0; y < 8; y++)
          {
              double value = 32.0+ 32.0*sin(dist(x + time, y, 64.0, 64.0) / 2.0)
                         + 32.0 + 32.0*sin(dist(x-time, y,  32.0, 32.0) / 2.0)
                         + 32.0 + 32.0*sin(dist(x, y + time, 95.0, 32.0) / 1.5)
                         + 32.0 + 32.0*sin(dist(x, y-time, 95.0, 50.0) / 2.0);
            int color = int(value)%256;
            get_colour(color, &R, &G, &B);
            LED(x, y, R, G, B);
          }    
        }

      //delay(10);
    }  
}


void printChar(uint8_t x, uint8_t y, Color For_color, Color Bk_color, char ch)
{
  uint8_t xx,yy;
  xx=0;
  yy=0; 
  
  for (yy=0; yy < 8; yy++)
    {
    for (xx=0; xx < 8; xx++)
      {
      if (bitRead(pgm_read_byte(&font8x8[ch-32][7-yy]),7-xx)) // 4 == Font witdh -1
      
        {
        LED(x+xx,y+yy,For_color.red, For_color.green, For_color.blue);
        }
      else
        {
        LED(x+xx,y+yy, Bk_color.red, Bk_color.green, Bk_color.blue);        
        }
      }
    }
}


void hScroll(uint8_t y, Color For_color, Color Bk_color, char *mystring)
{
// FONT 8x8
    for (int offset=0; offset <((lenString(mystring)-8)*8-1); offset++)
      {
      for (byte xx=0; xx<16; xx++)
        {
          for (byte yy=0; yy<8; yy++)
              {
                Color setcolor;
                if (getPixelHString(xx+offset, yy, mystring)) 
                  setcolor = For_color; 
                else setcolor=Bk_color;
                  LED(xx, yy+y, setcolor.red, setcolor.green, setcolor.blue);
              }          
        }
      delay(60);  
      }
}

void hScroll_colorwheel(uint8_t y, Color Bk_color, char *mystring, uint8_t font, uint8_t delaytime, uint8_t times, uint8_t dir)
{
  //int offset =0;
  // FONT 5x7
  int offset;
  Color setcolor, For_color;

  while (times)
    {
    for ((dir) ? offset=0 : offset=((lenString(mystring)-8)*8-1) ; (dir) ? offset <((lenString(mystring)-8)*8-1) : offset >0; (dir) ? offset++ : offset--)
      {
      for (byte xx=0; xx<16; xx++)
        {
        for (byte yy=0; yy<8; yy++)
            {
              get_colour(colourPos + 8*(yy+xx), &For_color.red, &For_color.green, &For_color.blue);
              if (getPixelHString(xx+offset,yy,mystring))
                setcolor = For_color;                
            
              else setcolor=Bk_color;
                LED(xx,(yy+y),setcolor.red, setcolor.green, setcolor.blue);
            }
        }
        delay(delaytime); 
        increment_colour_pos(2); 
      }
    times--;
    }
  }


 unsigned int lenString(char *p)
{
  unsigned int retVal=0;
  while(*p!='\0')
  { 
   retVal++;
   p++;
  }
  return retVal;
}

byte getPixelChar(uint8_t x, uint8_t y, char ch)

{
    //ch = ch-32;
    if (x > 7) return 0; // 4 = font Width -1
    return bitRead(pgm_read_byte(&font8x8[ch-32][7-y]),7-x); // 4 = Font witdh -1    
}

byte getPixelHString(uint16_t x, uint16_t y, char *p)
{
    p=p+x/7;
    return getPixelChar(x%7,y,*p);  
}


void drawImage(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color)
{
    for (uint16_t y = 0; y < height; y++)
    {
        for (uint16_t x = 0; x < width; x++)
        {
            uint16_t  myindex =  x/8 + y*2;
            uint8_t   mybitmask = 7-(x % 8);
            uint8_t   colorImage = bitRead(pgm_read_byte(&image[width-myindex]),mybitmask) & 1;
              if (colorImage)
                {
                LED(x+xoffset,7-(y+yoffset),For_color.red, For_color.green, For_color.blue);
                }
              else
                {
                LED(x+xoffset,7-(y+yoffset),Bk_color.red, Bk_color.green, Bk_color.blue);        
                }
          }
    }
}
void HScrollBigImageL(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, Color For_color, Color Bk_color, uint16_t delaytime )
{
    for (uint16_t i= 0; i < width-16; i++)
    {
    for (uint16_t y = 0; y < height; y++)
      {
        for (uint16_t x = 0; x < 16; x++)
        {
            uint16_t   myindex = (i+x)/8 + y * (width / 8);
            uint16_t   mybitmask = 7-((i+x) % 8);
            uint16_t   colorImage = bitRead(pgm_read_byte(&image[myindex]), mybitmask) & 1;
              if (colorImage)
                {
                LED((x+xoffset), 7-(y+yoffset), For_color.red, For_color.green, For_color.blue);                             
                }
              else
                {
                LED((x+xoffset), 7-(y+yoffset), Bk_color.red, Bk_color.green, Bk_color.blue);        
                }
          }
      }
  delay(delaytime);
  }
  
} 

 void HScrollBigImageL_colorwheel(uint16_t xoffset, uint16_t yoffset, uint16_t width, uint16_t height, const uint8_t *image, uint16_t delaytime )
{
    Color For_color;
    for (uint16_t i= 0; i < width-16; i++)
    {
    for (uint16_t y = 0; y < height; y++)
      {
        for (uint16_t x = 0; x < 16; x++)
        {
            uint16_t   myindex = (i+x)/8 + y * (width / 8);
            uint16_t   mybitmask = 7-((i+x) % 8);
            uint16_t   colorImage = bitRead(pgm_read_byte(&image[myindex]), mybitmask) & 1;
              if (colorImage)
                {
                get_colour(colourPos + 4*(4*x+2*y), &For_color.red, &For_color.green, &For_color.blue);
                LED(x+xoffset, 7-(y+yoffset), For_color.red, For_color.green, For_color.blue);                             
                }
              else
                {
                LED(x+xoffset, 7-(y+yoffset), 0, 0, 0);        
                }
          }
      }
  delay(delaytime);
  increment_colour_pos(2); 
  }
  
}
