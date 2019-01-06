#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <FastLED.h>
#include <IMUGY85.h>

IMUGY85 imu;
double ax, ay, az, gx, gy, gz, roll, pitch, yaw;


CHSV color1 = CHSV(0, 255, 255);
CHSV color2 = CHSV(96, 255, 255);

AudioInputAnalog         adc1(A9);  //A9 is on ADC0
AudioAnalyzeFFT256       fft256_1;
AudioConnection          patchCord1(adc1, fft256_1);

int FFTdisplayValueMax16[16]; //max vals for normalization over time
uint8_t FFTdisplayValue16[16]; //max vals for normalization over time
uint8_t FFTdisplayValue8[8]; //max vals for normalization over time
uint8_t FFTdisplayValue5[5]; //max vals for normalization over time
uint8_t FFTdisplayValue1;

int led = 6;


const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 5;

// Pixel layout
//
//      0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
//   +------------------------------------------------
// 0 |  .  0  1  2  3  4  5  6  7  8  9 10 11 12 13  .
// 1 | 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14
// 2 | 30 31 32 33 34 35 36  .  . 37 38 39 40 41 42 43
// 3 | 57 56 55 54 53 52 51  .  . 50 49 48 47 46 45 44
// 4 |  . 58 59 60 61 62  .  .  .  . 63 64 65 66 67  .

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds[ NUM_LEDS ];


// This function will return the right 'led index number' for
// a given set of X and Y coordinates on your RGB Shades.
// This code, plus the supporting 80-byte table is much smaller
// and much faster than trying to calculate the pixel ID with code.
#define LAST_VISIBLE_LED 67
uint8_t XY( uint8_t x, uint8_t y)
{
  // any out of bounds address maps to the first hidden pixel
  if ( (x >= kMatrixWidth) || (y >= kMatrixHeight) ) {
    return (LAST_VISIBLE_LED + 1);
  }

  const uint8_t ShadesTable[] = {
    68,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 69,
    29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14,
    30, 31, 32, 33, 34, 35, 36, 70, 71, 37, 38, 39, 40, 41, 42, 43,
    57, 56, 55, 54, 53, 52, 51, 72, 73, 50, 49, 48, 47, 46, 45, 44,
    74, 58, 59, 60, 61, 62, 75, 76, 77, 78, 63, 64, 65, 66, 67, 79
  };

  uint8_t i = (y * kMatrixWidth) + x;
  uint8_t j = ShadesTable[i];
  return j;
}

//IMUGY85.CPP change orientation   MadgwickQuaternionUpdate(az, ax, ay, (gz)*PI/180.0f, (gx)*PI/180.0f, (gy)*PI/180.0f,  mz,  mx, my);
//input_adc.cpp comment out        analogReference(INTERNAL);
void setup() {
  Serial.begin(115200);

  pinMode(led, OUTPUT);

  //audio library setup
  AudioMemory(3);
  fft256_1.windowFunction(AudioWindowHanning256);
  fft256_1.averageTogether(4);
  FastLED.addLeds<WS2811, 3, GRB>(leds, LAST_VISIBLE_LED + 1);
  FastLED.setBrightness(48);

  imu.init();
}


CHSV map_hsv(uint8_t input, uint8_t in_min, uint8_t in_max, CHSV* out_starting, CHSV* out_ending) {


  if (input <= in_min) return CHSV(*out_starting);
  if (input >= in_max) return CHSV(*out_ending);

  //calculate shortest path between colors
  int16_t shortest_path = out_ending->h; //no rollover
  if ((((int16_t)out_ending->h) + 256) - ((int16_t)out_starting->h) <= 127) {
    shortest_path += 256;  //rollover
  }
  else if ((int16_t)(out_starting->h) - (((int16_t)out_ending->h) - 255) <= 127) {
    shortest_path -= 256; //rollunder
  }


  return CHSV(
           ((input - in_min) * (shortest_path - out_starting->h + 1) / (in_max - in_min + 1) + out_starting->h), \
           (input - in_min) * (out_ending->s - out_starting->s + 1) / (in_max - in_min + 1) + out_starting->s, \
           (input - in_min) * (out_ending->v - out_starting->v + 1) / (in_max - in_min + 1) + out_starting->v);
}
void calcfftcolor(CHSV * temp_color, uint8_t input) {

  //make the tip of the color be color 2
  *temp_color = (input > 240) ? map_hsv(input, 240, 255, &color1, &color2) : color1;

  //ignore brightness, max it.
  temp_color->v = input;

  return;
}

int global_mode = 1;

void loop() {

  if (fft256_1.available()) {

    for (uint8_t i = 0; i < 16; i++) {
      int16_t n = 1000 * fft256_1.read((i * 2), (i * 2) + 2);

      //de-emphasize lower frequencies (for in clubs)
      switch (i) {
        case 0:  n = max(n - 30, 0); break;
        case 1:  n = max(n - 15, 0);  break;
        case 2:  n = max(n - 7, 0);  break;
        case 3:  n = max(n - 4, 0);  break;
        default: n = max(n - 3, 0);   break;
      }

      //falloff controll
      FFTdisplayValueMax16[i] = max(max(FFTdisplayValueMax16[i] * .98, n), 4);
      FFTdisplayValue16[i] = constrain(map(n, 0, FFTdisplayValueMax16[i], 0, 255), 0, 255);

      // downsample 16 samples to 8
      if (i & 0x01) {
        FFTdisplayValue8[i >> 1] = (FFTdisplayValue16[i] + FFTdisplayValue16[i - 1]) >> 1;
      }
    }

    // downsample 8 samples to 1
    FFTdisplayValue1 = 0;
    for (uint8_t i = 0; i < 4; i++) {
      //Serial.print(FFTdisplayValue8[i]);
      //Serial.print( ' ');
      FFTdisplayValue1 += FFTdisplayValue8[i] / 4;
    }
    //Serial.println( ' ');


    //downsample 8 to 5
    FFTdisplayValue5[0] = FFTdisplayValue8[1] ;
    FFTdisplayValue5[1] = FFTdisplayValue8[2];
    FFTdisplayValue5[2] = FFTdisplayValue8[3] ;
    FFTdisplayValue5[3] = FFTdisplayValue8[4] ;
    FFTdisplayValue5[4] = FFTdisplayValue8[6] ;

    if (global_mode == 0) {

      for (uint8_t y = 5; y > 0; y--) {
        for (uint8_t x = 0; x < 16; x++) {
          leds[XY(x, y)] = leds[XY(x, y - 1)];
        }
      }

      for (uint8_t i = 0; i < 16; i++) {
        //make the tip of the color be color 2
        CHSV temp_color;
        calcfftcolor(&temp_color, FFTdisplayValue16[i]);
        leds[XY(i, 0)] = temp_color;
      }

    }


    if (global_mode == 1) {
      for (uint8_t y = 0; y < 5; y++) {
        for (uint8_t x = 0; x < 16; x++) {
          leds[XY(x, y)] = leds[XY(x, y + 1)];
        }
      }

      for (uint8_t i = 0; i < 16; i++) {
        //make the tip of the color be color 2
        CHSV temp_color;
        calcfftcolor(&temp_color, FFTdisplayValue16[i]);
        leds[XY(i, 4)] = temp_color;
      }
    }

    if (global_mode == 2) {
      for (uint8_t y = 0; y < 5; y++) {
        for (uint8_t x = 15; x > 0; x--) {
          leds[XY(x, y)] = leds[XY(x - 1, y )];
        }
      }

      for (uint8_t i = 0; i < 5; i++) {
        //make the tip of the color be color 2
        CHSV temp_color;
        calcfftcolor(&temp_color, FFTdisplayValue5[i]);
        leds[XY(0, i)] = temp_color;
      }
    }

    if (global_mode == 3) {
      for (uint8_t y = 0; y < 5; y++) {
        for (uint8_t x = 0; x < 15 ; x++) {
          leds[XY(x, y)] = leds[XY(x + 1 , y )];
        }
      }

      for (uint8_t i = 0; i < 5; i++) {
        //make the tip of the color be color 2
        CHSV temp_color;
        calcfftcolor(&temp_color, FFTdisplayValue5[i]);
        leds[XY(15, i)] = temp_color;
      }
    }

    FastLED.show();
    imu.update();

    roll = imu.getRoll();
    pitch = imu.getPitch();
    yaw = imu.getYaw();
    Serial.print(pitch); Serial.print("\t");
    Serial.print(roll); Serial.print("\t");
    Serial.print(yaw); Serial.print("\t");
    Serial.println();


#define X_LIMIT1 35
#define X_LIMIT2 70

#define Y_LIMIT1 35
#define Y_LIMIT2 70

    if      (pitch < -X_LIMIT1 && pitch > -X_LIMIT2)             global_mode = 3;
    else if (pitch >  X_LIMIT1 && pitch <  X_LIMIT2)             global_mode = 2;
    else if (roll < (-90 + Y_LIMIT2) && roll > (-90 + Y_LIMIT1)) global_mode = 0;
    else if (roll < (-90 - Y_LIMIT1) && roll > (-90 - Y_LIMIT2)) global_mode = 1;

  }


}
