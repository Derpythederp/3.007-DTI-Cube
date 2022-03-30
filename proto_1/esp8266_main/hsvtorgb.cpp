// Reference code:
// https://gist.github.com/postspectacular/2a4a8db092011c6743a7#file-hsv2rgb-ino-L29
// https://piandmore.wordpress.com/2020/08/15/rgb-to-hsv-and-vice-versa/
// To check colour: https://www.rapidtables.com/convert/color/hsv-to-rgb.html

#include <Arduino.h>

float fract(float x) { return x - int(x); }

float mix(float a, float b, float t) { return a + (b - a) * t; }

float step(float e, float x) { return x < e ? 0.0 : 1.0; }

float* hsv2rgb(float h, float s, float v, float* rgb) {
  // Expects h, s, v values in float from 0.0 to 1.0
  // Writes RGB value to rgb buffer from 0.0 to 255.0
  h = h * 360.0;  // could be optimized but i dun like typing 0.1666666666
  float c = v*s;
  float tmp = h/60.0;
  float tmp2 = tmp-2*floor(tmp/2);
  float x = c*(1-abs(tmp2-1));
  float m = v-c;
  float r,g,b;
  int i = floor(tmp);  // every 60 degrees there's a new behaviour

  switch (i) {
    case 0:
      r = c;
      g = x;
      b = 0;
      break;
    case 1:
      r = x;
      g = c;
      b = 0;
      break;
    case 2: 
      r = 0;
      g = c;
      b = x;
      break;
    case 3:
      r = 0;
      g = x;
      b = c;
      break;
    case 4:
      r = x;
      g = 0;
      b = c;
      break;
    case 5:
      r = c;
      g = 0;
      b = x;
      break;
  }
  rgb[0] = constrain((int)255*(r+m),0,255);
  rgb[1] = constrain((int)255*(g+m),0,255);
  rgb[2] = constrain((int)255*(b+m),0,255);
  return rgb;
}

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}
