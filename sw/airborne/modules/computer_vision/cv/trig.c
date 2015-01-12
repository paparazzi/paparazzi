/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Trigonometry
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "trig.h"

static int cosine[] = {
  10000, 9998, 9994, 9986, 9976, 9962, 9945, 9925, 9903, 9877,
  9848, 9816, 9781, 9744, 9703, 9659, 9613, 9563, 9511, 9455,
  9397, 9336, 9272, 9205, 9135, 9063, 8988, 8910, 8829, 8746,
  8660, 8572, 8480, 8387, 8290, 8192, 8090, 7986, 7880, 7771,
  7660, 7547, 7431, 7314, 7193, 7071, 6947, 6820, 6691, 6561,
  6428, 6293, 6157, 6018, 5878, 5736, 5592, 5446, 5299, 5150,
  5000, 4848, 4695, 4540, 4384, 4226, 4067, 3907, 3746, 3584,
  3420, 3256, 3090, 2924, 2756, 2588, 2419, 2250, 2079, 1908,
  1736, 1564, 1392, 1219, 1045,  872,  698,  523,  349,  175,
  0
};

int sin_zelf(int ix)
{
  while (ix < 0) {
    ix = ix + 360;
  }
  while (ix >= 360) {
    ix = ix - 360;
  }
  if (ix < 90) { return cosine[90 - ix] / 10; }
  if (ix < 180) { return cosine[ix - 90] / 10; }
  if (ix < 270) { return -cosine[270 - ix] / 10; }
  if (ix < 360) { return -cosine[ix - 270] / 10; }
  return 0;
}

int cos_zelf(int ix)
{
  while (ix < 0) {
    ix = ix + 360;
  }
  while (ix >= 360) {
    ix = ix - 360;
  }
  if (ix < 90) { return cosine[ix] / 10; }
  if (ix < 180) { return -cosine[180 - ix] / 10; }
  if (ix < 270) { return -cosine[ix - 180] / 10; }
  if (ix < 360) { return cosine[360 - ix] / 10; }
  return 0;
}

int tan_zelf(int ix)
{

  while (ix < 0) {
    ix = ix + 360;
  }
  while (ix >= 360) {
    ix = ix - 360;
  }
  if (ix == 90) { return 9999; }
  if (ix == 270) { return -9999; }
  if (ix < 90) { return (1000 * cosine[90 - ix]) / cosine[ix]; }
  if (ix < 180) { return -(1000 * cosine[ix - 90]) / cosine[180 - ix]; }
  if (ix < 270) { return (1000 * cosine[270 - ix]) / cosine[ix - 180]; }
  if (ix < 360) { return -(1000 * cosine[ix - 270]) / cosine[360 - ix]; }
  return 0;
}

int asin_zelf(int y, int hyp)
{
  int quot, sgn, ix;
  if ((y > hyp) || (y == 0)) {
    return 0;
  }
  sgn = hyp * y;
  if (hyp < 0) {
    hyp = -hyp;
  }
  if (y < 0) {
    y = -y;
  }
  quot = (y * 10000) / hyp;
  if (quot > 9999) {
    quot = 9999;
  }
  for (ix = 0; ix < 90; ix++)
    if ((quot < cosine[ix]) && (quot >= cosine[ix + 1])) {
      break;
    }
  if (sgn < 0) {
    return -(90 - ix);
  } else {
    return 90 - ix;
  }
}

int acos_zelf(int x, int hyp)
{
  int quot, sgn, ix;
  if (x > hyp) {
    return 0;
  }
  if (x == 0) {
    if (hyp < 0) {
      return -90;
    } else {
      return 90;
    }
    return 0;
  }
  sgn = hyp * x;
  if (hyp < 0) {
    hyp = -hyp;
  }
  if (x < 0) {
    x = -x;
  }
  quot = (x * 10000) / hyp;
  if (quot > 9999) {
    quot = 9999;
  }
  for (ix = 0; ix < 90; ix++)
    if ((quot < cosine[ix]) && (quot >= cosine[ix + 1])) {
      break;
    }
  if (sgn < 0) {
    return -ix;
  } else {
    return ix;
  }
}

//atan_zelf(y/x) in degrees
int atan_zelf(int y, int x)
{
  int angle, flip, t, xy;

  if (x < 0) { x = -x; }
  if (y < 0) { y = -y; }
  flip = 0;
  if (x < y) { flip = 1; t = x;  x = y;  y = t; }
  if (x == 0) { return 90; }

  xy = (y * 1000) / x;
  angle = (360 * xy) / (6283 + ((((1764 * xy) / 1000) * xy) / 1000));
  if (flip) { angle = 90 - angle; }
  return angle;
}

unsigned int isqrt(unsigned int val)
{
  unsigned int temp, g = 0, b = 0x8000, bshft = 15;
  do {
    if (val >= (temp = (((g << 1) + b) << bshft--))) {
      g += b;
      val -= temp;
    }
  } while (b >>= 1);
  return g;
}
