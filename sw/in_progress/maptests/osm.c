/*
    http://modestmaps.mapstraction.com/trac/wiki/TileNamingConventions
    http://www.maptiler.org/google-maps-coordinates-tile-bounds-projection/
    http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames3

    gcc osm.c -lm -Wall -o osm
*/

#include <stdio.h>
#include <math.h>

int long2tile(double lon, int z) 
{ 
  return (int)(floor((lon + 180.0) / 360.0 * pow(2.0, z))); 
}
  
int lat2tile(double lat, int z)
{ 
  return (int)(floor((1.0 - log( tan(lat * M_PI/180.0) + 1.0 / cos(lat * M_PI/180.0)) / M_PI) / 2.0 * pow(2.0, z))); 
}

void gm_quadtree(int x, int y, int z, char *buffer)
{
    static const char *const quadrant = "qrts";
    int i;
    char *ptr = buffer;

    *ptr++ = 't';
    for (i = z-1; i >= 0; i--)
    {
        int xbit = (x >> i) & 1;
        int ybit = (y >> i) & 1;
        *ptr++ = quadrant[xbit + 2 * ybit];
    }
    *ptr++ = '\0';
}

void ms_quadtree(int x, int y, int z, char *buffer)
{
  int i;
  char *ptr = buffer;
  
  for (i = z; i > 0; i--)
  {
    int mask = 1 << (i - 1);
    char digit = '0';    
    if ((x & mask) != 0)
    {
      digit+=1;
    }
    if ((y & mask) != 0)
    {
      digit+=2;
    }
    *ptr++ = digit;
  }
  *ptr++ = '\0';
}

int main(void)
{
  int x, y, z = 14;
  double lat = 52.26483, lon = 9.99394;
  char ms_qkey[32];
  char gm_qkey[32];

  x = long2tile(lon, z);
  y = lat2tile(lat, z);
  ms_quadtree(x, y, z, ms_qkey);
  gm_quadtree(x, y, z, gm_qkey);
  
  printf("http://tile.openstreetmap.org/%d/%d/%d.png\n", z, x, y);
  printf("http://khm0.google.com/kh/v=45&x=%d&s=&y=%d&z=%d\n", x, y, z);
//  printf("http://kh.google.com/kh?v=3&t=%s\n", gm_qkey);
//  printf("http://mt1.google.com/mt/v=ap.95&hl=en&x=%d&y=%d&z=%d&s=G\n", x, y, z);
  printf("http://a0.ortho.tiles.virtualearth.net/tiles/a%s.jpeg?g=%d\n", ms_qkey, z+32);
  printf("http://r0.ortho.tiles.virtualearth.net/tiles/r%s.png?g=%d\n", ms_qkey, z+32);
  
  return(0);
};
