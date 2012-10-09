
/*

This is a quick hack - just to show how it might work. It gives a 3D view of
the GPS positions with an additional value (here: attitude phi) as colour.

Start this with your favorite .data telemetry file as argument and pipe it
to output.dat, then run gnuplot with:

#set terminal png nocrop enhanced size 800,600
#set output 'output.png'
set view 64, 8, 1, 1
set isosamples 50, 10
set hidden3d offset 1 trianglepattern 3 undefined 1 altdiagonal bentover
set palette
set xlabel "m"
set ylabel "m"
set zlabel "m"
splot "output.dat" using 1:2:3:4 title "roll" with linespoints palette
pause -1

*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/time.h>

int main( int argc, char* argv[] )
{
/*
   <message name="GPS" ID="8">
     <field name="mode"   type="uint8" unit="byte_mask"></field>
     <field name="utm_east"  type="int32" unit="cm"></field>
     <field name="utm_north"  type="int32" unit="cm"></field>
     <field name="course" type="int16" unit="decideg"></field>
     <field name="alt"    type="int32" unit="cm"></field>
     <field name="speed"  type="uint16" unit="cm/s"></field>
     <field name="climb"  type="int16" unit="cm/s"></field>
     <field name="itow"  type="uint32" unit="ms"></field>
     <field name="utm_zone"  type="uint8"></field>
     <field name="gps_nb_err" type="uint8"></field>
   </message>

7.73 11 GPS 0 55577549 665183336 0 -4310 0 0 345957748 31 0


   <message name="ATTITUDE" ID="6">
     <field name="phi" type="int16" unit="deg"></field>
     <field name="psi" type="int16" unit="deg"></field>
     <field name="theta" type="int16" unit="deg"></field>
   </message>

230.41 23 ATTITUDE -22 0 -4

   <message name="NAVIGATION_REF" ID="0x09">
     <field name="utm_east"  type="int32" unit="m"></field>
     <field name="utm_north" type="int32" unit="m"></field>
     <field name="utm_zone" type="uint8"></field>
   </message>

7.77 23 NAVIGATION_REF 567835 5790977 32

*/

	char line_in[256];

	float rtime;
	char id_name[255];
	unsigned int ac_id;

	unsigned int byte_mask;
	int utm_east;
	int utm_north;
	int course;
	int alt;
	int speed;
	int climb;
	unsigned int itow;
	unsigned int utm_zone;
	unsigned int gps_nb_err;

	unsigned int phi;
	unsigned int psi;
	unsigned int theta;

	unsigned int nr_utm_east;
	unsigned int nr_utm_north;
	unsigned int nr_utm_zone;

	unsigned int humid;
	unsigned int temp;

	char got_navref = 0;

	FILE *fd;

	if (argc != 2)
	{
    	printf("plot3dparse 'flight_file.data' > output.dat\n");
    	exit(0);
   	}

	fd = fopen( argv[1], "r" );

	if (!fd) exit(0);

	while (!feof(fd))
	{

		fgets( line_in, 256, fd );

		if (sscanf( line_in, "%e %i %s", &rtime, &ac_id, id_name ))
		{
			if (!strcmp( "NAVIGATION_REF", id_name))
			{
				sscanf( line_in,  "%e %i %s %i %i %i\n",
					&rtime,
					&ac_id,
					id_name,
					&nr_utm_east,
					&nr_utm_north,
					&nr_utm_zone );

				got_navref = 1;
			}

#if 0
			if (!strcmp( "DPICCO_STATUS", id_name))
			{
                #define DPICCO_HUMID_MAX 0x7FFF
                #define DPICCO_HUMID_RANGE 100.0

                #define DPICCO_TEMP_MAX 0x7FFF
                #define DPICCO_TEMP_RANGE 165.0
                #define DPICCO_TEMP_OFFS -40.0

                float fhumid, ftemp;

				sscanf( line_in,  "%e %i %s %i %i %e %e\n",
					&rtime,
					&ac_id,
					id_name,
					&humid,
					&temp );

                fhumid = (dpicco_val[0] * DPICCO_HUMID_RANGE) / DPICCO_HUMID_MAX;
                ftemp = ((dpicco_val[1] * DPICCO_TEMP_RANGE) / DPICCO_TEMP_MAX) + DPICCO_TEMP_OFFS;

				if (temp != 0) printf("%f %f %f\n", rtime, fhumid, ftemp);
			}
#endif

			if (!strcmp( "GPS", id_name))
			{
				sscanf( line_in,  "%e %i %s %i %i %i %i %i %i %i %i %i %i\n",
					&rtime,
					&ac_id,
					id_name,
					&byte_mask,
					&utm_east,
					&utm_north,
					&course,
					&alt,
					&speed,
					&climb,
					&itow,
					&utm_zone,
					&gps_nb_err );

			}

			if (!strcmp( "ATTITUDE", id_name))
			{
				sscanf( line_in,  "%e %i %s %i %i %i\n",
					&rtime,
					&ac_id,
					id_name,
					&phi,
					&psi,
					&theta );

				if (got_navref) printf("%i %i %i %i\n\n",
                                       utm_east/100  - nr_utm_east,
                                       utm_north/100 - nr_utm_north,
                                       alt / 100,
                                       phi );
			}

		}
	}

	fclose( fd );

	return(0);
}
