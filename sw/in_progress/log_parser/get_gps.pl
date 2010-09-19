#!/usr/bin/perl
#Author: Paul Cox
#This script reads a paparazzi log file and extracts the GPS messages
#It outputs Time/Lat/Long/Alt
#TODO: NMEA output, add speed
#Notes:
#mode 
#utm_east ALT_UNIT="m" UNIT="cm"
#utm_north ALT_UNIT="m" UNIT="cm"
#course ALT_UNIT="deg" UNIT="decideg"
#alt ALT_UNIT="m" UNIT="cm"
#speed ALT_UNIT="m/s" UNIT="cm/s"
#climb ALT_UNIT="m/s" UNIT="cm/s"
#week weeks
#itow ms
#utm_zone
#gps_nb_err
#0       1  2   3 4        5         6    7    8 9   10   11        12 13
#time    ID MSG M EAST     NORTH     C    ALT  S C   W    ITOW      ZO ERR
#144.225 20 GPS 3 19779772 497668512 1819 3625 9 -20 1601 303393500 31 0


use Geo::Coordinates::UTM;

sub getnmeatime {
    my $utw_h = 0;
    my $utw_d = 0;
    my $foo = $utw/60000/60;
    #calculate days and hours
    while ($foo > 1) {
      if ($utw_h == 23) {$utw_h = 0;$utw_d++;} else {$utw_h = $utw_h + 1;}
      $foo = $foo - 1;
    }
    #ensure proper leading zero for single digits
    $utw_h = sprintf("%02d",$utw_h);
    #foo+1 is the fractional hours
    my $utw_m = int($foo*60);
    #ensure proper leading zero for single digits
    $utw_m = sprintf("%02d",$utw_m);
    #calculate remaining seconds
    my $utw_s = sprintf("%06.3f",$utw/1000-$utw_m*60-$utw_h*60*60-$utw_d*60*60*24);
    my $time = $utw_h . $utw_m . $utw_s; #Time UTC HHMMSS.mmm  303318000/60000=5055.3/60=84.255/24=3.510625
    return $time;
} 

open INFILE, "<10_09_15__14_13_55.data" or die $!;
open OUTFILE, ">GPS_data.txt" or die $!;
open NMEAFILE, ">NMEA.log" or die $!;

$cnt=0;
$utw=0;
$toffset=0;
$solstart=0;
$totime=0;
$tdtime=0;
$hialt=0;

while (my $line = <INFILE>) {
  chomp($line); 
  @fields = split(/ /,$line);
  
  #Determine when GPS fix is acquired by looking for PDOP <1000 and numSV>3
  if ($fields[2] eq "GPS_SOL" and $fields[5] < 1000 and $fields[6] > 3) {
    if ($solstart == 0 ) {
      $solstart = $fields[0];
      printf "GPS SOL start time: $solstart\n";
    }
    $pacc = $fields[3];
    $sacc = $fields[4];
    $pdop = $fields[5];
    $numSV = $fields[6];
  }
  
  #We are going to look for GPS messages, in mode 3 (3D fix)
  # Skip messages that have the previous utw (duplicates)
  # Skip any messages with negative altitude (GPS not initialized yet)
  if ($fields[2] eq "GPS" and $fields[3] == "3" and $fields[11] != $utw and $fields[7] > 0 and $solstart != 0) {
  
    #takeoff is considered to be > 4 m/s on hor and vert
    if ($fields[8] > 400 and $fields[9] > 400 and $totime == 0) {
      $totime = $fields[0];
      my $gndalt = $fields[7];
      printf "Takeoff detected at time : $totime s\n";
    }
    
    if ($fields[7] > $hialt and $totime != 0 ) {
      $hialt = $fields[7];
      $hialtt = $fields[0];
    }
  
    #touchdown is considered when < 1 m/s on hor and vert
    if ($fields[8] < 100 and $fields[9] < 100 and $totime != 0 and $tdtime == 0) {
      $tdtime = $fields[0];
      printf "Highest Alt : %.2f ($hialtt)\n",($hialt-$gndalt)/100;
      printf "Touchdown detected at time : $tdtime s (flight time: %.2f min)\n",($tdtime-$totime)/60;
    }
  
    #store begin flight time
    if ($toffset == 0) { 
      $toffset = $fields[0];
      printf "GPS Start Time: $toffset\n";
    }
    #Calculate delta and store for averaging at the end
    $delta= $fields[0]-$prevtime;
    $prevtime = $fields[0];
    $sum += $delta;
    
    $utw=$fields[11];
    printf OUTFILE "Time: ";
    printf OUTFILE '%.2f',$fields[0] - $toffset;
#    printf OUTFILE " utme: $fields[4] utmn: $fields[5]";
    printf OUTFILE " Alt: ";
    printf OUTFILE '%.2f',($fields[7]/100);
    my $zone = $fields[12] . "V";
    #divide by 100 as gps provides utm in centimeters
    my ($latitude,$longitude)=utm_to_latlon('wgs84',$zone,$fields[4]/100,$fields[5]/100);
    printf OUTFILE " Lat: ";
    printf OUTFILE '%.6f',$latitude ;
    printf OUTFILE " Lon: \ ";
    printf OUTFILE '%.6f',$longitude;
    printf OUTFILE "\n";
    
    if ($totime == 0) { next;}
    if ($tdtime != 0) { break;}
    #Begin NMEA output
    #RMC,GGA/GSA/VTG/GSV
    printf NMEAFILE "\$GPRMC,";

    $time = getnmeatime($utw);

    printf NMEAFILE $time . ",A,";    #Active

    $degrees = abs(int($latitude));
    $minutes = (abs($latitude) - $degrees) * 60;
    $nmealat = sprintf("%02d%.4f",$degrees,$minutes). "," ."N";                               
    printf NMEAFILE $nmealat . ","; #LAT

    $degrees = abs(int($longitude));
    $minutes = (abs($longitude) - $degrees) * 60;
    $nmealon = sprintf("%03d%.4f",$degrees,$minutes);  
    if ($longitude > 0) {
      $nmealon .= ",E" ; #LON
    } else {
      $nmealon .= ",W" ; #LON
    }
    printf NMEAFILE $nmealon . ",";

    printf NMEAFILE '%.2f,',$fields[8]*.019438444; #gnd spd in knts from cm/s 
    printf NMEAFILE '%.2f,',$fields[6]/10; #trk angle in deg from decideg

    # ($year,$month,$day) = Monday_of_Week($week,"2010");
    # print
    printf NMEAFILE "150910" . ",,\n"; #date and mag var TODO:use real date
    $cnt++;
    
    #$GPRMC,121518.000,A,4452.767,N,00049.573,W,0.45,0.00,150910,,*1F
    #$GPGGA,121518.000,4452.767,N,00049.573,W,1,00,0.0,0.000,M,0.0,M,,*7D
    #$GPVTG,0.000,T,0,M,0.450,N,0.833,K*59
    printf NMEAFILE "\$GPGGA,$time,$nmealat,$nmealon,1,$numSV,$pdop,%.2f,M,%.2f,M,,\n",$fields[7]/100,$fields[7]/100;
    printf NMEAFILE "\$GPVTG,0.000,T,0,M,%.2f,N,%.2f,K\n",$fields[8]*.019438444,$fields[8]*.03598272;  
  }
}

printf "Number of GPS points: $cnt Avg. delta: ";
printf '%.2f',$sum/$cnt;
printf " Duration: ";
printf '%.2f',$cnt/4/60;
printf " minutes\n";

close INFILE;
close OUTFILE;

open NMEAFILE, "<NMEA.log" or die $!;
open OUTFILE, ">gps.nmea" or die $!;


while (my $line = <NMEAFILE>) { 
  chomp($line);
  #hack off CR
  $len = length($line);
  $cksum = 0;
  printf OUTFILE $line . "*";
  #hack off $
  $line = substr($line,1,$len);  
  for ( split(//, $line) ) { $cksum ^= ord($_); };
  
  printf OUTFILE '%02x',$cksum ;
  printf OUTFILE "\n";
}

close NMEAFILE;
close OUTFILE;

   