#!/bin/sh

PLANES="FLYINGWING ARCUSSPORT"

for PLANE in $PLANES ; do

PITCH=XXXXX
RADIUS=XXXXX
THROTTLE=XXXXX
case $PLANE in
FLYINGWING)
  PITCH=45
  RADIUS=-70
  THROTTLE=0.80
  ;;
ARCUSSPORT)
  PITCH=25
  RADIUS=-90
  THROTTLE=1.00
  ;;
esac

# PROFIL-HOEHE
HGTS="1000 0800 0500 0300"
for HGT in $HGTS ; do

NAMES="WRF_W WRF_N WRF_E WRF_S WRF_C FMN1_CN FMN2_W FMN2_E FMN3_NW FMN3_S FMN3_NE" # FMN4_NW FMN4_NE FMN4_SE FMN4_SW"
for NAME in $NAMES ; do

FILE=ASTAB_ScaleX_${HGT}_${NAME}_${PLANE}.xml
echo $FILE

ALT=600
LON=XXX
LAT=XXX
ANX=-150
ANY=0
ANH=15
HOMEX=XXX
HOMEY=XXX
case $NAME in
WRF_W)
  LON=11.05443341
  LAT=47.83191124
  ALT=630
  ANX=150
  ANY=150
  HOMELON=11.055026
  HOMELAT=47.832978
  ;;
WRF_N)
  LON=11.06125878
  LAT=47.83644877
  HOMELON=11.060113
  HOMELAT=47.836848
  ANX=0
  ANY=-150
  ;;
WRF_E)
  LON=11.06783629
  LAT=47.83179685
  ANX=0
  ANY=150
  HOMELON=11.066523
  HOMELAT=47.831212
  ;;
WRF_S)
  LON=11.06093467
  LAT=47.82744997
  ALT=616
  HOMELON=11.060476
  HOMELAT=47.827361
  ANX=-150
  ANY=0
  ;;
WRF_C)
  LON=11.06116345
  LAT=47.83196844
  HOMELON=11.058271
  HOMELAT=47.831780
  ANX=0
  ANY=150
  ;;
FMN1_CN)
  LON=11.06114936
  LAT=47.83194781
  HOMELON=11.061623
  HOMELAT=47.830809
  ANX=-100
  ANY=0
  ;;
FMN2_W)
  LON=11.05950962 
  LAT=47.83228396
  ANX=-100
  ANY=0
  HOMELON=11.058654
  HOMELAT=47.832972
  ;;
FMN2_E)
  LON=11.06229718 
  LAT=47.83226756
  ANX=100
  ANY=0
  HOMELON=11.061651
  HOMELAT=47.833210
  ;;
FMN3_NW)
  LON=11.0592718 
  LAT=47.83299724
  ANX=0
  ANY=100
  HOMELON=11.058655
  HOMELAT=47.832944
  ;;
FMN3_S)
  LON=11.06109197
  LAT=47.83162806
  HOMELON=11.061623
  HOMELAT=47.830809
  ANX=-100
  ANY=0
  ;;
FMN3_NE)
  LON=11.06223979
  LAT=47.83313662
  ANX=80
  ANY=-150
  HOMELON=11.061651
  HOMELAT=47.833210
  ;;
esac

DEFALT=`expr $ALT + 50`

echo '<!DOCTYPE flight_plan SYSTEM "flight_plan.dtd">' >$FILE
echo '<flight_plan alt="'${DEFALT}'" ground_alt="'${ALT}'" lat0="'${HOMELAT}'" lon0="'${HOMELON}'" max_dist_from_home="500" name="ScaleX_" security_height="30">' >>$FILE
echo '<header>' >>$FILE
echo '#include "subsystems/datalink/datalink.h"' >>$FILE
echo '</header>' >>$FILE
echo '' >>$FILE
echo '  <waypoints>'>>$FILE
echo '    <waypoint name="HOME" lon="'${HOMELON}'" lat="'${HOMELAT}'" height="40"/>' >>$FILE
echo '   <!-- BITTE NUR PROFIL UND ANFLUG AENDERN (PROFIL-KOORD. VON UNTEN RAUSKOPIEREN) -->' >>$FILE
echo '    <waypoint name="Profil" lon="'${LON}'" lat="'${LAT}'"/>' >>$FILE
echo '    <waypoint name="Anflug" x="'${ANX}'" y="'${ANY}'" height="30"/>' >>$FILE
echo '' >>$FILE
echo '    <waypoint name="WRF_W" lon="11.05443341" lat="47.83191124"/> <!--623m-->' >>$FILE
echo '    <waypoint name="WRF_N" lon="11.06125878" lat="47.83644877"/>' >>$FILE
echo '    <waypoint name="WRF_E" lon="11.06783629" lat="47.83179685"/>' >>$FILE
echo '    <waypoint name="WRF_S" lon="11.06093467" lat="47.82744997"/> <!--616m-->' >>$FILE
echo '    <waypoint name="WRF_C" lon="11.06116345" lat="47.83196844"/>' >>$FILE
echo '' >>$FILE
echo '    <!-- FMN Feuchtemessnetz 1 Profil -->' >>$FILE
echo '    <waypoint name="FMN1_CN" lon="11.06114936" lat="47.83194781"/>' >>$FILE
echo '' >>$FILE
echo '    <!-- FMN Feuchtemessnetz 2 Profile -->' >>$FILE
echo '    <waypoint name="FMN2_W" lon="11.05950962" lat="47.83228396"/>' >>$FILE
echo '    <waypoint name="FMN2_E" lon="11.06229718" lat="47.83226756"/>' >>$FILE
echo '' >>$FILE
echo '    <!-- FMN Feuchtemessnetz 3 Profile -->' >>$FILE
echo '    <waypoint name="FMN3_NW" lon="11.05927186" lat="47.83299724"/>' >>$FILE
echo '    <waypoint name="FMN3_S" lon="11.06109197" lat="47.83162806"/>' >>$FILE
echo '    <waypoint name="FMN3_NE" lon="11.06223979" lat="47.83313662"/>' >>$FILE
echo '' >>$FILE
echo '   <!-- FMN Feuchtemessnetz 3 Profile -->' >>$FILE
echo '    <waypoint name="FMN4_NW" lon="11.05928826" lat="47.83300544"/>' >>$FILE
echo '   <waypoint name="FMN4_NE" lon="11.06224799" lat="47.83312842"/>' >>$FILE
echo '    <waypoint name="FMN4_SE" lon="11.06232178" lat="47.83157067"/>' >>$FILE
echo '    <waypoint name="FMN4_SW" lon="11.05958341" lat="47.83155427"/>' >>$FILE
echo '' >>$FILE
echo '  </waypoints>' >>$FILE
echo '  <blocks>' >>$FILE
echo '' >>$FILE
echo '    <block name="Waiting for GPS fix">' >>$FILE
echo '      <!-- <set value="1" var="kill_throttle"/> -->' >>$FILE
echo '      <while cond="!GpsFixValid()"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Geo init">' >>$FILE
echo '      <while cond="LessThan(NavBlockTime(), 10)"/>' >>$FILE
echo '      <!-- <call fun="NavSetGroundReferenceHere()"/> -->' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Waiting for Remote Control">' >>$FILE
echo '      <while cond="RCLost()"/> ' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Unlock throttle">' >>$FILE
echo '      <set value="1" var="launch"/>' >>$FILE
echo '      <set value="0" var="kill_throttle"/>' >>$FILE
echo '      <set value="0" var="autopilot_flight_time"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Climb">' >>$FILE
echo '<!-- HIER HOEHE EINTRAGEN ! -->' >>$FILE
echo '      <circle wp="Profil" pitch="'${PITCH}'" radius="'${RADIUS}'" throttle="'${THROTTLE}'" until="GetPosAlt() > ground_alt+'${HGT}'" vmode="throttle"/>' >>$FILE
echo '      <deroute block="Descent"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Descent">' >>$FILE
echo '      <circle wp="Profil" climb="-2.0" radius="-70" throttle="0.30" pitch="auto" until="ground_alt+35 > GetPosAlt()" vmode="climb"/>' >>$FILE
echo '<!-- HIER HOEHE EINTRAGEN ! -->' >>$FILE
echo '      <exception cond="GetPosAlt() > ground_alt + '${HGT}' + 50 " deroute="Down"/>' >>$FILE
echo '      <deroute block="Landing"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Down">' >>$FILE
echo '     <circle wp="Profil" radius="-70" until="ground_alt+35 > GetPosAlt()" vmode="alt" alt="ground_alt+40"/>' >>$FILE
echo '      <deroute block="Landing"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Landing">' >>$FILE
echo '      <while cond="TRUE">' >>$FILE
echo '      <go wp="Anflug" approaching_time="1"/>' >>$FILE
echo '      <go from="Anflug" hmode="route" vmode="glide" wp="HOME" alt="ground_alt+'${ANH}'" approaching_time="1"/>' >>$FILE
echo '      </while>' >>$FILE
echo '      <deroute block="Standby"/>' >>$FILE
echo '    </block>' >>$FILE
echo '' >>$FILE
echo '    <block name="Standby" group="home" key="Ctrl+a" strip_button="Standby" strip_icon="home.png">' >>$FILE
echo '      <circle radius="-70" wp="HOME"/>' >>$FILE
echo '    </block>' >>$FILE
echo ' ' >>$FILE
echo '  </blocks>' >>$FILE
echo '</flight_plan>' >>$FILE

done
done
done # PLANES
