echo "Warning: Did you commit first?"
echo "Updating TUDelft Paparazzi Version"
svn update ./
rm -vrf ./conf/conf.xml.*-*
echo "Cleaning and Updating ENAC Paparazzi Version"
cd ../paparazzi3/
svn revert -R ./
svn status | sed 's/?   /rm -rf /' > rm.sh
chmod +x ./rm.sh
./rm.sh
svn update ./
cd ../ppz2svn/
echo "Exporting ENAC to TUDelft Paparazzi Version"
svn export ../paparazzi3/ ./ --force
echo "Adding new Files"
svn status | grep '? ' | sed 's/?      /svn add/' > add.sh
chmod +x ./add.sh
./add.sh
svn remove ./add.sh --force
svn remove ./tud.txt --force
# rm -f ./add.sh
echo "Removing Deleted Files"
svn export ./ ../paparazzi3/ --force
cd ../paparazzi3/
svn status | grep 'TUDelft' | sed 's/?     /rm -rf /' > rm.sh
chmod +x ./rm.sh
./rm.sh
rm -rf ./import.sh
rm -rf ./export.sh
rm -rf ./sw/airborne/modules/onboardcam
rm -rf ./sw/airborne/modules/opticflow
rm -rf ./sw/airborne/modules/asset_kite_actuator
rm -rf ./sw/airborne/modules/asset_kite_sensors
rm -rf ./sw/airborne/modules/bmp085
rm -rf ./sw/airborne/boards/tiny_sense.h
rm -rf ./sw/airborne/booz/aerovinci
rm -rf ./conf/simulator/jsbsim/aircraft/LISA_BOOZ_BART.xml
echo "Check this list manually: what files need to be added to ENAC server?" > ./rm.sh
svn status | sed 's/?       /svn remove ..\/ppz2svn\//' >> rm.sh
chmod +x ./rm.sh
gedit ./rm.sh
#svn status | sed 's/?    /rm -rf /' > ./rm.sh
cd ../ppz2svn
svn diff > tud.txt
gedit tud.txt
