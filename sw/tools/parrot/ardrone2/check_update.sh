#!/bin/sh

UPDATE_DIR=/update
UPDATE_PATH=$UPDATE_DIR/ardrone2_update.plf
VERSION_PATH=$UPDATE_DIR/version.txt
ERR_PATH=$UPDATE_DIR/err.log

echo "Check if need to start paparazzi ..."
START_PAPARAZZI=`grep start_paparazzi /data/config.ini | awk -F "=" '{ gsub(/ */,"",$2); print $2}'`
case $START_PAPARAZZI in
1)
    START_PAPARAZZI=raw
    ;;
2)
    START_PAPARAZZI=sdk
    ;;
*)
    START_PAPARAZZI=no
    ;;
esac
echo "START_PAPARAZZI=$START_PAPARAZZI"

echo "Copy version.txt file in ftp directory"
cp /firmware/version.txt $VERSION_PATH

PELF_ARGS=$(cat /tmp/.program.elf.arguments | tr '\n' ' ')

echo "Check if update is necessary ..."
if [ -e $UPDATE_PATH ] ; then
	VERSION=`cat $VERSION_PATH`
	
	if [ -e $ERR_PATH ] ; then
		CHECK_ERR=`cat $ERR_PATH`
		if [ "$CHECK_ERR" = "NEED_TO_FLASH" ] ; then
			CHECK_PLF=`/bin/checkplf $UPDATE_PATH $VERSION`
			if [ "$CHECK_PLF" = "NEED_TO_FLASH" ] ; then
				echo "ERR=FLASH_KO" > $ERR_PATH
			else
				/bin/checkplf $UPDATE_PATH $VERSION > $ERR_PATH	
			fi
		else
			/bin/checkplf $UPDATE_PATH $VERSION > $ERR_PATH	
		fi
	else
		/bin/checkplf $UPDATE_PATH $VERSION > $ERR_PATH
	fi
	
	CHECK_ERR=`cat $ERR_PATH`
	if [ "$CHECK_ERR" = "NEED_TO_FLASH" ] ; then 
    	echo "File $UPDATE_PATH exists... Start updating..."
 		pinst_trigger
		echo "Rebooting..."
    	reboot
    else
    	if [ "$CHECK_ERR" = "VERSION_OK" ] ; then
 	    	echo "Version OK"
    	elif [ "$CHECK_ERR" = "ERR=FLASH_KO" ] ; then
 	    	echo "Error during Updating... Removing..."
    	else
 	    	echo "File $UPDATE_PATH not valid... Removing..."
    	fi
# Cleaning update directory
    	rm -Rf $UPDATE_DIR/*
			cp /firmware/version.txt $VERSION_PATH
		echo "Start Drone software..."
		inetd
		
		# Check what to start
		if [ "$START_PAPARAZZI" = "raw" ] ; then
			(/data/video/raw/ap.elf; gpio 181 -d ho 1) &
		elif [ "$START_PAPARAZZI" = "sdk" ] ; then
			(/data/video/sdk/ap.elf; gpio 181 -d ho 1) &
		else
			(/bin/program.elf ${PELF_ARGS}; gpio 181 -d ho 1) &
		fi
	fi
else
    echo "File $UPDATE_PATH doesn't exists... Start Drone software..."
# Cleaning update directory
		rm -Rf $UPDATE_DIR/*
		cp /firmware/version.txt $VERSION_PATH

    inetd
    # Check what to start
    if [ "$START_PAPARAZZI" = "raw" ] ; then
        (/bin/program.elf ${PELF_ARGS}; gpio 181 -d ho 1) &
        sleep 10
	(/data/video/raw/ap.elf; gpio 181 -d ho 1) &
    elif [ "$START_PAPARAZZI" = "sdk" ] ; then
        (/bin/program.elf ${PELF_ARGS}; gpio 181 -d ho 1) &
        sleep 10
	(/data/video/sdk/ap.elf; gpio 181 -d ho 1) &
    else
	(/bin/program.elf ${PELF_ARGS}; gpio 181 -d ho 1) &
    fi
fi
