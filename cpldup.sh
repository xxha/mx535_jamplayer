#!/bin/sh

if [ "$#" != "1" ]; then
	echo "Please specify upgrade file, as follow:"
	echo "        cpldup filename"
	exit 1
fi

jamplayer -aprogram -ddo_real_time_isp=1 $1

if [ "$?" != "0" ]; then
	echo "cpld upgrade failed"
	exit 1
fi

