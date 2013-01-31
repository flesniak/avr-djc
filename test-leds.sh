#!/bin/bash

if [ "$1" == "" ]
then DELAY="1000000" #default delay 1s
else DELAY="$[$1*1000]"
fi

WALK="TRUE" #FALSE=all leds flash constantly; TRUE=test leds consecutively

while true
do
  for i in $(seq 0 31)
  do
    echo "Testing 90$(printf '%02X' $i)7F"
    amidi -p hw:1,0,0 -S "90$(printf '%02X' $i)7F"
    if [ "$WALK" == "TRUE" ]
    then usleep $DELAY
         amidi -p hw:1,0,0 -S "80$(printf '%02X' $i)00"
    fi
  done

  usleep $DELAY
  if [ "$WALK" == "TRUE" ]
  then continue
  fi

  for i in $(seq 0 31)
  do
    amidi -p hw:1,0,0 -S "80$(printf '%02X' $i)00"
  done
  usleep $DELAY
done

