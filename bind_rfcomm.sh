#!/bin/bash

dev_dst=/dev/rfcomm0
dev_num=0
addr=00:12:06:18:10:73

## to check device
#sudo hcitool scan
#sudo hcitool info 00:12:06:18:10:73
#sudo l2ping -c 4 00:12:06:18:10:73 

sudo rfcomm bind $dev_num $addr

## another (can't get goot result )
##sudo rfcomm listen hci0 0
##sudo rfcomm connect 0
