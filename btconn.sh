
dev_id=00:12:06:18:10:73

dev_no=0
dev=/dev/rfcomm$dev_no

## find device
#sudo hcitool scan
#sudo hcitool scan

## bind to rfcomm device
sudo rfcomm bind $dev_no $dev_id
