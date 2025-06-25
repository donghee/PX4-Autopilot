sudo systemctl stop proftpd

#cd "$(dirname "$0")/../../build/yp_fc-v1_default"
#sudo python3 -m pyftpdlib -p 21 -u zynq7k -P 1234 -d bin

# allow user to write 
cd $(dirname "$0")
sudo python3 ftpd.py