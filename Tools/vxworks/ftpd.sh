cd "$(dirname "$0")/../../build/yp_fc-v1_default"
sudo systemctl stop proftpd
sudo python3 -m pyftpdlib -p 21 -u zynq7k -P 1234 -d bin