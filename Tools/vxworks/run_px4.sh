#!/usr/bin/expect -f

set timeout 10

set host "192.168.5.61"
#set host "172.18.241.97"
set prompt "->|#|%"

spawn telnet $host
expect "Connected to"
expect -re $prompt

# set clock rate to 1000 Hz
send "sysClkRateSet 1000\r"
expect -re $prompt

# cmd shell
send "cmd\r"
expect -re $prompt

# cd host: 
send "cd \"host:\"\r"
expect -re $prompt

send "./px4 start\r"
expect -re $prompt

interact