#!/usr/bin/expect -f

set timeout 10

set host "192.168.5.61"
set prompt "->|#|%"

spawn telnet $host
expect "Connected to"
expect -re $prompt

# set clock rate to 300 Hz
send "sysClkRateSet 300\r"
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