#!/usr/bin/expect -f

# 시리얼 포트를 통해 picocom을 시작합니다.
spawn sudo picocom /dev/ttyUSB0 -b 115200 --imap lfcrlf

expect {
    "Terminal ready" {
        send "\r"
    }
    timeout {
        puts "picocom을 시작하는 데 실패했습니다."
        exit 1
    }
}

# U-Boot 프롬프트를 기다립니다.
expect {
    "zynq-uboot>" {
        # U-Boot 프롬프트에 도달한 후 실행할 명령으로 로그인 자동화
        send "printenv\r"
    }
    timeout {
        puts "U-Boot 프롬프트를 기다리다가 시간 초과했습니다."
        exit 1
    }
}

# U-Boot 다음 명령어를 실행한 후 결과를 기다립니다.
# setenv bootargs 'gem(0,0)host:vxWorks h=192.168.5.60 e=192.168.5.61:ffffff00 u=zynq7k pw=1234 f=0x08'
# setenv ipaddr '192.168.5.61'
# setenv serverip '192.168.5.60'
# tftpboot 0x05000000 uVxWorks; tftpboot 0x04000000 zynq-zc702.dtb; bootm 0x05000000 - 0x04000000

expect {
    "zynq-uboot>" {
        send "setenv ipaddr '192.168.5.61'\r"
        send "setenv serverip '192.168.5.60'\r"
        send "tftpboot 0x05000000 uVxWorks; tftpboot 0x04000000 zynq-zc702.dtb; bootm 0x05000000 - 0x04000000\r"
      }
    timeout {
        puts "명령어 실행을 기다리다가 시간 초과했습니다."
        exit 1
      }
}

interact