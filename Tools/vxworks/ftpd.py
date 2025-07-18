from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer
import os

def main():
    authorizer = DummyAuthorizer()
    # 사용자 추가 (예: 'user' 계정, 비밀번호 'pass', 쓰기 권한 부여)
    # cd "$(dirname "$0")/../../build/yp_fc-v1_default"
    CURRENT_PATH = os.path.dirname(os.path.abspath(__file__))
    BIN_PATH = os.path.join(CURRENT_PATH, '../../build/yp_fc-v1_default/bin')
    print(f"FTP Server Root: {BIN_PATH}")
    authorizer.add_user('zynq7k', '1234', BIN_PATH, perm='elradfmw')
    handler = FTPHandler
    handler.authorizer = authorizer
    handler.permit_foreign_addresses = True

    # 서버 설정
    server_address = ('', 21)  # 21번 포트 사용
    server = FTPServer(server_address, handler)

    # 서버 시작
    server.serve_forever()

if __name__ == "__main__":
    main()