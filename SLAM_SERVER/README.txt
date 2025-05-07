# Restbed ORB-SLAM3 Server Project

## 개요
이 프로젝트는 ORB-SLAM3 라이브러리의 SLAM 기능(예: 트래킹, 캘리브레이션 등)을 RESTful API로 노출하는 서버를 구현합니다. 다중 사용자가 동시에 접속하여 각자의 SLAM 세션을 사용할 수 있으며, 모든 세션은 로컬라이제이션 모드(매핑 기능 비활성화)로 동작합니다.

## 프로젝트 구조
프로젝트는 모듈식 구조로 설계되어 코드의 가독성, 유지 관리성 및 확장성이 향상되었습니다.

```
SLAM_SERVER/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── Config.h              - 설정 상수 및 환경 설정
│   ├── Utils.h               - 유틸리티 함수
│   ├── UserSession.h         - 사용자 세션 클래스
│   ├── SessionManager.h      - 세션 관리 클래스
│   ├── SlamServer.h          - 메인 서버 클래스
│   └── handlers/
│      ├── PingHandler.h      - Ping 요청 핸들러
│      ├── CalibrationHandler.h - 캘리브레이션 핸들러
│      ├── TrackingHandler.h  - 트래킹 핸들러
│      └── SessionHandler.h   - 세션 관리 핸들러
└── src/
    ├── Config.cpp            - 설정 구현
    ├── Utils.cpp             - 유틸리티 함수 구현
    ├── UserSession.cpp       - 사용자 세션 구현
    ├── SessionManager.cpp    - 세션 관리 구현
    ├── SlamServer.cpp        - 서버 구현
    ├── handlers/
    │  ├── PingHandler.cpp     - Ping 핸들러 구현
    │  ├── CalibrationHandler.cpp - 캘리브레이션 핸들러 구현
    │  ├── TrackingHandler.cpp - 트래킹 핸들러 구현
    │  └── SessionHandler.cpp  - 세션 핸들러 구현
    └── main.cpp              - 메인 진입점
```

## API 엔드포인트

1. `/ping` (GET)
   - 서버 연결 확인. 클라이언트에 "pong" 문자열 응답.

2. `/calibration` (POST)
   - JSON 형식 데이터(선박 id, 사용자명, slam_type, calibration_parameters 등)를 수신.
   - 지원 slam_type: mono, mono+imu.
   - SLAM 시스템 초기화 후, 고유 세션 ID를 생성하여 반환 (이후 요청 시 해당 세션 ID 사용).

3. `/tracking` (POST)
   - 클라이언트로부터 전송된 이미지 데이터를 디코딩 후 SLAM 트래킹 수행.
   - 쿼리 파라미터로 전달된 세션 ID에 해당하는 SLAM 인스턴스 사용.
   - 처리 후 카메라 자세(translation, quaternion)를 JSON 형식으로 응답.

4. `/sessions` (GET)
   - 현재 활성화된 모든 세션 목록 및 정보를 반환.

5. `/session/close` (POST)
   - 특정 세션 종료 및 관련 리소스 정리.
   - 쿼리 파라미터로 세션 ID 전달.

6. `/shutdown` (POST)
   - 서버 종료 요청. 호출 시 모든 세션 정리 후 서버 종료.

## 환경 세팅 및 종속성 설치

### 1. 운영체제 및 컴파일러
- Ubuntu Linux (18.04 LTS, 20.04 LTS 등) 또는 C++17 지원 OS
- GCC 또는 Clang (C++17 이상 지원)
- CMake 버전 3.10 이상

### 2. 필수 라이브러리

#### 2.1 OpenCV
```bash
# Ubuntu:
sudo apt update
sudo apt upgrade
sudo apt install libopencv-dev
```

#### 2.2 Restbed
```bash
# Ubuntu (패키지 설치):
su