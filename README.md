# STM32 Driver Project

---

## 하드웨어

* **MCU:** `STM32F103RBTx` (128KB Flash, 20KB RAM)
* **보드:** NUCLEO-F103RB

---

## 프로젝트 구조

```
stm32f1xx_driver/
│
├── Drivers/
│   ├── Inc/                 # 드라이버 헤더
│   │   ├── stm32f1xx.h         # MCU 레지스터/매크로
│   │   ├── stm32f1xx_gpio.h    # GPIO 헤더
│   │   ├── stm32f1xx_spi.h     # SPI 헤더
│   │   └── stm32f1xx_usart.h   # USART 헤더
│   │
│   └── Src/                 # 드라이버 소스
│       ├── stm32f1xx_gpio.c    # GPIO 구현
│       ├── stm32f1xx_spi.c     # SPI 구현
│       └── stm32f1xx_usart.c   # USART 구현
│
├── Src/                     # 메인 애플리케이션 및 예제
│   ├── 001_gpio_led_toggle.c
│   ├── ... (기타 예제 ...c)
│   ├── 011_spi_rx_it.c
│   ├── syscalls.c           # printf 리다이렉션 (ITM)
│   └── sysmem.c             
│
├── Startup/
│   └── startup_stm32f103rbtx.s # Startup 코드
│
├── Debug/                     # 빌드 관련 파일 (Makefile 등)
│   ├── makefile
│   ├── objects.list
│   ├── sources.mk
│   └── ...
│
└── STM32F103RBTX_FLASH.ld     # 링커 스크립트
```

---

## 구현된 드라이버

### 1. GPIO Driver

* **파일:** `stm32f1xx_gpio.h`, `stm32f1xx_gpio.c`
* **주요 기능:**
    * GPIO 포트 클럭 제어
    * 핀 모드 설정 (입력, 출력, 아날로그, AF)
    * 출력 타입 설정 (PP, OD)
    * 입력 풀업/풀다운 설정
    * 핀 속도 설정
    * 핀 입/출력 (읽기/쓰기/토글)
    * 외부 인터럽트(EXTI) 설정 (Rising/Falling/Both Edge)
    * NVIC 인터럽트 활성화 및 우선순위 설정
* **주요 API:** `GPIO_PeriClockControl`, `GPIO_Init`, `GPIO_DeInit`, `GPIO_ReadPin`, `GPIO_WritePin`, `GPIO_TogglePin`, `GPIO_IRQInterruptConfig`, `GPIO_IRQPriorityConfig`, `GPIO_EXTI_IRQHandler`

### 2. USART Driver

* **파일:** `stm32f1xx_usart.h`, `stm32f1xx_usart.c`
* **주요 기능:**
    * USART 클럭 제어
    * Baud Rate 자동 계산/설정
    * 전송 모드 설정 (Tx/Rx/TxRx)
    * Word Length 설정 (8/9 비트)
    * 패리티 설정 (None/Even/Odd)
    * 정지 비트 설정 (0.5/1/1.5/2)
    * 데이터 송수신 (Polling/Interrupt 방식)
    * 송수신 완료 콜백 지원 (Weak)
* **주요 API:** `USART_Init`, `USART_PeriClockControl`, `USART_PeripheralControl`, `USART_SetBaudRate`, `USART_Transmit`, `USART_Receive`, `USART_Transmit_IT`, `USART_Receive_IT`, `USART_IRQHandler`, `USART_RxCpltCallback`, `USART_TxCpltCallback`

### 3. SPI Driver

* **파일:** `stm32f1xx_spi.h`, `stm32f1xx_spi.c`
* **주요 기능:**
    * SPI 클럭 제어
    * 마스터/슬레이브 모드 설정
    * 통신 방향 설정 (2-line/1-line)
    * 데이터 크기 설정 (8/16 비트)
    * 클럭 극성(CPOL)/위상(CPHA) 설정
    * NSS 관리 (Software/Hardware)
    * Baud Rate Prescaler 설정
    * 데이터 송수신 (Polling/Interrupt 방식)
    * 송수신 완료 및 에러 콜백 지원 (Weak)
* **주요 API:** `SPI_Init`, `SPI_PeriClockControl`, `SPI_PeripheralControl`, `SPI_SSIConfig`, `SPI_SSOEConfig`, `SPI_Transmit`, `SPI_Receive`, `SPI_Transmit_IT`, `SPI_Receive_IT`, `SPI_IRQHandler`, `SPI_RxCpltCallback`, `SPI_TxCpltCallback`, `SPI_ErrorCallback`

---

## 예제 코드

`Src/` 디렉토리의 각 `.c` 파일은 특정 기능 테스트용임.

* **`001_gpio_led_toggle.c`**: GPIO 출력 (Polling) - LED 깜빡임.
* **`002_gpio_button_it.c`**: GPIO 입력 및 EXTI - 버튼 입력 시 LED 토글.
* **`003_uart_tx_test.c`**: USART 송신 (Polling) - 버튼 입력 시 문자열 전송.
* **`004_uart_rx_test.c`**: USART 수신/송신 (Polling, Echo) - 수신 데이터 그대로 송신.
* **`005_uart_tx_it_test.c`**: USART 송신 (Interrupt) - 버튼 입력 시 인터럽트 기반 전송.
* **`006_uart_rx_it_test.c`**: USART 수신 (Interrupt) - 인터럽트 기반 수신 후 Enter 입력 시 `printf` 출력.
* **`007_spi_tx_test.c`**: SPI 마스터 송신 (Polling) - 문자열 반복 전송.
* **`008_spi_tx_arduino.c`**: SPI 마스터 송신 (Arduino 통신) - 길이 정보 포함 데이터 전송.
* **`009_spi_rx_arduino.c`**: SPI 마스터 수신 (Arduino 통신) - 길이 정보 기반 데이터 수신.
* **`010_spi_rx_loopback.c`**: SPI 루프백 (Polling) - 송수신 데이터 비교
* **`011_spi_rx_it.c`**: SPI 송수신 (Interrupt, Loopback) - 버튼 입력 시 데이터 송신 후 인터럽트 기반 수신 및 결과 `printf` 출력.

---

## 빌드 및 디버깅

### 빌드 환경

* **IDE:** STM32CubeIDE

### `printf` 디버깅 (ITM/SWO)

* **구현:** `Src/syscalls.c` 에서 `_write` 시스템 콜 재정의, `ITM_SendChar` 호출.
* **확인:** STM32CubeIDE 디버그 세션 중 `SWV ITM Data Console` 사용. (SWV 활성화 및 "Start Trace" 필요)
