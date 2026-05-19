# BearDriverSTM 소프트웨어 상세 설계 사양서

> **문서번호**: SDD-BEARDRV-STM32-002
> **프로젝트**: BearDriverSTM - 듀얼 BLDC/PMSM 모터 드라이브 컨트롤러
> **MCU**: STM32G474VETx (ARM Cortex-M4F @ 170MHz, LQFP100)
> **펌웨어 버전**: 7.0.0.0
> **플랫폼**: STM32G474
> **작성일**: 2026-04-24
> **원본**: TI C2000 BearDriver → STM32G474 포팅 (BearDriverSTM_EV 기반 확장)

---

## 목차

1. [개요](#1-개요)
2. [시스템 아키텍처](#2-시스템-아키텍처)
3. [하드웨어 구성](#3-하드웨어-구성)
4. [소프트웨어 모듈 상세 설계](#4-소프트웨어-모듈-상세-설계)
   - 4.1 [상태머신 모듈 (bear_driver)](#41-상태머신-모듈-bear_driver)
   - 4.2 [모터 제어 모듈 (bldc_motor)](#42-모터-제어-모듈-bldc_motor)
   - 4.3 [PID 제어기 모듈 (pid)](#43-pid-제어기-모듈-pid)
   - 4.4 [인코더 모듈 (encoder)](#44-인코더-모듈-encoder)
   - 4.5 [홀 센서 모듈 (hall_sensor)](#45-홀-센서-모듈-hall_sensor)
   - 4.6 [궤적 생성 모듈 (traj)](#46-궤적-생성-모듈-traj)
   - 4.7 [비상정지 모듈 (EStop_SCurve)](#47-비상정지-모듈-estop_scurve)
   - 4.8 [통신 모듈 (sci_coms)](#48-통신-모듈-sci_coms)
   - 4.9 [API 모듈 (api)](#49-api-모듈-api)
   - 4.10 [SPI FRAM 모듈 (spi_flash)](#410-spi-fram-모듈-spi_flash)
   - 4.11 [소프트웨어 타이머 모듈 (timers)](#411-소프트웨어-타이머-모듈-timers)
   - 4.12 [차동 구동 제한기 모듈 (differential_drive_limiter)](#412-차동-구동-제한기-모듈-differential_drive_limiter)
   - 4.13 [CRC 모듈 (crc)](#413-crc-모듈-crc)
   - 4.14 [버전 관리 모듈 (version)](#414-버전-관리-모듈-version)
   - 4.15 [디버그 모듈 (debug)](#415-디버그-모듈-debug)
   - 4.16 [ISR 실행 시간 측정 모듈 (cpu_time)](#416-isr-실행-시간-측정-모듈-cpu_time)
5. [데이터 구조 및 자료형](#5-데이터-구조-및-자료형)
6. [인터럽트 설계](#6-인터럽트-설계)
7. [실행 컨텍스트 및 타이밍](#7-실행-컨텍스트-및-타이밍)
8. [통신 프로토콜 상세](#8-통신-프로토콜-상세)
9. [안전 기능 설계](#9-안전-기능-설계)
10. [비휘발성 저장 설계](#10-비휘발성-저장-설계)
11. [파라미터 정의](#11-파라미터-정의)
12. [파일 구조 및 빌드](#12-파일-구조-및-빌드)
- [부록 A: TI → STM32 포팅 변경 이력](#부록-a-ti--stm32-포팅-변경-이력)
- [부록 B: 에러 코드 비트맵](#부록-b-에러-코드-비트맵)
- [부록 C: 모터 상태 워드 구조](#부록-c-모터-상태-워드-구조)
- [부록 D: TI C2000 vs STM32G474 전류 제어기 비교 분석](#부록-d-ti-c2000-vs-stm32g474-전류-제어기-비교-분석)
- [부록 E: TI C2000 vs STM32G474 속도 제어기 비교 분석](#부록-e-ti-c2000-vs-stm32g474-속도-제어기-비교-분석)

---

## 1. 개요

### 1.1 문서 목적

본 문서는 BearDriverSTM 듀얼 BLDC/PMSM 모터 드라이브 컨트롤러 펌웨어의 소프트웨어 상세 설계를 기술한다. BearDriverSTM_EV(SDD-BEARDRV-STM32-001) 아키텍처를 기반으로, 듀얼 모터 독립 제어(TIM1 + TIM20)와 확장된 ADC 5채널 구성을 적용한 버전이다.

### 1.2 시스템 개요

BearDriverSTM은 서비스 로봇 구동용 듀얼 BLDC/PMSM 모터 제어 펌웨어이다. BearDriverSTM_EV 대비 주요 차이점:

- **듀얼 PWM 타이머**: TIM1(Motor1) + TIM20(Motor2) 독립 PWM, 반주기 오프셋 인터리빙
- **5개 ADC**: ADC3/ADC4(Motor1 전류) + ADC1/ADC2(Motor2 전류) + ADC5(Vbus/Ibus)
- **3개 USART**: USART1(Base) + USART2(Debug) + USART3(RS485)
- **독립 게이트 드라이버**: 각 모터별 STDRIVE102BH (nSTBY/FAULT/FLAG)
- **CAN 버스**: FDCAN1 (PA11/PA12) 하드웨어 준비

**핵심 사양:**

| 항목 | 사양 |
|------|------|
| MCU | STM32G474VETx (ARM Cortex-M4F, FPU, LQFP100) |
| 시스템 클럭 | 170 MHz (HSI 16MHz → PLL) |
| PWM 주파수 | 10 kHz (Center-aligned) |
| FOC 루프 | 10 kHz (100 μs) |
| 속도 루프 | 1 kHz (FOC 10분주) |
| 메인 루프 | ~4.5 kHz |
| 제어 모터 수 | 2 (듀얼, 독립 타이머) |
| 전류 감지 | 2-shunt (Phase A+B 측정, Phase C = Kirchhoff) |
| 피드백 센서 | 쿼드러처 인코더 + 홀 센서 |
| 통신 | UART×2 + RS485 (SLIP 프로토콜) |
| 비휘발성 저장 | SPI FRAM (FM25V02A, 32KB) |

### 1.3 TI C2000 대비 변경 사항

| 항목 | TI C2000 (F2806x) | STM32G474 (BearDriverSTM) |
|------|-------------------|--------------------------|
| MCU | F28069M (C28x, 90MHz, 고정소수점) | **STM32G474VETx** (Cortex-M4F, 170MHz, FPU) |
| 수치 표현 | `_iq` 고정소수점 (IQmath) | **float** 부동소수점 (HW FPU) |
| 인터럽트 | PIE 벡터 테이블 | **NVIC** (Cortex-M4) |
| HAL | HAL_Handle (TI MotorWare) | **STM32 HAL** (CubeMX 생성) |
| 게이트 드라이버 | DRV8323 (SPI 레지스터) | **STDRIVE102BH ×2** (GPIO: nSTBY/FAULT/FLAG) |
| Motor1 PWM | EPWM1A/1B~3A/3B | **TIM1** CH1-3/CH1N-3N (PE8~PE13) |
| Motor2 PWM | 공유 EPWM (소프트웨어 분할) | **TIM20** CH1-3/CH1N-3N (PE2~PE6, PF2) 독립 타이머 |
| ISR 인터리빙 | 소프트웨어 인터리빙 | **TIM20 CNT=ARR 오프셋** (하드웨어 반주기 분리) |
| 전류 감지 | 3-shunt (DRV8323 내장 앰프) | **2-shunt** (외장 OPA, Phase C = Kirchhoff) |
| 전류 ADC | 단일 ADC 다중 SOC | **ADC3+ADC4** (M1) + **ADC1+ADC2** (M2) 교차 트리거 |
| Vbus ADC | ISR 내부 ADC 읽기 | **ADC5** (SysTick 1kHz, read-then-start) |
| 인코더 | QEP 모듈 (EQEP1/2) | **TIM5** (M1, PA0/PA1) + **TIM2** (M2, PD3/PD4) |
| 저속 캡처 | QEP 캡처 유닛 | **TIM3_CH2** (M1, PA4) + **TIM4_CH1** (M2, PB6) |
| 통신 | SCI-A/B (DSP UART) ×2 | **USART1/2/3** ×3 (Base + Debug + RS485) |
| RS485 DE | GPIO 소프트웨어 제어 | **USART3_DE** (PB14, AF7 하드웨어 자동 제어) |
| 비휘발 저장 | Flash EEPROM 에뮬레이션 | **SPI FRAM** (FM25V02A, 32KB, HW NSS PA15) |
| E-Stop | GPIO 폴링 | **PD14** (active-low, 디바운스 100회) |
| HW 버전 | 하드코딩 | **GPIO 4-bit** (PB4/PB5/PB7/PE1) |
| 브레이크 | GPIO 직접 구동 | **TIM8_CH4** (M1, PD1) + **TIM15_CH2** (M2, PB15) PWM |
| CAN | 없음 | **FDCAN1** (PA11/PA12) HW 준비 |
| 디버그 출력 | 없음 | **DAC1** (PA5) + **DAC2** (PA6) |
| LED | 없음 | **PA7** (Err) + **PC4** (Run) |
| USER_Params 구조체 | 49 필드 (EST/CTRL/TRAJ 포함) | **17 필드** (TI 레거시 32필드 제거) |

### 1.4 적용 모터

| 모터 모델 | 극쌍 수 | 인코더 | 적용 로봇 |
|-----------|---------|--------|-----------|
| ZLLG50ASM200_4096 | 10 | 4096 PPR | Servi-Q (기본) |
| ZLLG65ASM250 | 15 | 1024 PPR | Servi, Serviplus, Carti100 |
| ZLLG65ASM250_4096 | 15 | 4096 PPR | Cart carrier POC |
| ZLLG65ASM150_4096 | 15 | 4096 PPR | 고토크 변형 |
| ZLLG65ASM250_L | 15 | 1024 PPR | 저인덕턴스 변형 |

### 1.5 용어 및 약어

| 약어 | 설명 |
|------|------|
| FOC | Field-Oriented Control (자계 지향 제어) |
| SVM | Space Vector Modulation (공간벡터 변조) |
| BLDC | Brushless DC Motor |
| PMSM | Permanent Magnet Synchronous Motor |
| PID | Proportional-Integral-Derivative 제어기 |
| SLIP | Serial Line Internet Protocol |
| STO | Safe Torque Off (안전 토크 차단) |
| SS2 | Safe Stop 2 (안전 정지 2, ISO 13849-1) |
| pu | Per-Unit (정규화 값, TI legacy) |
| KRPM | Kilo-RPM (천 회전/분) |
| ISR | Interrupt Service Routine |
| DWT | Data Watchpoint and Trace |

---

## 2. 시스템 아키텍처

### 2.1 계층 구조

```
┌─────────────────────────────────────────────────────────────────┐
│                    응용 계층 (Application Layer)                   │
│  bear_driver : 시스템 상태머신, 듀얼 모터 총괄 관리                │
│  api         : 호스트 레지스터 맵 기반 명령/응답 처리                │
├─────────────────────────────────────────────────────────────────┤
│                    모터 제어 계층 (Motor Control Layer)             │
│  bldc_motor  : FOC 전류 루프, 속도 루프, Clarke/Park, SVM           │
│  pid         : PID 제어기 (속도, Id, Iq, Hall 토크)                │
│  traj        : 속도 궤적 생성 (가감속 제한)                         │
│  EStop_SCurve: 비상정지 S-curve 감속 프로파일                       │
├─────────────────────────────────────────────────────────────────┤
│                    센서 계층 (Sensor Layer)                        │
│  encoder     : 쿼드러처 인코더 위치/속도 추정                       │
│  hall_sensor : 홀 센서 6-Step BLDC 전환                            │
├─────────────────────────────────────────────────────────────────┤
│                    통신 계층 (Communication Layer)                  │
│  sci_coms    : UART/RS485 드라이버 + SLIP 프로토콜 인코더/디코더     │
│  crc         : XOR-16 체크섬 검증 (TI bear::xor_crc 호환)          │
│  spi_flash   : SPI FRAM 파라미터 저장 (FM25V02A)                    │
├─────────────────────────────────────────────────────────────────┤
│                    유틸리티 계층 (Utility Layer)                    │
│  timers                 : 소프트웨어 타이머 관리                    │
│  differential_drive_limiter : 차동 구동 속도 제한기                  │
│  version / version_info : 펌웨어 버전 관리                          │
│  user_params / user_motor_database : 시스템/모터 파라미터            │
├─────────────────────────────────────────────────────────────────┤
│                    HAL 계층 (Hardware Abstraction)                  │
│  STM32G4xx HAL Driver : GPIO, UART, SPI, ADC, TIM, DAC, FDCAN    │
│  CMSIS                : ARM Cortex-M4 코어 정의                    │
├─────────────────────────────────────────────────────────────────┤
│                    하드웨어 (Hardware)                              │
│  STM32G474VETx, STDRIVE102BH×2, 전류센서, 인코더×2, 홀 센서×2     │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 실행 컨텍스트 분류

| 컨텍스트 | 주기 | 우선순위 | 처리 내용 |
|----------|------|---------|----------|
| ADC3 JEOC ISR (Motor1) | 10 kHz | 2 | TIM20_TRGO → ADC3+ADC4 → Motor1 FOC ISR |
| ADC1 JEOC ISR (Motor2) | 10 kHz | 2 | TIM1_TRGO → ADC1+ADC2 → Motor2 FOC ISR |
| TIM5 ISR | 비주기적 | 3 | Motor1 인코더 오버플로우 |
| TIM2 ISR | 비주기적 | 3 | Motor2 인코더 오버플로우 |
| USART ISR | 바이트 단위 | 3 | UART 수신/송신 콜백 |
| TIM1_BRK_TIM15 ISR | 이벤트 | 4 | Motor1 게이트 드라이버 fault (PE15 → TIM1_BKIN) |
| TIM20_BRK ISR | 이벤트 | 4 | Motor2 게이트 드라이버 fault (PF9 → TIM20_BKIN) |
| SysTick ISR | 1 kHz (1 ms) | 15 | HAL 시스템 틱, Vbus ADC, 10ms 타이머 |
| 메인 루프 | ~4.5 kHz | 배경 | 상태머신, 속도 루프, 통신, 진단 |

> **ISR 인터리빙**: TIM20 카운터를 ARR로 프리로드하여 Motor1/Motor2 ADC 트리거가 반주기(~50μs) 오프셋. 두 ISR이 교대 실행되어 CPU 부하 분산.

### 2.3 모듈 의존성

```
bear_driver ─┬→ bldc_motor ─┬→ pid
             │              ├→ encoder
             │              ├→ hall_sensor
             │              ├→ EStop_SCurve
             │              └→ user_params ─→ user_motor_database
             │
             ├→ api ─→ sci_coms ─→ crc
             │
             ├→ spi_flash ─→ flash_layout
             │
             ├→ timers
             │
             ├→ differential_drive_limiter
             │
             └→ version / version_info
```

---

## 3. 하드웨어 구성

### 3.1 주변장치 맵핑

| 주변장치 | 기능 | 채널/핀 | 설정 |
|----------|------|---------|------|
| **TIM1** | **Motor1** PWM | CH1(PE9)/CH2(PE11)/CH3(PE13): H-side; CH1N(PE8)/CH2N(PE10)/CH3N(PE12): L-side; CH4: ADC 트리거 | 10kHz Center-aligned, TRGO=OC4REF→ADC1/ADC2, BKIN=PE15 |
| **TIM20** | **Motor2** PWM | CH1(PE2)/CH2(PE3)/CH3(PF2): H-side; CH1N(PE4)/CH2N(PE5)/CH3N(PE6): L-side; CH4: ADC 트리거 | 10kHz Center-aligned, TRGO=OC4REF→ADC3/ADC4, BKIN=PF9, **CNT=ARR 프리로드** |
| **TIM5** | Motor1 인코더 | CH1(PA0), CH2(PA1) | Encoder mode TI12 (4x), ARR=`ENCODER_LINES×4-1` |
| **TIM2** | Motor2 인코더 | CH1(PD3), CH2(PD4) | Encoder mode TI12 (4x), ARR=`ENCODER_LINES×4-1` |
| **TIM3** | Motor1 저속 캡처 | CH2(PA4) | Input Capture, PSC 모드 의존 |
| **TIM4** | Motor2 저속 캡처 | CH1(PB6) | Input Capture, PSC 모드 의존 |
| **TIM8** | Motor1 브레이크 | CH4(PD1) | PWM 출력 |
| **TIM15** | Motor2 브레이크 | CH2(PB15) | PWM 출력 |
| **ADC1** | Motor2 전류 | Injected: CH6(PC0):PhA, CH7(PC1):PhB | TIM1_TRGO 교차 트리거, JEOS 인터럽트 |
| **ADC2** | Motor2 서미스터 | Injected: CH12(PB2):NTC | TIM1_TRGO (ADC1 동기) |
| **ADC3** | Motor1 전류+서미스터 | Injected: CH4(PE7):PhA, CH12(PB1):NTC | TIM20_TRGO 교차 트리거, JEOS 인터럽트 |
| **ADC4** | Motor1 전류 | Injected: CH1(PE14):PhB | TIM20_TRGO (ADC3 동기) |
| **ADC5** | 버스 전압/전류 | Regular: CH9(PD12):Voltage, CH10(PD13):Current | SW 트리거, SysTick 1kHz 폴링 |
| **USART1** | Base 통신 | TX(PA9), RX(PA10) | 115200 baud |
| **USART2** | 디버그/부트로더 | TX(PA2), RX(PA3) | 921600 baud (SCIA_BAUD_RATE 오버라이드) |
| **USART3** | RS485 API | TX(PD8), RX(PD9), DE(PB14/AF7) | 115200 baud, HAL_RS485Ex_Init |
| **SPI3** | FRAM (FM25V02A) | SCK(PC10), MOSI(PC12), MISO(PC11), NSS(PA15) | 10.625 MHz (DIV16), Mode 0, HW NSS |
| **DAC1** | 디버그 출력1 | CH2(PA5) | 12-bit, SW 트리거 |
| **DAC2** | 디버그 출력2 | CH1(PA6) | 12-bit, SW 트리거 |
| **FDCAN1** | CAN 버스 | RX(PA11), TX(PA12) | HW 준비, SW 미구현 |

> **교차 트리거 설계**: TIM1(Motor1 PWM)의 TRGO가 ADC1/ADC2(Motor2 전류)를 트리거하고, TIM20(Motor2 PWM)의 TRGO가 ADC3/ADC4(Motor1 전류)를 트리거한다. 인터리빙 오프셋에 의해 각 모터의 전류는 자신의 PWM 반주기 시점에서 샘플링된다.

### 3.2 GPIO 핀 할당

#### Motor1 인버터 출력 (TIM1, STDRIVE102BH)

| 핀 | 기능 | 방향 | 설명 |
|----|------|------|------|
| PE8 | po_INL1_1 | 출력 | Phase A Low-side |
| PE9 | po_INH1_1 | 출력 | Phase A High-side |
| PE10 | po_INL2_1 | 출력 | Phase B Low-side |
| PE11 | po_INH2_1 | 출력 | Phase B High-side |
| PE12 | po_INL3_1 | 출력 | Phase C Low-side |
| PE13 | po_INH3_1 | 출력 | Phase C High-side |

#### Motor2 인버터 출력 (TIM20, STDRIVE102BH)

| 핀 | 기능 | 방향 | 설명 |
|----|------|------|------|
| PE2 | po_INH1_2 | 출력 | Phase A High-side |
| PE3 | po_INH2_2 | 출력 | Phase B High-side |
| PE4 | po_INL1_2 | 출력 | Phase A Low-side |
| PE5 | po_INL2_2 | 출력 | Phase B Low-side |
| PE6 | po_INL3_2 | 출력 | Phase C Low-side |
| PF2 | po_INH3_2 | 출력 | Phase C High-side |

#### 전류 센서 (ADC)

| 핀 | 기능 | ADC | 설명 |
|----|------|-----|------|
| PE7 | ai_OA_OA_1 | ADC3_CH4 | Motor1 Phase A |
| PE14 | ai_OA_OB_1 | ADC4_CH1 | Motor1 Phase B |
| PB0 | ai_OA_OC_1 | — | Motor1 Phase C (미사용, 2-shunt Kirchhoff) |
| PC0 | ai_OA_OA_2 | ADC1_CH6 | Motor2 Phase A |
| PC1 | ai_OA_OB_2 | ADC1_CH7 | Motor2 Phase B |
| PC2 | ai_OA_OC_2 | — | Motor2 Phase C (미사용, 2-shunt Kirchhoff) |
| PC3 | ai_Vref | ADC1_CH9 | 기준 전압 |

#### 버스 전압/전류 및 서미스터

| 핀 | 기능 | ADC | 설명 |
|----|------|-----|------|
| PD12 | ai_Voltage | ADC5_CH9 | 버스 전압 |
| PD13 | ai_Current | ADC5_CH10 | 버스 전류 |
| PB1 | ai_Thermistor_1 | ADC3_CH12 | Motor1 NTC |
| PB2 | ai_Thermistor_2 | ADC2_CH12 | Motor2 NTC |

#### 인코더 및 홀 센서

| 핀 | 기능 | 타이머 | 설명 |
|----|------|--------|------|
| PA0 | di_EQEPA_1 | TIM5_CH1 | Motor1 인코더 A |
| PA1 | di_EQEPB_1 | TIM5_CH2 | Motor1 인코더 B |
| PA4 | di_EQEPA_1A4 | TIM3_CH2 | Motor1 저속 캡처 |
| PD3 | di_EQEPA_2 | TIM2_CH1 | Motor2 인코더 A |
| PD4 | di_EQEPB_2 | TIM2_CH2 | Motor2 인코더 B |
| PB6 | pi_EQEPA_2 | TIM4_CH1 | Motor2 저속 캡처 |
| PB11 | di_HALLA_1 | GPIO | Motor1 홀 A (PULLUP) |
| PB12 | di_HALLB_1 | GPIO | Motor1 홀 B (PULLUP) |
| PB13 | di_HALLC_1 | GPIO | Motor1 홀 C (PULLUP) |
| PD7 | di_HALLA_2 | GPIO | Motor2 홀 A |
| PD6 | di_HALLB_2 | GPIO | Motor2 홀 B |
| PD5 | di_HALLC_2 | GPIO | Motor2 홀 C |

#### 게이트 드라이버 제어

| 핀 | 기능 | 방향 | 설명 |
|----|------|------|------|
| PC5 | do_nSTBY_1 | 출력 | Motor1 Standby (active-low) |
| PB9 | do_nSTBY_2 | 출력 | Motor2 Standby (active-low) |
| PE15 | di_FAULT_1 | 입력 | Motor1 nFAULT (TIM1_BKIN, active-low) |
| PF9 | di_FAULT_2 | 입력 | Motor2 nFAULT (TIM20_BKIN, active-low) |
| PB10 | di_FLAG_1 | 입력 | Motor1 FLAG |
| PE0 | di_FLAG_2 | 입력 | Motor2 FLAG |

#### 통신

| 핀 | 기능 | 방향 | 설명 |
|----|------|------|------|
| PA2 | uart2_Debug_TX | 출력 | USART2 TX (921600 baud) |
| PA3 | uart2_Debug_RX | 입력 | USART2 RX |
| PA9 | uart1_Base_TX | 출력 | USART1 TX (115200 baud) |
| PA10 | uart1_Base_RX | 입력 | USART1 RX |
| PD8 | uart3_485_TX | 출력 | USART3 RS485 TX |
| PD9 | uart3_485_RX | 입력 | USART3 RS485 RX |
| PB14 | do_485_Dir | 출력 | USART3_DE (AF7, HW 자동 제어) |
| PA11 | CAN1_RX | 입력 | FDCAN1 수신 |
| PA12 | CAN1_TX | 출력 | FDCAN1 송신 |

#### SPI / DAC / 기타

| 핀 | 기능 | 방향 | 설명 |
|----|------|------|------|
| PA15 | SPI3_FLASH | 출력 | FRAM NSS (HW, pulse mode) |
| PC10 | spi3_SCK | 출력 | SPI3 클럭 |
| PC11 | SPI3_MISO | 입력 | SPI3 데이터 입력 |
| PC12 | SPI3_MOSI | 출력 | SPI3 데이터 출력 |
| PA5 | dac1_Debug1 | 출력 | DAC1_OUT2 |
| PA6 | dac2_Debug2 | 출력 | DAC2_OUT1 |
| PA7 | do_LED_Err | 출력 | 에러 LED |
| PC4 | do_LED_Run | 출력 | 동작 LED |
| PD14 | di_nESTOP_IN | 입력 | 비상정지 (active-low) |
| PB4 | di_rev_B0 | 입력 | HW 버전 bit 0 |
| PB5 | di_rev_B1 | 입력 | HW 버전 bit 1 |
| PB7 | di_rev_B2 | 입력 | HW 버전 bit 2 |
| PE1 | di_rev_B3 | 입력 | HW 버전 bit 3 |
| PD1 | po_Brake_1 | 출력 | Motor1 브레이크 (TIM8_CH4) |
| PB15 | po_Brake_2 | 출력 | Motor2 브레이크 (TIM15_CH2) |
| PA13 | SWDIO | I/O | SWD 데이터 |
| PA14 | SWCLK | 입력 | SWD 클럭 |
| PB3 | SWO | 출력 | Serial Wire Output |

### 3.3 클럭 트리

```
HSI (16 MHz) → PLL (PLLM=/4, PLLN=×85, PLLR=/2) → SYSCLK = 170 MHz
    ├→ AHB  = 170 MHz
    ├→ APB1 = 170 MHz (PCLK1)
    ├→ APB2 = 170 MHz (PCLK2)
    └→ Flash Latency = 4 WS (SCALE1_BOOST mode)
```

### 3.4 듀얼 모터 주변장치 매핑 요약

| 항목 | Motor1 | Motor2 |
|------|--------|--------|
| **PWM 타이머** | **TIM1** (PE8~PE13) | **TIM20** (PE2~PE6, PF2) |
| **전류 ADC** | ADC3(PhA, PE7) + ADC4(PhB, PE14) | ADC1(PhA+PhB, PC0/PC1) |
| **서미스터** | ADC3 injected rank2 (PB1) | ADC2 injected rank1 (PB2) |
| **ADC 트리거** | TIM20_TRGO (교차) | TIM1_TRGO (교차) |
| **위상 오프셋** | Baseline (CNT=0) | **Half-period** (CNT=ARR) |
| **인코더 QEP** | TIM5 (PA0/PA1) | TIM2 (PD3/PD4) |
| **저속 캡처** | TIM3_CH2 (PA4) | TIM4_CH1 (PB6) |
| **홀 센서** | PB11/PB12/PB13 | PD7/PD6/PD5 |
| **게이트 드라이버** | nSTBY=PC5, FAULT=PE15, FLAG=PB10 | nSTBY=PB9, FAULT=PF9, FLAG=PE0 |
| **BKIN** | TIM1_BKIN (PE15) | TIM20_BKIN (PF9) |
| **브레이크** | TIM8_CH4 (PD1) | TIM15_CH2 (PB15) |

---

## 4. 소프트웨어 모듈 상세 설계

### 4.1 상태머신 모듈 (bear_driver)

**파일**: `BearDriver/Inc/bear_driver.h`, `BearDriver/Src/bear_driver.cpp`

#### 4.1.1 모듈 개요

시스템 전체 수명주기를 관리하는 최상위 상태머신이다. 모터 초기화, 정상 운전, 비상정지, 고장 관리, 복구를 총괄한다.

#### 4.1.2 상태 정의

```c
typedef enum MAIN_STATE_t {
  STATE_INIT,           // 초기화 대기
  STATE_CALC_OFFSETS,   // ADC 오프셋 캘리브레이션
  STATE_RUN,            // 정상 운전
  STATE_SS2_ESTOP,      // SS2 비상정지 (S-curve 감속)
  STATE_FAULT,          // 고장 상태
  STATE_FAULT_RESTART,  // 고장 복구 중
  STATE_ESTOP,          // STO 비상정지 (즉시 차단)
  STATE_ESTOP_RESTART,  // 비상정지 복구 중
  STATE_STALL_LOCK      // 스톨 잠금 (수동 해제 필요)
} MAIN_STATE;
```

#### 4.1.3 상태 전이 설계

```
STATE_INIT ─→ STATE_CALC_OFFSETS ─→ STATE_RUN ─┬→ STATE_FAULT ─→ STATE_FAULT_RESTART ─→ STATE_RUN
                                                ├→ STATE_SS2_ESTOP ─→ STATE_ESTOP ─→ STATE_ESTOP_RESTART ─→ STATE_RUN
                                                └→ STATE_STALL_LOCK (수동 해제 필요)
```

상태 전이 조건:
- `STATE_RUN → STATE_FAULT`: 치명 에러 비트 감지 (kStallError, kHallError, kGateDriverError 등)
- `STATE_RUN → STATE_SS2_ESTOP`: E-Stop 입력 활성 (PD14 LOW, 디바운스 100회)
- `STATE_SS2_ESTOP → STATE_ESTOP`: S-curve 감속 완료 또는 타임아웃
- `STATE_FAULT → STATE_FAULT_RESTART`: 호스트 재시작 명령 수신
- `STATE_RUN → STATE_STALL_LOCK`: 스톨 감지 + `stall_lock_enable=true`

#### 4.1.4 상태별 동작

| 상태 | 동작 | TI 대비 |
|------|------|---------|
| `STATE_INIT` | 주변장치 초기화, ADC 캘리브레이션, 파라미터 로드 | TI: HAL_init()+PIE, STM32: HAL_Init()+NVIC |
| `STATE_CALC_OFFSETS` | ADC 전류 오프셋 IIR 필터 (1초, 10000샘플) + ±1.05A 범위 검증 | TI: 단일 모터, STM32: 양쪽 독립 |
| `STATE_RUN` | 정상 운전 — FOC + 속도루프 + 통신 | TI와 동일한 제어 흐름 |
| `STATE_SS2_ESTOP` | S-curve 감속 → PWM OFF → 브레이크 ON | TI: GPIO 브레이크, STM32: TIM8/TIM15 PWM |
| `STATE_FAULT` | PWM 비활성, 에러 코드 보고, 호스트 대기 | TI와 동일 |
| `STATE_FAULT_RESTART` | setupMotor() 재호출, ADC 재시작, 상태 초기화 | TI: DRV8323 SPI 리셋, STM32: STDRIVE GPIO 리셋 |
| `STATE_ESTOP` | 즉시 토크 차단 (STO), 브레이크 인가 | TI와 동일 |
| `STATE_STALL_LOCK` | 모터 비활성, 수동 해제 대기 | TI와 동일 |

#### 4.1.5 외부 인터페이스

```c
void BearDriver_Main(void);              // 메인 엔트리 (무한 루프)
MAIN_STATE getMainState(void);
void setMainState(MAIN_STATE state);
void disableMotors(void);               // 양쪽 모터 비활성 + SD assert
void Motor1_ADC_ReadAndISR(void);       // ADC3 JEOC → Motor1 FOC
void Motor2_ADC_ReadAndISR(void);       // ADC1 JEOC → Motor2 FOC
void Motor1_ISR_Handler(void);
void Motor2_ISR_Handler(void);
void BearDriver_SlowADC_Update(void);  // ADC5 Vbus/Ibus (SysTick 1kHz)
```

#### 4.1.6 메인 루프 실행 순서

```
BearDriver_MainLoop() [~4.5kHz]
  ├→ 1단계: E-Stop 디바운스 확인 (ESTOP_DEBOUNCE_THRESHOLD = 100 ≈ 22ms)
  ├→ 2단계: 상태머신 디스패치 (switch/case)
  ├→ 3단계: API_ProcessHostComs()     // 통신 처리 (1패킷/cycle)
  ├→ 4단계: motor1.run(&gUserParams)  // Motor1 저속 루프
  └→ 5단계: motor2.run(&gUserParams)  // Motor2 저속 루프
```

#### 4.1.7 초기화 시퀀스 (BearDriver_Main)

```
BearDriver_Main()
  ├→ DWT 사이클 카운터 활성화
  ├→ CPU_TIME_init(&cpu_time_m1/m2, SystemCoreClock, PWM_FREQUENCY)
  ├→ ADC 캘리브레이션 (hadc1~hadc5)
  ├→ HAL_Delay(1000ms)
  ├→ HW 버전 읽기: PB4/PB5/PB7/PE1 → motor_hd_version (4-bit)
  ├→ USER_setParamsMtr(&gUserParams, motor_hd_version)
  ├→ USER_checkForErrors() — 실패 시 무한 루프
  ├→ SCI_Init()
  ├→ spi_flash.init(&hspi3, nullptr, 0)  ← HW NSS
  ├→ FlashLayout 읽기 → PID / disable_motors_on_boot / kinematic_limits
  ├→ motor1.setupMotor(&gUserParams, &htim1, &hadc3, &hadc4)   // Motor1 = TIM1
  ├→ motor2.setupMotor(&gUserParams, &htim20, &hadc1, &hadc2)  // Motor2 = TIM20
  ├→ Hall 센서 setup: M1(PB11/12/13), M2(PD7/6/5)
  ├→ Encoder init: M1=TIM5+TIM3, M2=TIM2+TIM4
  ├→ Gate driver setup: M1(PC5/PE15/PB10), M2(PB9/PF9/PE0)
  ├→ Timers_Init()
  ├→ ADC 시작:
  │   HAL_ADCEx_InjectedStart_IT(&hadc1)  // Motor2 currents
  │   HAL_ADCEx_InjectedStart(&hadc2)     // Motor2 NTC
  │   HAL_ADCEx_InjectedStart_IT(&hadc3)  // Motor1 currents
  │   HAL_ADCEx_InjectedStart(&hadc4)     // Motor1 PhB
  │   HAL_ADC_Start(&hadc5)              // Vbus regular
  ├→ Encoder 시작: TIM5(M1), TIM2(M2)
  ├→ Motor1 PWM 시작: TIM1 CH1~3, CH1N~3N
  ├→ ★ __HAL_TIM_SET_COUNTER(&htim20, ARR)  // 인터리빙 오프셋
  ├→ Motor2 PWM 시작: TIM20 CH1~3, CH1N~3N
  ├→ disablePWM() ×2 (부팅 안전)
  ├→ gateDriver.powerUp() ×2
  └→ 무한 루프 → main_loop()
```

#### 4.1.8 ISR 인터리빙 타이밍

```
TIM1 (Motor1):  ╱╲ peak    ╱╲ peak       TIM1_TRGO → ADC1 → Motor2 ISR
               ╱    ╲    ╱    ╲
          ────╱──────╲──╱──────╲────
              0    50μs  100μs  150μs

TIM20 (Motor2):    ╲ peak ╱╲ peak ╱      TIM20_TRGO → ADC3 → Motor1 ISR
                    ╲    ╱  ╲    ╱
          ──────────╲──╱────╲──╱──────
                   50μs  100μs

ISR:  M2   M1   M2   M1   M2   M1    ← 교대 실행 (~50μs 간격)
```

#### 4.1.9 설정 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `ESTOP_DEBOUNCE_THRESHOLD` | 100 | E-Stop 디바운스 (~22ms @ 4.5kHz) |
| `config.disable_motors_on_boot` | true | 부팅 시 모터 비활성 |
| `config.stall_lock_enable` | true | 스톨 잠금 기본값 |
| `config.init_delay_ms` | 1000 | 초기화 지연 |
| `timer_period[CMD]` | 100 | 통신 타임아웃 (ms) |

#### 4.1.10 TI 대비 STM32 적응 사항

| 항목 | TI (F2806x) | STM32G474 | 비고 |
|------|-------------|-----------|------|
| ISR wrapper | `motor1_ISR()` bare | `Motor1_ADC_ReadAndISR()` | ADC read 후 ISR |
| Fault ISR | HW PIE interrupt | ISR top 폴링 | 기능 동등 |
| Gate driver | DRV8323 (SPI) | STDRIVE102BH×2 (GPIO) | 듀얼 독립 |
| Vbus ADC | ISR 내부 | SysTick 1kHz (ADC5) | 분리 |
| Motor2 PWM | 공유 타이머 | **TIM20 독립** | 인터리빙 |
| Motor2 offset | 양쪽 calibration | 양쪽 calibration | 동일 |

---

### 4.2 모터 제어 모듈 (bldc_motor)

**파일**: `BearDriver/Inc/bldc_motor.h`, `BearDriver/Src/bldc_motor.cpp`

TI 원본과 동일한 FOC 알고리즘 (Clarke → Park → PID(Id,Iq) → Inv.Park → SVM). 수치 연산: TI `_iq` 고정소수점 → STM32 `float` (HW FPU). 전류 감지: TI 3-shunt (DRV8323 내장 앰프) → STM32 **2-shunt** (외장 OPA, Phase C = Kirchhoff).

#### 4.2.1 2-Shunt FOC ISR 흐름

```
Motor1_ADC_ReadAndISR() [ADC3 JEOS, TIM20_TRGO]
  ├→ PhA = ADC3 Rank1 (PE7) × (ADC_TO_AMPS / 65536)
  ├→ PhB = ADC4 Rank1 (PE14) × (ADC_TO_AMPS / 65536)
  ├→ PhC = -(PhA + PhB)   ← Kirchhoff
  ├→ Thermistor = ADC3 Rank2 (PB1)
  └→ Motor1_ISR_Handler()

Motor2_ADC_ReadAndISR() [ADC1 JEOS, TIM1_TRGO]
  ├→ PhA = ADC1 Rank1 (PC0) × (ADC_TO_AMPS / 65536)
  ├→ PhB = ADC1 Rank2 (PC1) × (ADC_TO_AMPS / 65536)
  ├→ PhC = -(PhA + PhB)   ← Kirchhoff
  ├→ Thermistor = ADC2 Rank1 (PB2)
  └→ Motor2_ISR_Handler()
```

#### 4.2.2 Motor::ISR() 내부 흐름

```
Motor::ISR()
  ├→ 에러 체크 (gate driver fault, 홀 센서, 스톨)
  ├→ Clarke 변환: Ia,Ib,Ic → Iα,Iβ
  ├→ Park 변환: Iα,Iβ → Id,Iq (전기각 기준)
  ├→ 제어 분기 (TI if/else 구조):
  │   ├→ if (Flag_Run_Identify) — 활성 제어
  │   │   ├→ [SPEED] RunSpeedLoop() → Iq_ref
  │   │   ├→ RunCurrentLoop(): PID(Id)→Vd, PID(Iq)→Vq
  │   │   └→ Flag_needCurrentDecay = true
  │   ├→ else if (Flag_needCurrentDecay) — 전류 감쇠 (TI 동일)
  │   │   ├→ [SPEED] RunSpeedLoop() — decay 중에도 속도 PID 유지
  │   │   ├→ |Id_meas| or |Iq_meas| > 0.1A? ← 실측 전류(idq) 기준, ADC 노이즈 바닥(~50mA) 이상
  │   │   │   ├→ YES: idq_ref *= current_decay_, RunCurrentLoop()
  │   │   │   └→ NO: Flag_needCurrentDecay = false
  │   │   └→ 주의: 종료 조건은 실측 전류(idq) 사용 (레퍼런스 아님).
  │   │      RunSpeedLoop()이 idq_ref를 덮어쓰므로 레퍼런스 기준 시 종료 불가.
  │   └→ else — 유휴: PID 리셋, pwm=0, disablePWM()
  ├→ 역 Park 변환: Vd,Vq → Vα,Vβ
  ├→ SVM: Vα,Vβ → Ta,Tb,Tc 듀티
  ├→ PWM 레지스터 갱신 (TIM1 또는 TIM20 CCR)
  ├→ 인코더 trackPeriod() (10kHz)
  └→ [÷10] 속도 루프: encoder.updateValues(), traj 갱신
```

> TI 대비: 알고리즘 동일. `_iq` → `float`, DRV8323 SPI fault read → STDRIVE GPIO fault read.

#### 4.2.3 에러 감지 상수

| 항목 | 값 | 설명 |
|------|-----|------|
| `STALL_THRESHOLD_RPM` | 5.0 | 스톨 판정 속도 |
| `STALL_CURRENT_THRESHOLD_A` | 3.0 | 스톨 판정 전류 |
| `STALL_TIME_MS` | 500 | 스톨 지속 시간 |
| `HALL_ERROR_TIMEOUT_MS` | 100 | 홀 센서 타임아웃 |
| `WINDUP_LIMIT` | 0.95 | PID 와인드업 경고 임계값 |

---

### 4.3 PID 제어기 모듈 (pid)

**파일**: `BearDriver/Inc/pid.h`, `BearDriver/Src/pid.cpp`

범용 PID 제어기. 모터당 3개 인스턴스: 속도(Speed), d축 전류(Id), q축 전류(Iq).

| 파라미터 | 설명 | 비고 |
|---------|------|------|
| Kp, Ki, Kd | 비례/적분/미분 게인 | FRAM 저장/복원 |
| Kff | 피드포워드 게인 | 속도 PID에 사용 |
| outMin, outMax | 출력 클램프 | 와인드업 방지 |
| dN | 미분 필터 계수 | 고주파 노이즈 억제 |
| UdOutMax | D항 출력 제한 (0=비활성) | 속도 PID: `spdMinMax × 0.1f` (최대 전류의 10%) |

**D항 출력 제한 (UdOutMax)**: `UdOutMax > 0` 설정 시 D항 입력·출력 양방향에 대칭 클램프 적용.
속도 급변(역전) 시 D항 스파이크를 제한하여 전류 포화 방지. 전류 PID에는 미적용 (`Kd=0`).

> TI 대비: `_iq` → `float`. 알고리즘 동일 (병렬형 PID + anti-windup + derivative filter). UdOutMax는 STM32 추가 기능.

### 4.4 인코더 모듈 (encoder)

**파일**: `BearDriver/Inc/encoder.h`, `BearDriver/Src/encoder.cpp`

쿼드러처 인코더 위치/속도 추정. 고속(카운트 차분) + 저속(주기 캡처) 복합 추정.

| 항목 | Motor1 | Motor2 | TI 대비 |
|------|--------|--------|---------|
| QEP 타이머 | TIM5 (PA0/PA1) | TIM2 (PD3/PD4) | TI: EQEP1/2 모듈 → STM32: TIM Encoder Mode |
| 캡처 타이머 | TIM3_CH2 (PA4) | TIM4_CH1 (PB6) | TI: QEP 캡처 유닛 → STM32: Input Capture |
| 오버플로우 ISR | TIM5_IRQn (우선순위 3) | TIM2_IRQn (우선순위 3) | TI: EQEP ISR |

- `updateValues()`: 1kHz (속도 루프), 카운트 차분 → RPM
- `trackPeriod()`: 10kHz (FOC ISR), 캡처 주기 → 저속 RPM 보완

**SW 캡처 모드 `capture_seen` 패턴**: HW 입력 캡처 타이머를 사용할 수 없는 경우,
`trackPeriod()` 소프트웨어 에지 감지 경로에서 방향 역전 직후 속도 스파이크를 방지한다.

| 단계 | 조건 | 동작 |
|------|------|------|
| 방향 전환 에지 | `direction != hw_dir` | `speriod=MAX`, `capture_seen=false` |
| 역전 후 첫 번째 동방향 에지 | `capture_seen==false` | 앵커만 설정 (`capture_seen=true`) |
| 역전 후 두 번째 이상 동방향 에지 | `capture_seen==true` | `speriod=period_cntr` 정상 적용 |

역전 직후 첫 에지는 앵커-온리(anchor only) 처리하여 이전 방향과의 잔류 시간이 속도 계산에 혼입되는 것을 방지한다. TI 원본은 HW 캡처 전용 패턴이며 SW 모드에는 미적용 — STM32 추가 기능.

> TI 대비: TI QEP 캡처 유닛 → STM32 TIM Input Capture + 소프트웨어 에지 감지 겸용.

### 4.5 홀 센서 모듈 (hall_sensor)

**파일**: `BearDriver/Inc/hall_sensor.h`, `BearDriver/Src/hall_sensor.cpp`

3상 홀 센서 6-Step BLDC 전환. GPIO 직접 읽기 → 섹터 판정 → 전기각 초기값.

| 항목 | Motor1 | Motor2 |
|------|--------|--------|
| 홀 A | PB11 (di_HALLA_1) | PD7 (di_HALLA_2) |
| 홀 B | PB12 (di_HALLB_1) | PD6 (di_HALLB_2) |
| 홀 C | PB13 (di_HALLC_1) | PD5 (di_HALLC_2) |

> TI 대비: TI GPIO 읽기와 동일. 룩업 테이블 기반 섹터→전기각 매핑.

### 4.6 궤적 생성 모듈 (traj)

**파일**: `BearDriver/Inc/traj.h`, `BearDriver/Src/traj.cpp`

속도 궤적 생성 — 매 ISR 사이클(10kHz)마다 `traj_maxDelta`씩 목표 속도에 접근.

- `setTrajMaxDelta(D)`: MaxAccel_rpmps + traj_maxDelta 동시 갱신
- `Flag_Run_Identify` 분기: true → `traj_target=SpeedRef_rpm`, false → `traj_target=0`

> TI 대비: TI TRAJ 모듈 (TRAJ_setMaxDelta) 동일 구조. EST/CTRL/PowerWarp 관련 궤적은 제거됨.

### 4.7 비상정지 모듈 (EStop_SCurve)

**파일**: `BearDriver/Inc/EStop_SCurve.h`, `BearDriver/Src/EStop_SCurve.c`

S-curve 감속 프로파일. SS2 비상정지 시 급정지가 아닌 부드러운 감속 후 정지.

- 입력: 현재 속도, 목표=0
- 출력: 감속 궤적 (가속도 연속)
- 완료 조건: 속도 < 임계값 → STATE_ESTOP 전이

> TI 대비: 알고리즘 동일. 브레이크 인가: TI GPIO → STM32 TIM8_CH4(M1)/TIM15_CH2(M2) PWM.

---

### 4.8 통신 모듈 (sci_coms)

TI SCI 모듈을 STM32 USART로 포팅. SLIP 프로토콜, 버퍼 구조, 상태머신 동일. ISR에서 HAL 우회하여 직접 RDR/TDR 접근 (TI SCI 레지스터 직접 접근과 동등).

```c
typedef enum {
  SCI_A_FD = 0,   // USART2 (PA2/PA3, 921600 baud)
  SCI_B_FD = 1    // USART3 (PD8/PD9/PB14, 115200 baud)
} SCI_Device_e;
```

> USART1 (PA9/PA10, 115200) 은 Base 통신용 예약. SCI 모듈 미통합.

#### 4.8.1 버퍼 설계

| 버퍼 | 크기 | 마스크 | 설명 |
|------|------|--------|------|
| RX 원형 버퍼 | 128 바이트 | 0x7F | ISR → 메인 루프 |
| TX 원형 버퍼 | **512 바이트** | **0x1FF** | 메인 루프 → ISR (USART2 FIFO 충분 확보) |
| SLIP 디코딩 버퍼 | 128 바이트 | - | SLIP 디코딩된 패킷 임시 저장 |

#### 4.8.2 TX FIFO 모드 (USART2 + USART3)

USART2(SCI_A_FD) 및 USART3(SCI_B_FD) 모두 TX FIFO 8단계 활성화, threshold = 1/2(4바이트). TXFT 인터럽트로 ISR 횟수 ~4배 감소. `SCI_WriteTxBuffer`, `SCI_TxCallback` 분기 없이 단일 경로.

```
SCI_WriteTxBuffer(c)
  └→ UART_IT_TXFT 활성화  // USART2 + USART3 공통 (양쪽 FIFO 모드)

TX 인터럽트 (TXFT)
  └→ SCI_TxCallback(dev)
      └→ while(TXFNF) TDR = buf[out++]  // FIFO 최대 8바이트 드레인
           └→ [버퍼 비어있음] TXFT 인터럽트 비활성화
```

> usart.c USER CODE 2: 양쪽 USART 모두 `HAL_UARTEx_SetTxFifoThreshold(1/2)` + `HAL_UARTEx_EnableFifoMode()`.
> stm32g4xx_it.c: USART2/USART3 모두 `UART_FLAG_TXFT` / `UART_IT_TXFT` 체크.

#### 4.8.3 RX 드레인 루프

단일 바이트 RXNE 읽기 대신 FIFO 내 모든 바이트를 ISR 1회 진입에서 소비:

```c
if (__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE)) {
  while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE)) {
    uint8_t data = (uint8_t)(huart->Instance->RDR & 0xFFU);
    SCI_RxCallback(dev, data);
  }
}
```

ISR 오버런 방지 + RX 원형버퍼 풀 시 `SCI_RxOverflow[dev]++` 계수.

#### 4.8.4 에러 카운터

디버거 Watch 변수 (extern in sci_coms.h):

| 변수 | 설명 |
|------|------|
| `SCI_RxOverflow[2]` | ISR에서 RX 원형버퍼 풀로 바이트 드롭된 횟수 |
| `SCI_CrcErrors[2]` | 수신 패킷 CRC 불일치 횟수 |
| `SCI_SlipErrors[2]` | SLIP 디코딩 오류 횟수 (버퍼 오버플로, 잘못된 ESC, 비정상 START) |

#### 4.8.5 인터페이스

| 함수 | 설명 |
|------|------|
| `SCI_Init()` | 전체 SCI 시스템 초기화 |
| `SCI_Device_Init(dev)` | 특정 UART 디바이스 초기화 (버퍼 바이트 단위 클리어) |
| `SCI_ReadPacket(dev, ...)` | 패킷 수신 (SLIP 디코딩 + XOR-16 CRC 검증) |
| `SCI_SendPacket(dev, packet)` | 패킷 송신 (SLIP 인코딩, TX overflow 보호) |
| `SCI_TxEmpty(dev)` | TX 완료 확인 (SW 버퍼 + UART_FLAG_TC) |
| `SCI_RxCallback(dev, data)` | RX ISR 콜백 (버퍼 풀 시 SCI_RxOverflow 계수) |
| `SCI_TxCallback(dev)` | TX ISR 콜백 (FIFO 드레인, 양쪽 USART 동일 경로) |
| `SCI_Printf(dev, fmt, ...)` | printf 형식 디버그 출력 (SCI_A_FD 권장) |

---

### 4.9 API 모듈 (api)

**파일**: `BearDriver/Inc/api.h`, `BearDriver/Src/api.cpp`

호스트 레지스터 맵 기반 명령/응답 처리. BasePacket(호스트→모터) 수신 → 레지스터 읽기/쓰기 → MotorPacket(모터→호스트) 응답.

> TI 대비: 프로토콜 동일. MotorPacket에 arg5(토크, 0.1Nm), arg6(온도, 0.1°C) 추가 — 서미스터 ADC 기반 NTC 계산.

---

### 4.10 SPI FRAM 모듈 (spi_flash)

TI Flash EEPROM 에뮬레이션 → STM32 SPI FRAM (FM25V02A, 32KB) 으로 변경.

| 항목 | TI C2000 | STM32G474 |
|------|----------|-----------|
| 저장 매체 | Flash EEPROM 에뮬레이션 | **SPI FRAM** (FM25V02A) |
| 내구성 | ~10⁵ cycles | **10¹⁴ cycles** |
| 용량 | 제한적 (Flash 섹터) | **32KB** (바이트 주소) |
| 쓰기 방식 | 섹터 소거 → 프로그램 | **즉시 쓰기** (소거 불필요) |
| NSS 제어 | — | HW NSS pulse (PA15) |

`cs_port = nullptr` → `deselect()` 에서 `__HAL_SPI_DISABLE()` → NSS HIGH.

---

### 4.11 소프트웨어 타이머 모듈 (timers)

**파일**: `BearDriver/Inc/timers.h`, `BearDriver/Src/timers.cpp`

SysTick 10ms 콜백 기반 다중 소프트웨어 타이머. 통신 타임아웃, 디바운스, 주기적 진단에 사용.

> TI 대비: TI CPU 타이머 → STM32 SysTick 10ms 콜백. 기능 동일.

### 4.12 차동 구동 제한기 모듈 (differential_drive_limiter)

**파일**: `BearDriver/Inc/differential_drive_limiter_helper.h`, `BearDriver/Src/differential_drive_limiter_helper.cpp`

좌/우 모터 속도 명령을 운동학적 제한(최대 선속도, 최대 각속도)에 맞게 클램핑. 파라미터 FRAM 저장.

> TI 대비: TI에 없는 STM32 추가 기능 (듀얼 모터 전용).

### 4.13 CRC 모듈 (crc)

**파일**: `BearDriver/Inc/crc.h`

XOR-16 체크섬 (init=0, 다항식/리플렉션/최종 XOR 없음). SLIP 패킷 무결성 검증.

```c
// XOR all 16-bit little-endian words of the payload
crc = word[0] ^ word[1] ^ ... ^ word[n-1]
```

> TI 대비: TI `bear::xor_crc<uint16_t>` 와 알고리즘 완전 동일.
> 이전 CRC-16-CCITT (0x1021, init=0xFFFF) 에서 변경 — TI 호환성 복원.

### 4.14 버전 관리 모듈 (version)

**파일**: `BearDriver/Inc/version.h`, `BearDriver/Inc/version_info.h`, `BearDriver/Src/version_info.cpp`

TI 원본 스타일 적용: Doxygen 없음, VERSION_STRING/PLATFORM_NAME/PROJECT_NAME 제거.

```c
// version.h (TI 스타일)
#define MAJOR_VERSION  6
#define MINOR_VERSION  0
#define REVISION       0
#define BUILD          0
```

버전 응답 (`getVersion`): `VERSION_INFO_t` 유니온 사용, TI와 동일한 packing.
```c
pOut->arg1 = v->groups.major_minor;     // little-endian: lower 16bit = MAJOR = 6
pOut->arg2 = v->groups.revision_build;
```

> TI 대비: TI 원본 `version.h` 스타일 복원. 이전 VERSION_STRING/PLATFORM_NAME/PROJECT_NAME 매크로 제거.

### 4.15 디버그 모듈 (debug)

**파일**: `BearDriver/Inc/debug.h`, `BearDriver/Src/debug.c`

DAC 출력 기반 실시간 신호 관측. DAC1_CH2(PA5), DAC2_CH1(PA6).

> TI 대비: TI에 없는 STM32 추가 기능 (HW DAC 활용).

### 4.16 ISR 실행 시간 측정 모듈 (cpu_time)

**파일**: `BearDriver/Inc/cpu_time.h` (헤더 전용, inline 구현)

#### 4.16.1 모듈 개요

ARM Cortex-M4 DWT(Data Watchpoint and Trace) 사이클 카운터를 이용한 ISR 실행 시간 측정 모듈. TI MotorWare `cpu_time.h/cpu_time.c` (CPU Timer 2 기반)를 대체한다.

> TI 대비: TI CPU Timer 2 (free-running 32-bit counter) → ARM DWT->CYCCNT (32-bit cycle counter). 별도 하드웨어 타이머 소비 없이 동일 기능 제공.

#### 4.16.2 데이터 구조

```c
typedef struct {
  uint32_t start_cnt;       // DWT->CYCCNT at ISR entry
  uint32_t isr_cycles;      // Last ISR execution cycles
  uint32_t isr_cycles_max;  // Peak ISR execution cycles (worst case)
  uint32_t pwm_period_cyc;  // PWM period in CPU cycles (= SYSCLK / PWM_Hz)
  float    cpu_usage;       // CPU usage ratio (0.0 ~ 1.0), updated per ISR
  float    cpu_usage_max;   // Peak CPU usage ratio (0.0 ~ 1.0)
} CPU_TIME_Obj;
```

#### 4.16.3 인터페이스

| 함수 | 호출 위치 | 설명 |
|------|----------|------|
| `CPU_TIME_init(obj, sysclk_hz, pwm_freq_hz)` | `BearDriver_Main()` 초기화부 | 구조체 초기화, `pwm_period_cyc = sysclk_hz / pwm_freq_hz` |
| `CPU_TIME_start(obj)` | `MotorX_ADC_ReadAndISR()` 진입 | `DWT->CYCCNT` 스냅샷 저장 |
| `CPU_TIME_end(obj)` | `MotorX_ADC_ReadAndISR()` 종료 | elapsed 계산, peak 갱신, `cpu_usage` 산출 |
| `CPU_TIME_getUsage(obj)` | 디버거/API | 현재 ISR CPU 사용률 반환 |
| `CPU_TIME_getUsageMax(obj)` | 디버거/API | 피크 ISR CPU 사용률 반환 |
| `CPU_TIME_resetMax(obj)` | 디버거/API | 피크값 리셋 |

#### 4.16.4 인스턴스

```c
CPU_TIME_Obj cpu_time_m1;   // Motor1 ISR 측정 (ADC3 콜백)
CPU_TIME_Obj cpu_time_m2;   // Motor2 ISR 측정 (ADC1 콜백)
```

#### 4.16.5 측정 구간

```
Motor1_ADC_ReadAndISR() {
  CPU_TIME_start(&cpu_time_m1);    ← DWT->CYCCNT 스냅샷
  // ADC 읽기 → 전류 변환 → FOC → PWM 출력
  Motor1_ISR_Handler();
  CPU_TIME_end(&cpu_time_m1);      ← elapsed = CYCCNT - start, cpu_usage 갱신
}
```

Motor2도 동일 패턴 (`cpu_time_m2`).

#### 4.16.6 CPU 사용률 계산

```
cpu_usage = isr_cycles / pwm_period_cyc
          = isr_cycles / (170,000,000 / 10,000)
          = isr_cycles / 17,000
```

| 구간 | cpu_usage | 판정 |
|------|-----------|------|
| < 0.50 | 안전 | 충분한 여유 |
| 0.50 ~ 0.70 | 정상 | 운용 가능 |
| 0.70 ~ 0.80 | 주의 | 최적화 검토 |
| > 0.80 | 위험 | ISR 오버런 위험 |

#### 4.16.7 측정 오버헤드

| 항목 | 사이클 |
|------|--------|
| `CPU_TIME_start` | ~3 |
| `CPU_TIME_end` (정수 연산) | ~6 |
| `CPU_TIME_end` (float 변환+VDIV.F32) | ~18 |
| **합계** | **~27 (~0.16%)** |

---

## 5. 데이터 구조 및 자료형

### 5.1 주요 데이터 구조

| 구조체 | 용도 | 주요 필드 |
|--------|------|----------|
| `USER_Params` | 시스템/모터 파라미터 (17필드) | hwVersion, motor_numPolePairs, motor_Rs, motor_Ls_d/q, maxCurrent, spdParams |
| `HAL_AdcData_t` | ADC 데이터 (ISR→제어) | Iabc_A[3], dcBus, temperature_adc |
| `HAL_PwmData_t` | PWM 듀티 (제어→TIM) | Tabc[3] (-1.0~1.0) |
| `Motor` | 모터 클래스 (C++) | encoder, hallSensor, gateDriver, pid[3], adcData, pwmData |
| `BasePacket_t` | 호스트→모터 명령 | rw, reg, arg1, arg2, crc |
| `MotorPacket_t` | 모터→호스트 응답 | rw, reg, arg1~arg6, crc |
| `FlashLayout` | FRAM 저장 구조 | pid, disable_motors_on_boot, kinematic_limits (각각 magic tag) |

### 5.2 자료형

| TI 원본 | STM32 포팅 | 비고 |
|---------|-----------|------|
| `_iq` (32-bit 고정소수점) | `float` (32-bit IEEE 754) | Cortex-M4 HW FPU |
| `_iq24`, `_iq15` 등 | `float` (통일) | 스케일링 제거 |
| `uint16_t` pu 값 | `float` (실수 단위) | A, V, RPM 직접 표현 |

---

## 6. 인터럽트 설계

### 6.1 NVIC 우선순위 테이블

| 우선순위 | NVIC IRQ | ISR | 주기 | 핵심 처리 |
|---------|----------|-----|------|----------|
| **0** | TIM1_UP_TIM16 | TIM1_UP_TIM16_IRQHandler | 10kHz | Motor1 TIM update |
| **0** | TIM20_UP | TIM20_UP_IRQHandler | 10kHz | Motor2 TIM update |
| **2** | ADC1_2 | ADC1_2_IRQHandler | TIM1 동기 | → Motor2_ADC_ReadAndISR |
| **2** | ADC3 | ADC3_IRQHandler | TIM20 동기 | → Motor1_ADC_ReadAndISR |
| **3** | TIM5 | TIM5_IRQHandler | 비주기 | Motor1 인코더 overflow |
| **3** | TIM2 | TIM2_IRQHandler | 비주기 | Motor2 인코더 overflow |
| **3** | USART2 | USART2_IRQHandler | RX: 드레인루프, TX: FIFO(TXFT) | SCI RX/TX (HAL 우회, TX FIFO 8단계) |
| **3** | USART3 | USART3_IRQHandler | RX: 드레인루프, TX: FIFO(TXFT) | SCI RS485 RX/TX (TX FIFO 8단계) |
| **4** | TIM1_BRK_TIM15 | TIM1_BRK_TIM15_IRQHandler | 이벤트 | Motor1 gate driver fault |
| **4** | TIM20_BRK | TIM20_BRK_IRQHandler | 이벤트 | Motor2 gate driver fault |
| **15** | SysTick | SysTick_Handler | 1kHz | HAL tick + SlowADC + 10ms timer |

### 6.2 ADC 콜백 디스패치

```cpp
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC3) {
    Motor1_ADC_ReadAndISR();   // TIM20_TRGO → ADC3+ADC4 → Motor1
  } else if (hadc->Instance == ADC1) {
    Motor2_ADC_ReadAndISR();   // TIM1_TRGO → ADC1+ADC2 → Motor2
  }
}
```

### 6.3 ISR 디버그 카운터

`ISR_SysTick_Count`, `ISR_ADC1_2_Count`, `ISR_ADC3_Count`, `ISR_TIM1_BRK_TIM15_Count`, `ISR_TIM2_Count`, `ISR_TIM5_Count`, `ISR_TIM20_BRK_Count`, `ISR_USART2_Count`, `ISR_USART3_Count`, `IdleLoopCount`.

### 6.4 공유 데이터 보호

ISR↔메인 루프 간 공유 데이터 보호:

| 데이터 | 쓰기 컨텍스트 | 읽기 컨텍스트 | 보호 방식 |
|--------|-------------|-------------|----------|
| `adcData` (Iabc, dcBus) | ADC ISR (10kHz) | 메인 루프 | 단방향 (ISR→Main), atomic float write |
| `pwmData` (Tabc 듀티) | FOC ISR | TIM CCR 갱신 | ISR 내부 완결 |
| `SpeedRef_rpm` | 메인 루프 (API) | FOC ISR 속도루프 | 단일 float, 자연 atomic |
| `error_code` | FOC ISR | 메인 루프 상태머신 | volatile uint16_t, 비트 OR 누적 |
| SCI rx_buffer | USART ISR | 메인 루프 SLIP 디코더 | Circular buffer (head/tail 분리) |

> TI 대비: 동일한 보호 전략. Cortex-M4 32-bit aligned float write는 atomic (단일 STR 명령).

---

## 7. 실행 컨텍스트 및 타이밍

### 7.1 듀얼 모터 인터리빙 타이밍

```
시간 →
      0μs        50μs       100μs      150μs      200μs
  ├────────────┤────────────┤────────────┤────────────┤

M2 ┃█ ADC1_ISR ┃            ┃█ ADC1_ISR ┃            ┃
   ┃ (FOC M2)  ┃            ┃ (FOC M2)  ┃            ┃
   ┃ ~15μs     ┃            ┃ ~15μs     ┃            ┃

M1 ┃            ┃█ ADC3_ISR ┃            ┃█ ADC3_ISR ┃
   ┃            ┃ (FOC M1)  ┃            ┃ (FOC M1)  ┃
   ┃            ┃ ~15μs     ┃            ┃ ~15μs     ┃

SysTick ┃                        █ (1ms)              ┃

Main ┃ ░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░┃
     ┃ MainLoop (상태머신 + 통신 + 속도루프)          ┃
```

### 7.2 제어 루프 분주 관계

```
PWM (10kHz) ─→ [÷1] ─→ ADC trigger ─→ ISR (10kHz per motor)
                                        [÷1]  ─→ 전류 루프 (10kHz)
                                        [÷10] ─→ 속도 루프 (1kHz)
                                        [÷10] ─→ encoder.updateValues() (1kHz)
                                        [÷1]  ─→ encoder.trackPeriod() (10kHz)
                                        [÷1]  ─→ 궤적 갱신 (10kHz)
```

---

## 8. 통신 프로토콜 상세

### 8.1 물리 계층

| 인터페이스 | 핀 | 보레이트 | 방식 | 용도 |
|-----------|-----|---------|------|------|
| USART1 | PA9/PA10 | 115200 | Full-duplex | Base 통신 (예약) |
| USART2 | PA2/PA3 | 921600 | Full-duplex | 디버그, STM 부트로더 |
| USART3 | PD8/PD9/PB14 | 115200 | RS485 Half-duplex | 호스트 API |

### 8.2 SLIP 프로토콜

| 바이트 | 의미 |
|--------|------|
| 0xC1 | START (패킷 시작) |
| 0xC0 | END (패킷 종료) |
| 0xDB | ESC (이스케이프 접두사) |
| 0xDB 0xDC | 데이터 내 0xC0 인코딩 |
| 0xDB 0xDE | 데이터 내 0xC1 인코딩 |
| 0xDB 0xDD | 데이터 내 0xDB 인코딩 |

수신 상태머신: `SRX_IDLE → SRX_CHAR → SRX_ESC`
버퍼: `rx_isr_buffer[2][128]` (RX ISR circular), `tx_isr_buffer[2][512]` (TX ISR circular), `rx_slip_buffer[2][128]` (SLIP decode linear)

### 8.3 RS485 통신

- 트랜시버: MAX3485EESA+
- DE 제어: PB14 = USART3_DE (AF7, 하드웨어 자동 제어)
- DEAT/DEDT = 4 (~1μs DE 어설션/디어설션 여유, bit period 8.68μs @ 115200)
- TX 중 RX 억제: `!rs485_tx_active` guard in `SCI_ProcessHostComs()`
- TX 완료 확인: `SCI_TxEmpty()` → UART_FLAG_TC (shift register drain)
- PD9(RX) GPIO_PULLUP: 버스 부동 시 spurious RXNE 방지

> TI 대비: TI SCI GPIO DE 제어 → STM32 USART HW DE 자동 제어. 프로토콜 동일.

---

## 9. 안전 기능 설계

### 9.1 비상정지

- 입력 핀: **PD14** (di_nESTOP_IN, active-low)
- 디바운스: `ESTOP_DEBOUNCE_THRESHOLD=100` (~22ms @ 4.5kHz 메인 루프)
- 동작 순서: E-Stop 감지 → `STATE_SS2_ESTOP` → S-curve 감속 → `STATE_ESTOP` (STO)
- HW 버전 읽기: PB4/PB5/PB7/PE1 (4-bit GPIO → `motor_hd_version`)

> TI 대비: TI GPIO 폴링 동일 구조. 핀 할당만 변경.

### 9.2 스톨 감지

양쪽 모터 독립 적용. 조건: 속도 < `STALL_THRESHOLD_RPM` AND 전류 > `STALL_CURRENT_THRESHOLD_A` AND 지속 시간 > `STALL_TIME_MS`.

- `stall_lock_enable=true`: `STATE_STALL_LOCK` 전이 (수동 해제 필요)
- `stall_lock_enable=false`: `STATE_FAULT` 전이 (자동 재시작 가능)

> TI 대비: 알고리즘 동일. 듀얼 모터 독립 판정 추가.

### 9.3 게이트 드라이버 Fault

| 모터 | nFAULT 핀 | BKIN | ISR |
|------|----------|------|-----|
| Motor1 | PE15 (di_FAULT_1) | TIM1_BKIN | TIM1_BRK_TIM15 (우선순위 4) |
| Motor2 | PF9 (di_FAULT_2) | TIM20_BKIN | TIM20_BRK (우선순위 4) |

> TI 대비: TI DRV8323 nFAULT (SPI 상태 읽기) → STM32 STDRIVE102BH nFAULT (GPIO + BKIN HW 연동).

### 9.4 Fault → Restart 흐름

호스트 재시작 명령 수신 → `STATE_FAULT_RESTART`:

```
STATE_FAULT_RESTART
  ├→ SetErrorCode(kErrorNone) × 2       ← 에러 플래그 전체 클리어
  ├→ fault_latch_m1 = 0, fault_latch_m2 = 0  ← ISR fault latch 클리어
  ├→ Flag_bypassFaultCheck = true × 2   ← ISR fault 폴링 우회 (TI PIE_disableExtInt 대체)
  ├→ gateDriver.reset() × 2             ← SD 펄스 + fault latch 클리어 (3ms blocking)
  └→ FALLTHROUGH ↓

STATE_ESTOP_RESTART
  ├→ Flag_bypassFaultCheck = true × 2   ← 직접 진입 경로용
  ├→ setupMotor() × 2                   ← 제어 변수 재초기화 (offsetCalcCount=0 포함)
  ├→ hallSensor.setup() × 2
  ├→ encoder.init() + initCapture() × 2
  ├→ SetErrorCode(kErrorNone) × 2       ← ISR 재감지 대비 2차 클리어
  └→ Timers_Start(TIMER_INIT) → STATE_INIT

STATE_INIT → STATE_CALC_OFFSETS
  ├→ 오프셋 캘리브레이션 (IIR 필터 1.0초 + ±1.05A 범위 검증)
  ├→ Flag_bypassFaultCheck = false × 2  ← ISR fault 폴링 복원 (TI PIE_enableExtInt 대체)
  └→ STATE_RUN
```

**ISR Race Condition 방지 (Flag_bypassFaultCheck)**:
`gateDriver.reset()` 의 3ms blocking `HAL_Delay()` 동안 ISR 이 ~30회 실행된다.
nFAULT 핀이 gate driver 안정화 전에 LOW 이면 ISR 의 `checkFault()` 가
`gateDriver.disable()` 을 호출하여 reset 을 무효화한다.
`Flag_bypassFaultCheck = true` 로 ISR fault 폴링을 우회하고,
캘리브레이션 완료 후 `false` 로 복원한다 (TI `PIE_disableExtInt/enableExtInt` 대체).

setupMotor 인자:
```c
motor1.setupMotor(&gUserParams, &htim1, &hadc3, &hadc4);
motor2.setupMotor(&gUserParams, &htim20, &hadc1, &hadc2);
```

### 9.5 워치독 (IWDG)

**설정값**

| 항목 | 값 | 비고 |
|------|-----|------|
| 클럭 | LSI 32 kHz | 독립 클럭 — 시스템 클럭 장애와 무관 |
| Prescaler | `/4` (`IWDG_PRESCALER_4`) | tick = 8 kHz (0.125 ms) |
| Reload | 4095 | 12-bit 최대값 |
| **Ti (Timeout)** | **512 ms** | (4095+1) × 0.125 ms |
| Window | 4095 | Window 모드 비활성 |
| 리셋 방식 | RESET only | 인터럽트 미사용 |

**지연 시작 패턴 (TI `HAL_setupWatchdog()` 동일)**

`main.c` 주변장치 초기화 블록에서 `MX_IWDG_Init()`이 호출되지만, `iwdg.c`의 `USER CODE BEGIN IWDG_Init 0`에서 첫 번째 호출을 early return으로 차단한다. 이를 통해 `HAL_Delay(1000)` 및 플래시 초기화 구간을 워치독 비활성 상태로 안전하게 통과한다.

```
1st call  (main.c peripheral init)        → s_iwdg_deferred=true  → 파라미터 저장 후 return
2nd call  (BearDriver_Main, for(;;) 직전) → HAL_IWDG_Init() 실행  → IWDG 시작
```

**리프레시 구조**

```cpp
MX_IWDG_Init();          // IWDG 시작 (초기화 완료 후)
for (;;) {
    HAL_IWDG_Refresh(&hiwdg);           // 외부 루프 kick
    while (gFlag_enableSystem) {
        HAL_IWDG_Refresh(&hiwdg);       // 내부 루프 kick (~4.5 kHz)
        Timers_Process();
        main_loop();
    }
}
```

> TI 대비: `WDOG_clearCounter(halHandle->wdogHandle)` → `HAL_IWDG_Refresh(&hiwdg)`. 호출 위치·구조 동일. Ti TI ~418 ms → STM32 512 ms.

---

## 10. 비휘발성 저장 설계

SPI3 FRAM (FM25V02A, 32KB). HW NSS (PA15): `init(&hspi3, nullptr, 0)`.

### 10.1 저장 항목

| 항목 | Magic | 내용 |
|------|-------|------|
| `disable_motors_on_boot` | 0x36F2 | 부팅 시 모터 비활성 여부 (bool) |
| `pid` | 0x36F2 | PID 게인 (Kp, Ki, Kd, Kff, K, outMax) |
| `kinematic_limits` | 0x36F2 | 차동 구동 운동학 제한 파라미터 |

- Magic = `kValid(0x36F2)`: 유효 데이터, `kClear(0xFFFF)`: FRAM 초기 상태
- 읽기: 부팅 시 1회, magic 확인 후 로드 (무효 시 기본값 유지)
- 쓰기: 호스트 API 명령 시 즉시 기록 (FRAM 소거 불필요)

> TI 대비: TI Flash EEPROM 에뮬레이션 (섹터 소거 필요, ~10⁵ 내구성) → STM32 FRAM (즉시 쓰기, 10¹⁴ 내구성).

---

## 11. 파라미터 정의

### 11.1 시스템 파라미터

| 파라미터 | 값 | 단위 |
|---------|-----|------|
| ADV_TIM_CLK_MHz | 170 | MHz |
| USER_PWM_FREQ_kHz | 10.0 | kHz |
| USER_CTRL_FREQ_Hz | 10000.0 | Hz |
| USER_CTRL_PERIOD_sec | 0.0001 | s |

### 11.2 전류/전압 스케일링

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| RSHUNT | 0.005 Ω | 션트 저항 |
| AMPLIFICATION_GAIN | 12 | 증폭 게인 |
| ADC_TO_AMPS | 55.0 | Vref/(Rshunt×Gain) |
| VBUS_ADC_TO_VOLT | 55.96 | Vref/분압비 |
| PWM_PERIOD_CYCLES | 17000 | 170M/10k |
| DEAD_TIME_COUNTS | 170 | 1000ns DTG |

### 11.3 모터 파라미터 (기본 모터: ZLLG50ASM200_4096)

| 파라미터 | 값 | 단위 |
|---------|-----|------|
| USER_MOTOR_NUM_POLE_PAIRS | 10 | 극쌍 |
| USER_MOTOR_Rs | 모터별 상이 | Ω |
| USER_MOTOR_Ls_d / Ls_q | 모터별 상이 | H |
| USER_MOTOR_ENCODER_LINES | 4096 | PPR |
| USER_MOTOR_MAX_CURRENT | 모터별 상이 | A |

### 11.4 제어 파라미터

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| numPwmTicksPerIsrTick | 1 | PWM→ISR 분주 |
| numCtrlTicksPerSpeedTick | 10 | ISR→속도루프 분주 |
| USER_MAX_VS_MAG_PU | 0.5 | 최대 전압 벡터 크기 (pu) |
| USER_OFFSET_POLE_rps | 20.0 | 오프셋 필터 극 |
| USER_VOLTAGE_FILTER_POLE_rps | 344.62 | 전압 필터 극 |

### 11.5 속도 PID 기본 게인 (SI 단위)

| 파라미터 | 값 | 단위 | 범위 | 설명 |
|---------|-----|------|------|------|
| Kp | 0.08 | A/RPM | 0.02 ~ 0.2 | 비례 게인 |
| Ki | 2.0 | A/(RPM·s) | 0.5 ~ 5.0 | 적분 게인 (Kis = Ki×Ts) |
| Kd | 0.0008 | A·s/RPM | 0.0002 ~ 0.002 | 미분 게인 (Kds = Kd/Ts) |
| Kff | 0.1 | A/(RPM/tick) | 0.0 ~ 0.5 | 피드포워드 게인 |
| dN | 200.0 | rad/s | 50 ~ 500 | 미분 필터 극점 (≈31.8 Hz) |
| Kout | 1.0 | - | - | 출력 루프 게인 (고정) |
| outMax | 모터 maxCurrent | A | - | 출력 제한 |

#### API 게인 변환 (TI per-unit ↔ STM SI)

API 통신에서 TI 호환 per-unit 게인을 수신하면, 아래 변환 상수로 SI 단위로 변환한다.

| 변환 상수 | 값 | TI (pu) | STM (SI) | 산출 |
|----------|-----|---------|----------|------|
| `SPD_KP_PU_TO_SI` | 0.001778 | Kp=45 | 0.08 | 0.08/45 |
| `SPD_KI_PU_TO_SI` | 0.006667 | Ki=300 | 2.0 | 2.0/300 |
| `SPD_KD_PU_TO_SI` | 0.016 | Kd=0.05 | 0.0008 | 0.0008/0.05 |
| `SPD_KFF_PU_TO_SI` | 0.02222 | Kff=4.5 | 0.1 | 0.1/4.5 |

---

## 12. 파일 구조 및 빌드

### 12.1 디렉토리 구조

```
BearDriverSTM/
├── Core/
│   ├── Inc/          main.h, stm32g4xx_hal_conf.h, stm32g4xx_it.h
│   ├── Src/          main.c, stm32g4xx_it.c, adc.c, tim.c, usart.c, spi.c, dac.c, gpio.c
│   └── Startup/      startup_stm32g474vetx.s
│
├── BearDriver/
│   ├── Inc/          bear_driver.h, bldc_motor.h, api.h, pid.h, encoder.h,
│   │                 hall_sensor.h, traj.h, EStop_SCurve.h, sci_coms.h,
│   │                 spi_flash.h, flash_layout.h, crc.h, timers.h,
│   │                 version.h, version_info.h, user_params.h,
│   │                 user_motor_database.h, differential_drive_limiter_helper.h,
│   │                 debug.h, cpu_time.h
│   └── Src/          bear_driver.cpp, bldc_motor.cpp, api.cpp, pid.cpp,
│                     encoder.cpp, hall_sensor.cpp, traj.cpp, EStop_SCurve.c,
│                     sci_coms.cpp, spi_flash.cpp, timers.cpp,
│                     version_info.cpp, differential_drive_limiter_helper.cpp,
│                     debug.c
│
├── Drivers/
│   ├── CMSIS/        ARM Cortex-M4 코어 정의
│   └── STM32G4xx_HAL_Driver/   STM32 HAL (Inc/ + Src/)
│
├── docs/             software_detailed_design.md
├── BearDriverSTM.ioc
└── .cproject / .project
```

### 12.2 언어 및 컴파일러

| 항목 | 사양 |
|------|------|
| 언어 | C11 (Core) + C++03 (BearDriver) |
| 컴파일러 | arm-none-eabi-gcc/g++ (STM32CubeIDE) |
| FPU | Cortex-M4 HW FPU (float) |
| C/C++ 혼합 | `extern "C"` 래퍼 |

### 12.3 C/C++ 인터페이스

```c
// bear_driver.h
#ifdef __cplusplus
extern "C" {
#endif
void BearDriver_Main(void);
void Motor1_ADC_ReadAndISR(void);
void Motor2_ADC_ReadAndISR(void);
void BearDriver_SlowADC_Update(void);
#ifdef __cplusplus
}
#endif
```

---

## 부록 A: TI → STM32 포팅 변경 이력

### A.1 초기 포팅 (BearDriverSTM_EV 기반)

§1.3 TI C2000 대비 변경 사항 참조.

### A.2 TI 호환성 복원 및 UART 개선 (2026-05-18)

| 구분 | 파일 | 변경 내용 |
|------|------|-----------|
| **CRC** | `crc.h` | CRC-16-CCITT → XOR-16 (TI `bear::xor_crc<uint16_t>` 완전 일치) |
| **버전** | `version.h` | TI 원본 스타일 복원: VERSION_STRING/PLATFORM_NAME/PROJECT_NAME 제거, MAJOR=6 |
| **버전** | `version.cpp` | Version_Print()에서 PROJECT_NAME/PLATFORM_NAME 출력 제거 |
| **버전** | `api.cpp` | getVersion: `VERSION_INFO_t` 유니온 사용 → TI와 동일한 major_minor packing |
| **버퍼 타입** | `sci_coms.cpp` | rx/tx_isr_buffer, rx_slip_buffer: `uint16_t` → `uint8_t` (zero-byte interleaving 버그 수정) |
| **USART3 FIFO** | `usart.c` | USART3 TX FIFO 활성화: threshold=1/2, EnableFifoMode (기존 Disabled) |
| **RS485 DE 타이밍** | `usart.c` | DEAT/DEDT: `(0,0)` → `(4,4)` (~1μs 여유 마진) |
| **RX pull-up** | `usart.c` | PD9(USART3 RX) GPIO_PULLUP 추가: 버스 부동 spurious RXNE 방지 |
| **TX 인터럽트 통합** | `sci_coms.cpp` | `SCI_WriteTxBuffer`: SCI_A_FD/SCI_B_FD 분기 제거 → UART_IT_TXFT 단일 경로 |
| **TX 콜백 통합** | `sci_coms.cpp` | `SCI_TxCallback`: if/else 분기 제거 → FIFO 드레인 단일 경로 (양쪽 USART) |
| **RX 드레인 루프** | `stm32g4xx_it.c` | USART2/3 RXNE: 단일 바이트 → while(RXNE) 드레인 루프 |
| **USART3 TX 인터럽트** | `stm32g4xx_it.c` | UART_IT_TXE → UART_IT_TXFT (FIFO 모드 일치) |
| **에러 카운터** | `sci_coms.cpp/h` | `SCI_RxOverflow`, `SCI_CrcErrors`, `SCI_SlipErrors` [2] 추가 |
| **memset 크기** | `sci_coms.cpp` | `SCI_Device_Init`: `SIZE * sizeof(uint16_t)` → `SIZE` (바이트 단위) |

### A.3 ISR 실행 시간 측정 및 Fault Latch 수정 (2026-05-19)

| 구분 | 파일 | 변경 내용 |
|------|------|-----------|
| **CPU_TIME 모듈** | `cpu_time.h` (신규) | DWT->CYCCNT 기반 ISR 실행 시간 측정 모듈 추가 (TI cpu_time.h/c 대체) |
| **ISR 측정 적용** | `bear_driver.cpp` | `Motor1_ADC_ReadAndISR()` / `Motor2_ADC_ReadAndISR()` 에 `CPU_TIME_start/end` 삽입 |
| **CPU 사용률** | `cpu_time.h` | `cpu_usage` (float, 0.0~1.0) 및 `cpu_usage_max` 필드 추가, ISR 매 사이클 자동 계산 |
| **초기화** | `bear_driver.cpp` | `BearDriver_Main()` 에서 `CPU_TIME_init(&cpu_time_m1/m2, SystemCoreClock, PWM_FREQUENCY)` 호출 |
| **Fault latch 클리어** | `bear_driver.cpp` | `STATE_FAULT_RESTART` 에서 `fault_latch_m1 = 0; fault_latch_m2 = 0;` 추가 (리스타트 후 잔류값 버그 수정) |
| **Fault latch 선언** | `bear_driver.cpp` | `volatile uint32_t fault_latch_m1, fault_latch_m2` 변수 선언 추가 (기존 미선언) |

---

## 부록 B: 에러 코드 비트맵

| 비트 | 플래그 | 마스크 | 심각도 | 설명 |
|------|--------|--------|--------|------|
| 0 | `kStallError` | 0x0001 | 치명 | 스톨 감지 |
| 1 | `kHallError` | 0x0002 | 치명 | 홀 센서 이상 |
| 2 | `kGateDriverError` | 0x0004 | 치명 | STDRIVE102BH nFAULT |
| 3 | `kWindupSpeedError` | 0x0008 | 경고 | 속도 PID 와인드업 |
| 4 | `kWindupCurrentIdError` | 0x0010 | 경고 | Id PID 와인드업 |
| 5 | `kWindupCurrentIqError` | 0x0020 | 경고 | Iq PID 와인드업 |
| 6 | `kHallPhaseAngleError` | 0x0040 | 치명 | 홀 위상각 검증 실패 |
| 7 | `kEncoderPhaseAngleError` | 0x0080 | 치명 | 인코더 위상각 검증 실패 |
| 8 | `kCommunicationsError` | 0x0100 | 경고 | 통신 타임아웃 |
| 9 | `kEStopError` | 0x0200 | 치명 | E-Stop 활성 |
| 10 | `kOffsetCalibrationWarning` | 0x0400 | 경고 | 오프셋 캘리브레이션 범위 초과 (±1.05A), fallback(0.0A) 사용 |

경고 마스크 (`kMotorErrorCodeWarnMask` = 0x0538): 경고 비트만 → 모터 계속 구동. 치명 비트 → `STATE_FAULT`.

---

## 부록 C: 모터 상태 워드 구조

```
getMotorStatus() (32비트):
  [31:22] error_code (10비트)
  [21:0]  gate driver 상태 워드 (하위 5비트 사용)
```

| 비트 | 이름 | 의미 |
|------|------|------|
| 0 | kStatusBit_nSTBY | nSTBY 핀 레벨 |
| 1 | kStatusBit_FLAG | FLAG 핀 레벨 |
| 2 | kStatusBit_nFAULT | nFAULT 핀 레벨 (1=OK) |
| 3-4 | kStatusShift_State | kStateActive(0) / kStateStandby(1) / kStateFault(2) |

양쪽 모터 모두 gate driver 활성 (TI 대비: 듀얼 독립 STDRIVE102BH).

---

## 부록 D: TI C2000 vs STM32G474 전류 제어기 비교 분석

> **목적**: TI BearDriver (C2000 F28069) 원본과 본 STM32G474 포트의 전류 제어기 구현 차이를
> 체계적으로 문서화하여 향후 튜닝·이식·유지보수의 기준으로 활용한다.
>
> **대상 파일**:
> - TI: `devel_serviq/firmware/Projects/BLDCDriver/BearDriver/src/bldc_motor.cpp`
> - STM32: `BearDriver/Src/bldc_motor.cpp`, `BearDriver/Src/pid.cpp`

---

### D.1 신호 도메인 — 가장 근본적인 차이

| 항목 | TI (C2000) | STM32 |
|------|-----------|-------|
| 전류 신호 | Per-unit `_iq` (Q24 고정소수점) | SI 단위 [A] |
| 전압 신호 | Per-unit `_iq` | SI 단위 [V] |
| PID 게인 | 무차원 [PU/PU] | 물리 단위 [V/A] |
| 연산 하드웨어 | C2000 내장 MAC (고정소수점) | Cortex-M4 FPU (float) |

TI 코드에는 `fullScaleCurrent`, `fullScaleVoltage` 스케일링 인수가 모든 게인 계산에 개입한다.
STM32 포트에서는 PU 변환 전체를 제거하여 **게인 = 물리 단위 직결**, 튜닝 직관성 향상.

---

### D.2 전류 제어기 대역폭

두 버전 모두 **극-영점 소거(Pole-Zero Cancellation)** 기반 PI를 사용하며, Kp 공식 구조는 동일하다.
차이는 수동 스케일링 인수에 있다.

#### D.2.1 Kp 계산

| 항목 | TI | STM32 |
|------|----|-------|
| Kp 공식 | `dq_p_pid_gain × 0.25 × Ls / Ts × (PU 환산)` | `currentBwCoeff × Ls / Ts` |
| `dq_p_pid_gain` | **0.4** (수동 보수 디튜닝) | — |
| `currentBwCoeff` | — | **0.25** (TI 원래 계수, 디튜닝 없음) |
| 유효 계수 α | `0.4 × 0.25 = 0.1` | `0.25` |
| 유효 대역폭 | `α / Ts = 0.1 / 100μs = 1000 rad/s ≈ **159 Hz**` | `0.25 / 100μs = 2500 rad/s ≈ **398 Hz**` |

TI는 실험적으로 `dq_p_pid_gain=0.4`를 적용하여 대역폭을 **원형 대비 2.5× 낮춰** 보수 운용했다.
STM32는 TI 원형 계수(`0.25`)를 그대로 사용한다.

#### D.2.2 Ki 계산

| 항목 | TI | STM32 |
|------|----|-------|
| Ki 공식 | `dq_i_pid_gain × Rs/Ls × Ts` | `Rs/Ls × Ts` |
| `dq_i_pid_gain` | **0.1** (10% 수준) | — |
| 극-영점 소거 달성도 | ~10% (의도적 감쇠) | **100% (완전 소거)** |

TI는 Ki도 `dq_i_pid_gain=0.1`로 대폭 축소하여 적분기를 의도적으로 느리게 운용했다.

---

### D.3 PID 적분 구조

`PID_run()` (전류 루프 전용) 의 적분 수식:

```
Up   = Kp × e
Ui[n] = Ui[n-1] + Ki × Up    ←  Ki × Kp × e   (표준 Ki×e 구조 아님)
Out  = Up + Ui
```

이산 Z-영역 전달함수:

```
C(z) = Kp × (1 + Ki) - Kp × z⁻¹)  /  (1 - z⁻¹)
```

연속 영점: `s ≈ −Ki / Ts`
`Ki = Rs/Ls × Ts` 대입 → **영점 = 플랜트 극 `s = −Rs/Ls`** → 극-영점 소거 성립.

양쪽 모두 동일한 구조. STM32 전류 루프에서 D항은 사용하지 않는다 (`Kd=0`).

---

### D.4 Iq 동적 전압 제한 — 양쪽 동일

매 ISR마다 Id PID 출력(Vd) 기준으로 Iq PID 출력 제한을 동적 갱신:

```
Vq_max = √( Vs_max² − Vd² )
```

| 구현 | TI | STM32 |
|------|----|-------|
| Vs_max | `USER_MAX_VS_MAG_PU × dcBus_pu` (PU) | `USER_MAX_VS_MAG_PU × VdcBus_Volt` (V) |
| 연산 | `_IQsqrt()` | `sqrtf()` |

알고리즘 자체는 완전히 동일. **Id 제어기가 먼저 실행되고 Iq가 나머지 전압을 사용**.

---

### D.5 dq 교차 결합 디커플링 — 양쪽 모두 미구현

이론적 완전 FOC에서는 아래 피드포워드 항이 필요하다:

```
Vd_ff = −ωe × Lq × Iq        (q→d 교차 결합 보상)
Vq_ff = +ωe × Ld × Id + ωe × λm   (역기전력 + d→q 교차 결합 보상)
```

**TI 및 STM32 양쪽 모두 이 디커플링 없이 순수 P+I만 사용한다.**

저속·중속 동작 범위에서는 교차 결합 항(`ωe × L × I`)이 작아 실용상 문제없다.
고속 영역에서는 잠재적 정상상태 오차 발생 가능하며, 필요 시 `Vq_ff` 항 추가가 성능 개선에 유효하다.

---

### D.6 DC 버스 보상 — 양쪽 동일

InversePark 출력 Vαβ [V 또는 PU]를 실제 버스 전압으로 나누어 모듈레이션 인덱스로 변환:

```
Vαβ_norm = Vαβ / Vbus
```

TI: `_IQmpy(Vab, 1/dcBus_pu)` | STM32: `Vab × (1.0f / VdcBus_Volt)`.
알고리즘 동일. SVGEN 입력은 양쪽 모두 `[-0.5, +0.5]` 범위.

---

### D.7 SVGEN — 사실상 동일

| 항목 | TI | STM32 |
|------|----|-------|
| 모듈 | `SVGEN_run()` | `SpaceVectorGen()` |
| 영상 시퀀스 주입 | 내부 3고조파 주입 | min-max centering (SVPWM 등가) |
| 출력 범위 | `Tabc ∈ [-0.5, +0.5]` | 동일 |

min-max centering `vshift = -0.5(vmax+vmin)` 은 3고조파 주입과 수학적으로 등가이며,
선형 변조 한계를 `1/√3 ≈ 0.577 × Vbus`로 동일하게 확장한다.

---

### D.8 SVGENCURRENT — TI 전용, STM32 미구현

| 항목 | TI | STM32 |
|------|----|-------|
| 모듈 | `SVGENCURRENT_compPwmData()` | **없음** |
| 기능 1 | 데드타임 보상 (PWM 듀티 교정) | 미구현 |
| 기능 2 | 2-shunt → 3상 전류 재구성 | 불필요 (3-shunt 고정) |
| 전류 센싱 | 2-shunt 또는 3-shunt (모드 전환) | 3-shunt 항상 |

STM32 하드웨어는 3상 전류를 항상 측정하므로 재구성 불필요.
단, **데드타임 보상 없음** — 데드타임이 상대적으로 긴 경우 낮은 속도에서 전류 왜형 발생 가능.

---

### D.9 전류 오프셋 캘리브레이션

| 항목 | TI | STM32 |
|------|----|-------|
| 방법 | SVGENCURRENT 프레임워크 내 평균 | 독립 IIR 필터 (`runOffsetsCalculation`) |
| 필터 극점 | 20.0 rad/s (동일) | 20.0 rad/s (TI 동일) |
| 캘리브레이션 시간 | ~1.0s | ~1.0s (TI 동일) |
| 범위 검증 | TI 동일 | ±1.05A → 초과 시 fallback 0.0A |

알고리즘 구조는 TI와 동일하게 유지되었다.

---

### D.10 속도 PID — 주요 차이점

| 항목 | TI | STM32 |
|------|----|-------|
| 신호 단위 | Per-unit (krpm) | SI [RPM] |
| PID 함수 | `PID_run_spd()` (P+I+D+FF) | 동일 |
| D항 | 사용 (`Kd_spd` 비영, `dN_spd`) | 동일 |
| **UdOutMax** | **없음** | **추가: `spdMinMax × 0.1f`** |
| 속도 추정 | T-method (QEP 캡처 유닛) | T-method (TIM3/4 HW 캡처 또는 SW `trackPeriod()`) |
| 방향 역전 보호 | `capture_seen` (HW 캡처 전용) | `capture_seen` (HW·SW 양 모드 적용) |

---

### D.11 Rs 온라인 재계산 — TI 전용

TI 코드에는 `Flag_enableRsRecalc` 플래그와 Rs 실시간 보정 루틴이 존재한다.
STM32 포트에서는 **제거됨** — `user_params.h`에 고정 모터 파라미터를 사용.

---

### D.12 종합 비교표

| 항목 | TI C2000 | STM32G474 | 차이 |
|------|---------|-----------|------|
| 신호 도메인 | Per-unit `_iq` | SI float | 도메인 변경 |
| 전류 루프 BW | ~159 Hz (α=0.1) | ~398 Hz (α=0.25) | **2.5× 높음** |
| Ki (극-영점 소거) | 10% 수준 | **100% 완전 소거** | 10× 적분 속도 차이 |
| dq 디커플링 | 없음 | 없음 | 동일 |
| Iq 동적 전압 제한 | √(Vs²−Vd²) | √(Vs²−Vd²) | **동일** |
| DC 버스 보상 | 있음 | 있음 | **동일** |
| SVGEN | SVGEN_run() | min-max centering | **수학적 등가** |
| SVGENCURRENT | 있음 (데드타임 보상) | **없음** | 미이식 |
| 데드타임 보상 | 있음 | **없음** | 미이식 |
| 3상 전류 센싱 | 2-shunt/3-shunt | 3-shunt 고정 | HW 차이 |
| UdOutMax (속도 D항 제한) | 없음 | **추가** | STM32 개선 |
| capture_seen (SW 모드) | 없음 | **추가** | STM32 개선 |
| Rs 온라인 재계산 | 있음 | **없음** | 기능 축소 |
| 고정소수점 연산 | C2000 MAC | Cortex-M4 FPU | HW 차이 |

---

### D.13 주의사항 및 개선 검토 항목

**① Ki 10× 차이 — 주의 필요**

STM32 Ki는 TI 대비 10× 크다. 현재 하드웨어(Rs=0.42Ω, Ls=0.88mH)에서 안정 운전 중이라면
문제없으나, 모터 파라미터 측정 오차가 클 경우 적분기 발산 리스크가 TI보다 높다.
이상 현상 발생 시 `currentBwCoeff` 축소 또는 `Ki_Id = α_i × Rs/Ls × Ts` (α_i < 1)로 조정한다.

**② 데드타임 보상 (SVGENCURRENT) 미이식**

3-shunt 구성에서는 전류 재구성 불필요하지만, 데드타임에 의한 PWM 왜형 보상은 별개 문제다.
저속(< 10 RPM)에서 전류 리플이 크면 소프트웨어 데드타임 보상 추가를 검토한다.

**③ dq 교차 결합 디커플링 검토**

최대 속도에서 교차 결합 항 크기:
```
Vq_coupling ≈ ωe × Ld × Id_max ≈ (330 × 10 × 2π/60) × 0.00088 × 20 ≈ 0.60 V
Vs_max ≈ 24 × 0.95 / √3 ≈ 13.2 V
```
비율 ~4.6% — 현재 운용 범위에서는 무시 가능하나, 버스 전압 상승 시 재검토 필요.

---

## 부록 E: TI C2000 vs STM32G474 속도 제어기 비교 분석

> **목적**: TI BearDriver (C2000 F28069) 원본과 본 STM32G474 포트의 속도 제어기 구현 차이를
> 체계적으로 문서화하여 향후 튜닝·이식·유지보수의 기준으로 활용한다.
>
> **대상 파일**:
> - TI: `devel_serviq/firmware/Projects/BLDCDriver/BearDriver/src/bldc_motor.cpp`, `include/pid.h`
> - STM32: `BearDriver/Src/bldc_motor.cpp`, `BearDriver/Src/pid.cpp`

---

### E.1 신호 도메인

| 항목 | TI (C2000) | STM32 |
|------|-----------|-------|
| 속도 단위 | Per-unit `_iq` (krpm) | SI float (RPM) |
| PID 출력 | Per-unit Iq_ref | SI Iq_ref (A) |
| 게인 단위 | 무차원 [PU/PU] | 물리 단위 [A/RPM] |

TI 코드에는 `speed_krpm_to_pu_sf`, `current_A_to_pu_sf` 스케일링 인수가 게인 및 출력 제한 계산에 개입한다.
STM32 포트에서는 PU 변환을 제거하여 **게인 = 물리 단위 직결**, 튜닝 직관성 향상.

---

### E.2 PID 구조 — 가장 근본적인 차이

TI는 속도 PID에 D항을 사용하지 않는다. STM32는 D항·미분 필터·D항 클램프를 추가했다.

| 항목 | TI | STM32 |
|------|----|-------|
| 구성 | **P + I + FF** | **P + I + D + FF** |
| Kd_spd | **0.0** (D항 없음) | **0.0008** A·s/RPM |
| dN (미분 필터 극점) | **0.0** (필터 없음) | **200 rad/s** (~31.8 Hz) |
| UdOutMax | **없음** | **maxCurrent × 0.1f** |

D항 추가 이유: 방향 역전 시 오차 급변으로 Iq_ref 스파이크 발생 → 전류 포화 방지.
`UdOutMax = maxCurrent × 0.1f`로 D항 기여분을 최대 전류의 10%로 제한하여 과도한 D항 킥 억제.

---

### E.3 PID 게인

단위가 다르므로 수치 직접 비교 불가. 구조적 대응만 기술.

| 파라미터 | TI | STM32 | 단위 |
|---------|-----|-------|------|
| Kp_spd | 25.0 | 0.08 | TI: PU/PU, STM32: A/RPM |
| Ki_spd | 250.0 | 2.0 | TI: PU/PU, STM32: A/(RPM·s) |
| Kd_spd | **0.0** | **0.0008** | — / A·s/RPM |
| Kff_spd | 4.5 | 0.1 | TI: PU/PU, STM32: A/(RPM/tick) |
| dN_spd | **0.0** | **200.0** | — / rad/s |
| Kout | 1.0 | 1.0 | — |
| outMin/Max | ±maxCurrent (PU) | ±maxCurrent (A) | 단위만 다름 |

---

### E.4 Trajectory — 동일

양쪽 모두 K=0.010 기반 1차 트래커 + maxDelta 가속도 제한.

```
delta     = clamp(K × (target − intValue),  ±maxDelta)
dValue    = delta          ← 가속도 피드포워드
intValue += delta
```

| 항목 | TI | STM32 |
|------|----|-------|
| K 값 | 0.010 | **0.010** (동일) |
| 동작 | 지수 수렴 + maxDelta 제한 | 동일 |
| 피드포워드 소스 | `TRAJ_getDValue()` | `traj_dValue` |
| 단위 | PU/tick | RPM/tick |

K=0.010의 의미: 오차의 1%씩 추적 → 시정수 ≈ 100tick (= 100 ms @ 1 kHz 속도 루프).
오차 > maxDelta/K = 100 × maxDelta 이면 maxDelta에 클램프 → 순수 레이트 리미터 동작.

---

### E.5 Anti-windup — 동일

양쪽 모두 `I_windup = 0` 고정 (백트래킹 비활성).

```c
// TI pid.h
obj->I_windup = _IQ(0.0);  //*pOutValue - g;  ← 주석 처리됨

// STM32 pid.cpp
handle->I_windup = 0.0f;   // backtracking disabled — caused oscillation
```

포화 감지는 `UiSatFlag` + `windupCount` 카운터로 경고 플래그(`kWindupSpeedError`) 생성.
cascade 구조(속도→전류 이중 루프)에서 백트래킹 항이 진동을 유발하므로 양쪽 모두 비활성.

---

### E.6 속도 피드백 부호

```cpp
// TI
Speed_krpm = encoder.getVelocity();

// STM32
Speed_rpm = -encoder.getVelocity();  // 반전: 인코더 감소 = 정방향 토크
```

하드웨어 배선 차이(저측 션트 + 인코더 방향)로 인한 부호 반전. 제어 알고리즘 자체는 동일.

---

### E.7 속도 추정 소스

| 항목 | TI | STM32 |
|------|----|-------|
| 방법 | T-method (QEP HW 캡처 유닛) | T-method (TIM3/4 HW 캡처 + SW `trackPeriod()`) |
| capture_seen 역전 보호 | HW 모드 전용 | **HW + SW 양 모드** (STM32 추가) |
| 속도 필터 | 2개 1차 LPF | 1개 EMA (LPF fc=10 Hz) |

---

### E.8 E-Stop 통합 — 동일

```
// 정상 모드
PID_run_spd(pid, traj_intValue,       Speed, traj_dValue,      &out)

// E-Stop 모드
Calculate_EStop_Scurve(...)
PID_run_spd(pid, estop.cmd_velocity,  Speed, estop.cmd_acc,    &out)
```

S-커브 레퍼런스와 가속도 피드포워드를 속도 PID에 그대로 주입하는 구조 양쪽 동일.

---

### E.9 종합 비교표

| 항목 | TI C2000 | STM32G474 | 차이 |
|------|---------|-----------|------|
| 신호 도메인 | PU krpm `_iq` | SI RPM / A | 단위 변경 |
| PID 구성 | P + I + FF | **P + I + D + FF** | STM32 D항 추가 |
| D항 / dN | 없음 / 없음 | Kd=0.0008 / 200 rad/s | STM32 추가 |
| UdOutMax | 없음 | maxCurrent × 0.1f | STM32 추가 |
| Trajectory K | 0.010 | 0.010 | **동일** |
| Anti-windup 백트래킹 | `I_windup=0` (비활성) | `I_windup=0` (비활성) | **동일** |
| 속도 피드백 부호 | 정방향 | 반전(-) | HW 배선 차이 |
| E-Stop 통합 | 있음 | 있음 | **동일** |
| capture_seen SW 모드 | 없음 | **추가** | STM32 개선 |

---

### E.10 주의사항 및 개선 검토 항목

**① D항 추가 — 속도 추정 노이즈 주의**

`dN=200 rad/s` (~31.8 Hz) 필터가 고주파 노이즈를 억제하나,
저속·저분해능 인코더 환경에서 속도 추정 리플이 Iq_ref 리플로 이어질 수 있다.
증상 발생 시 `Kd_spd` 축소 또는 `dN_spd` 감소(예: 50~100 rad/s) 검토.

**② Anti-windup 비활성 — 장시간 포화 시 회복 특성**

백트래킹 없이 적분기가 포화 한계(`UiOutMax`)에 고정된 채 유지된다.
부하 토크가 `maxCurrent` 초과 지속 시 목표 해제 후 오버슈트 발생 가능.
cascade 구조를 고려한 조건부 백트래킹(내부 루프 포화 신호 연동) 재설계를 장기 개선으로 검토.

**③ TI Kp 실효값 — 게인 비교 참고**

TI Kp=25.0 (PU/PU)의 SI 환산 시 fullScaleSpeed, fullScaleCurrent 값이 필요하여 정확한 수치 비교 불가.
현재 하드웨어에서 STM32 Kp=0.08 A/RPM이 안정 운전 중이라면 유효하며,
TI 원본 대비 실효 대역폭은 실험적 Bode plot 측정으로 검증 권장.

---

*끝.*
