---
title: "속도 제어 알고리즘"
weight: 1
draft: false
---
# 속도 제어 알고리즘

## RPM 측정 알고리즘

`DriveSystem` 클래스의 `measureRpm()` 함수가 RPM 측정을 담당합니다.

1. **DMA 입력 캡처 버퍼 사용**

   - DMA가 버퍼(`m_icDataCh`)에 캡처 값을 순환 저장  
   - DMA의 `NDTR` 값으로 최근 측정 인덱스 식별

2. **주기({{< katex >}}\Delta t{{< /katex >}}) 계산**

   - 최근 인덱스와 이전 인덱스의 타이머 값 차이 계산  
   - {{< katex >}}\Delta t{{< /katex >}} (초) = (현재 캡처 값 - 과거 캡처 값) / 타이머 주파수(`TIMER_FREQ`)

3. **주파수를 RPM으로 변환**

   {{< katex display=true >}}
   RPM = \frac{60 \times f}{PULSE\_PER\_REV}
   {{< /katex >}}


   - 예) 엔코더 1024 펄스/회전 시, `PULSE_PER_REV = 1024`

4. **정지 상태 판정**

   - DMA에 데이터 변화가 없거나 펄스 간격이 비정상이면 정지 상태로 판단

### RPM 측정 코드

```cpp
double DriveSystem::measureRpm()
{
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(&m_hdmaIC);
    int currentIndex = wrapIndex(IC_BUF_SIZE - ndtr - 1, IC_BUF_SIZE);
    int prevIndex = wrapIndex(currentIndex - 2, IC_BUF_SIZE);

    uint32_t currentTime = m_icDataCh[currentIndex];
    uint32_t prevTime    = m_icDataCh[prevIndex];

    if (prevTime == 0)
        return 0.0f;

    double dt = static_cast<double>(currentTime - prevTime) / static_cast<double>(TIMER_FREQ);
    if (dt <= 0.0f || isZeroSpeedDetected(currentIndex, 0.4f))
        return 0.0f;

    double frequency = 1.0f / dt;
    double rpm = (60.0f * frequency) / PULSE_PER_REV;
    return rpm;
}
```

### 정지 상태 검출 코드

```cpp
bool DriveSystem::isZeroSpeedDetected(int currentIndex, double threshold)
{
    if (m_icDataCh[m_prev_index] == m_icDataCh[currentIndex])
        m_zero_sampling_stack += 1;
    else
        m_zero_sampling_stack = 0;

    if (m_zero_sampling_stack > 50)
        return true;

    m_prev_index = currentIndex;

    // 추가 펄스 간격 비정상 여부 체크 로직
    return false;
}
```

## STM32 타이머 설정

### TIM2 설정

```c
void MX_TIM2_Init(void)
{
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1700 - 1;  // 예: 170 MHz → 100 kHz (10 µs)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // Input Capture 설정, 양쪽 엣지 감지
}
```

### PWM 타이머 설정

- TIM1, TIM4 등 PWM 모드 설정
- PWM 듀티 사이클 변경으로 모터 속도 및 방향 제어

## 결론

- 입력 캡처 + DMA로 정확한 RPM 측정
- PID 제어를 통한 정밀한 모터 제어
- 타이머 설정을 통해 효율적이고 정확한 속도 제어 구현


