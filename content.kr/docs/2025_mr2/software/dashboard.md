---
title: "대시보드 애플리케이션"
weight: 1
draft: false
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---
# 대시보드 애플리케이션

## 개요

[**대시보드 애플리케이션**](https://github.com/URC-kaist/urckaist-dashboard)은 [Tauri](https://tauri.app/), [React](https://reactjs.org/), [Typescript](https://www.typescriptlang.org/), 및 [Vite](https://vitejs.dev/)로 구축된 최신 데스크톱 애플리케이션입니다. 이 애플리케이션은 ROS (Robot Operating System) 백엔드와 상호 작용하도록 설계되었으며, WebSockets를 통한 통신을 위해 [ROSLIB](http://wiki.ros.org/roslibjs)을 활용합니다. 인터페이스는 비디오 스트림, 네트워크 핑, 로그 및 다양한 ROS 토픽과 같은 데이터를 모니터링, 제어 및 시각화할 수 있는 여러 탭을 제공합니다.

## 설치

대시보드 애플리케이션은 다음과 같은 여러 OS 플랫폼에 대해 크로스 컴파일되어 번들됩니다:

- Windows
- macOS (x86_64 / arm64)
- Linux

번들된 애플리케이션은 [릴리즈](https://github.com/URC-kaist/urckaist-dashboard/releases) 섹션에서 다운로드할 수 있습니다.

## 소스에서 설치하기

**필수 조건:**

- Node.js 및 pnpm
- Rust 툴체인 (Tauri에 필요)
- 실행 중인 ROS 환경 (ROS 마스터에 연결할 경우)

**설치 단계:**

1. **리포지토리 클론:**

   ```bash
   git clone https://github.com/URC-kaist/urckaist-dashboard.git
   cd urckaist-dashboard
   ```

2. **노드 의존성 설치:**

   ```bash
   pnpm install
   ```

3. **Rust 환경 설정:**

   Rust 툴체인이 설치되어 있는지 확인하십시오. [https://www.rust-lang.org/tools/install](https://www.rust-lang.org/tools/install)의 지침을 따르세요.

4. **애플리케이션 빌드:**

   Vite를 사용하여 프론트엔드를 빌드하고 Tauri로 애플리케이션을 번들합니다:

   ```bash
   pnpm tauri build
   ```

5. **개발 모드에서 실행:**

   개발 서버를 시작합니다:

   ```bash
   pnpm tauri dev
   ```

## 프로젝트 구조

아래는 프로젝트 구조의 간략한 개요입니다:

```none
.
├── App.css
├── App.tsx
├── MainPage.css
├── MainPage.tsx
├── ROSContext.tsx
├── VideoStream.tsx
├── main.tsx
└── src/
    ├── tabs/
    │   ├── Dashboard/
    │   │   ├── Dashboard.tsx
    │   │   ├── Logs.tsx
    │   │   └── Bms.tsx
    │   ├── Drive/
    │   │   └── Drive.tsx
    │   ├── PID/
    │   │   └── PID.tsx
    │   ├── Science/
    │   │   └── Science.tsx
    │   └── Misc/
    │       └── Misc.tsx
    └── components/
        ├── GamePadVisualizer.tsx
        └── NetworkPing.tsx
```

이 프로젝트는 Tauri, Cargo 및 Vite에 특정한 다양한 자산(아이콘, 이미지, GLB 모델)과 구성 파일도 포함하고 있습니다.

## ROS 통합

이 애플리케이션은 **ROSLIB**을 사용하여 ROS 통신을 통합합니다. `ROSContext.tsx`에 정의된 중앙 컨텍스트는 연결 상태를 관리하고 헬퍼 함수를 제공합니다.

**핵심 구성 요소:**

- **ROSProvider:** 애플리케이션을 감싸 ROS 연결 상태와 함수를 제공합니다.
- **useROS Hook:** 구성 요소가 ROS 연결 세부 정보를 접근하고, 연결을 초기화하며, 연결 이벤트(예: 연결, 오류, 종료)를 처리할 수 있도록 합니다.

**`ROSContext.tsx`의 예제 사용법:**

```typescript
import { createContext, useContext, useState, ReactNode } from 'react';
import ROSLIB from 'roslib';

interface ROSContextType {
  ros: ROSLIB.Ros | null;
  server: string;
  loading: boolean;
  status: 'Connected' | 'Disconnected' | null;
  message: string | null;
  initializeROS: (ip: string, port: string) => void;
}

const ROSContext = createContext<ROSContextType | undefined>(undefined);

export function ROSProvider({ children }: { children: ReactNode }) {
  // 여기에 상태 및 초기화 로직을 추가합니다...
}

export function useROS(): ROSContextType {
  const context = useContext(ROSContext);
  if (!context) {
    throw new Error('useROS는 ROSProvider 내에서 사용되어야 합니다');
  }
  return context;
}
```

## 화면 및 탭

메인 인터페이스는 탭형 대시보드로 구성되어 있습니다 (자세한 내용은 `MainPage.tsx` 참조):

- **대시보드 탭:** 연결된 ROS 토픽, 노드, 비디오 스트림, 네트워크 핑, 로그 및 게임패드 시각화를 개요로 표시합니다.
- **드라이브 탭:** 주행 제어 기능을 제공합니다.
- **PID_Debug 탭:** PID 디버깅 작업을 위한 탭입니다.
- **사이언스 탭:** 과학 미션 관련 기능을 처리합니다.
- **기타 탭:** 추가적인 기타 기능을 포함합니다.

각 탭은 활성 연결 상태에 따라 해당 구성 요소를 동적으로 로드합니다. ROS 연결이 설정되면 탭이 표시되며, 그렇지 않으면 연결 폼이 표시됩니다.

## 의존성

이 프로젝트는 다음과 같은 의존성에 의존합니다:

### TS 의존성
- **Leaflet**: 인터랙티브 지도를 생성하기 위한 JavaScript 라이브러리.
- **Three.js**: 3D 렌더링 라이브러리.
- **ROSLIB**: ROS 통신을 위한 JavaScript 라이브러리.
- **React**: 사용자 인터페이스 구축을 위한 JavaScript 라이브러리.

### Rust 의존성
- **ping**: ICMP 에코 요청을 보내기 위한 Rust 라이브러리.
- **macaddr**: MAC 주소를 다루기 위한 Rust 라이브러리.
- **wol-rs**: Wake-on-LAN 패킷을 보내기 위한 Rust 라이브러리.

## 사용법

1. **애플리케이션 실행:**
   - 개발 서버를 실행하거나, 프로덕션을 위해 애플리케이션을 빌드합니다.
2. **ROS에 연결:**
   - 애플리케이션이 시작되면 ROS 마스터의 IP 주소와 포트를 입력하라는 메시지가 표시됩니다.
   - 연결에 성공하면 대시보드에 ROS 토픽 및 노드의 실시간 데이터가 표시됩니다.
3. **탭 탐색:**
   - 화면 상단의 탭 바를 사용하여 기능(대시보드, 드라이브, PID_Debug, 사이언스, 기타) 간 전환합니다.
4. **연결 해제:**
   - 헤더에 있는 **연결 해제** 버튼을 클릭하여 ROS 연결을 종료합니다.

## 결론

이 대시보드 애플리케이션은 최신 웹 기술과 ROS 통합을 활용하여 로봇 시스템을 모니터링하고 제어하기 위한 강력하고 직관적인 인터페이스를 제공합니다. 추가적인 사용자 정의나 기여에 대한 자세한 내용은 소스 코드와 인라인 문서를 참조하십시오.
