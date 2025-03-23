---
title: "Science"
weight: 1
draft: false
# bookFlatSection: false
# bookToc: true
# bookHidden: false
# bookCollapseSection: false
# bookComments: false
# bookSearchExclude: false
---
# Science Experiment

**2025 Team Science Mission**

**김로빈 (Robin Kim)**  
한국과학기술원 전기 및 전자 공학부, 대전 305-701  
(발표자 연락처: rlovekaist@kaist.ac.kr)

## 요약

본 연구는 사이언스 미션 중 생명 탐지를 위한 실험 선정 과정을 다룬다. 생명 탐지를 위해 두 가지 주요 접근 방식을 사용하여 총 세 가지 실험을 수행하였다.  첫째, 뷰렛 용액과 브래드포드 용액을 사용한 두 가지 단백질 탐지 실험을 진행하였고,  둘째, 증류수를 용매로 하여 UV-Vis 스펙트럼 스캐닝을 수행하였다.  단백질 탐지 실험으로는 뷰렛 반응, 닌하이드린 반응, 로리 반응, 브래드포드 반응 총 네 가지 실험을 진행하였으며, 미션에 주어진 시간, 실험이 로버 내부에서 진행된다는 점, 단백질 탐지의 정밀도 등을 고려하여 뷰렛 반응과 브래드포드 반응을 최종적으로 선정하였다.

## Abstract

This study outlines the process of selecting experiments for life detection as part of the Science Mission. Two major approaches were employed for life detection, resulting in the execution of three distinct experiments. These included two protein detection experiments using Biuret solution and Bradford solution, as well as UV-Vis spectral scanning with distilled water as the solvent. Four protein detection reactions — Biuret, Ninhydrin, Lowry, and Bradford — were conducted. Considering factors such as the allocated mission time, the experiment being conducted inside the rover, and the precision required for protein detection, the  
Biuret reaction and Bradford reaction were selected as the most suitable methods.

**Keywords:** 단백질 탐지, UV-Vis 스펙트럼 스캐닝, 뷰렛 반응, 브래드포드 반응  
**Keywords:** Protein Detection, UV-Vis Spectral Scanning, Biuret Reaction, Bradford Reaction

# 1. 서 론

## 스펙트로포토미터를 이용한 단백질 탐지

시료 내부의 단백질 양을 정량적으로 계산하는 방법에는 컬러메트릭(colorimetric) 방법과 전기영동(electrophoresis) 방법이 있다.  단백질 정량의 시초는 시료를 황산(sulfuric acid)에서 분해하고 증류(distillation) 과정을 거쳐 남은 염산(HCl)의 양을 이용해  시료 내 단백질 양을 측정하는 Kjeldahl method이다. 하지만 스펙트로포토미터를 사용하면 보다 쉽게 단백질의 양을 계산할 수 있다.  이 방법은 단백질과 결합하여 색을 띄는 시약을 사용하는 방식으로, 시약이 단백질에 결합했을 때 나타나는 색의 파장대를 이용해  해당 파장에서의 흡광도를 측정하면 시약이 단백질에 결합한 정도를 알 수 있다. 이 방법을 단백질 정량 중 컬러메트릭 방법이라고 한다.

컬러메트릭 방법에는 다음의 시험들이 있다:

- Amino Acid analysis
- UV absorption
- 뷰렛 반응 (Biuret)
- 로리 반응 (Lowry)
- 비신코니닉산(BCA) 시험

각 시험은 민감도, 소요 시간, 간섭 물질 저항력 등이 다르다. 아래 표는 각 시약 별 주요 특성을 정리한 것이다.

| 시험명                      | 흡광도 (nm) | 작용 기전                    | 장점                    | 표준액 범위              |
|-----------------------------|-------------|------------------------------|-------------------------|--------------------------|
| Amino Acid analysis         | 570         | 단백질 가수분해, 아미노산 유도체화  | 높은 민감도              | 10~200 μg/mL             |
| UV absorption               | 280         | Tyrosine/tryptophan 흡수      | 소요시간 적음, 검체량 적음  | 0.2~2 mg/mL              |
|                             | 205         | 펩티드 결합 부위              | 비단백 흡수 가능성 높음    | 0.2~2 mg/mL              |
| Biuret                      | 540~560     | 구리 환원 (Cu²⁺ → Cu¹⁺)         | detergent 저항력 큼, 민감도 낮음 | 0.5~10 mg/mL         |
| Lowry                       | 500~750     | 구리 환원, Folin 시약 반응      | 높은 민감도, 간섭 물질 저항력 낮음 | 5~100 μg/mL         |
| Bicinchoninic acid (BCA)    | 562         | 구리 환원, BCA 반응            | detergent, denaturing agent 저항력 큼, reducing agent 저항력 낮음 | 10~1200 μg/mL  |
| Bradford                    | 470         | Coomassie Brilliant Blue 복합체 형성 | 짧은 소요시간, reducing agent 저항력 큼, detergent 저항력 낮음 | 0.1~1 mg/mL     |

시약과 표준액의 반응물을 스펙트로포토미터에 넣고 해당 시약의 최대 흡광도 파장에서 흡광도를 측정한다. 시약의 농도에 따른 흡광도 값을 이용하여 선형 그래프를 작성하고, 일차함수를 통해 시료의 단백질 농도를 추정할 수 있다. 뷰렛, 닌하이드린, 로리, 브래드포드 반응은 모두 계란흰자 표준액을 사용하여 실험되었다.

# 2. 실험 방법

본 연구에서 수행된 실험 방법은 다음과 같이 진행되었다.

## 2.1 단백질 탐지 실험

실험 전, 흙탕물 샘플을 준비하여 흙 입자가 흡광도에 미치는 영향을 최소화하였다. 샘플 준비 후 다음의 실험들이 진행되었다:

- **뷰렛 반응**  
  1% CuSO₄ 용액과 5% NaOH 용액을 준비하고,  날계란의 흰자만을 분리하여 0.1%, 0.2%, …, 1% 농도의 계란흰자 표준 용액을 제조. 각 용액을 정해진 피펫팅 방식으로 큐벳에 담아 540nm 및 560nm에서 흡광도를 측정하였다.

- **로리 반응**  
  Na₂CO₃와 NaOH, CuSO₄·5H₂O 용액을 준비한 후 Lowry reagent A를 제조하고,  Folin Ciocalteu 용액과 반응시켜 620nm와 750nm에서 흡광도를 측정하였다.

- **닌하이드린 반응**  
  10 mL 에탄올(또는 아세톤)에 0.2 g 닌하이드린을 녹여 2% 닌하이드린 용액을 제조. 이후, 계란흰자 표준액 및 트리신 용액을 사용하여 570nm와 440nm에서 흡광도를 측정하였다.

- **브래드포드 반응**  
  Coomassie Brilliant Blue G-250 염료를 이용하여 시약 용액을 제조한 후, 595nm 및 465nm에서 흡광도를 측정하였다.

## 2.2 UV-VIS 스펙트럼 스캐닝

- **증류수 용매 사용**  
  종이 필터, 0.45 μm 시린지 필터, 0.2 μm 시린지 필터를 순차적으로 사용하여 300nm에서 700nm (또는 확장된 범위 200nm~1000nm)까지 5nm 간격으로 흡광도를 측정하였다. 데이터는 비선형 (exponential) 또는 피크 피팅을 통해 분석되었다.

- **에탄올 용매 사용**  
  에탄올에 용해되는 성분 분석을 위해 동일한 방식의 실험을 반복하였다.

# 3. 실험 결과

## 3.1 뷰렛 반응 결과

실험은 두 차례 진행되었으며, blank 포함 및 미포함 실험 결과가 각각 산출되었다.

**(1) 뷰렛 반응 1 (Blank 미포함)**

Egg albumin (%) 및 흡광도 측정 결과:

| Egg albumin (%) | 540nm Abs | 560nm Abs |
|-----------------|-----------|-----------|
| 0.1             | 0.202     | 0.226     |
| 0.2             | 0.244     | 0.269     |
| 0.3             | 0.281     | 0.307     |
| 0.4             | 0.356     | 0.392     |
| 0.5             | 0.319     | 0.347     |
| 0.6             | 0.382     | 0.414     |
| 0.7             | 0.351     | 0.378     |
| 0.8             | 0.394     | 0.421     |
| 0.9             | 0.448     | 0.472     |
| 1.0             | 0.449     | 0.467     |

*표 2. 뷰렛 반응 1 결과*

Soil sample 결과:

| 540nm Abs | 0.258 |
|-----------|-------|
| 560nm Abs | 0.275 |

*표 3. 뷰렛 반응 1 soil sample 결과*

**(2) 뷰렛 반응 2 (Blank 포함)**

Blank 및 각 농도별 흡광도:

| Egg albumin (%) | 540nm Abs (Blank 포함) |
|-----------------|------------------------|
| blank           | -0.062                 |
| 0.1             | 0.091                  |
| 0.2             | 0.006                  |
| 0.3             | 0.165                  |
| 0.4             | 0.149                  |
| 0.5             | 0.177                  |
| 0.6             | 0.187                  |
| 0.7             | 0.223                  |
| 0.8             | 0.210                  |
| 0.9             | 0.226                  |

(560nm Abs의 결과도 유사하게 측정됨; 표 4 참조)

Soil sample (뷰렛 반응 2):

| 540nm Abs | 0.110, 0.035, 0.059, 0.064 |
|-----------|----------------------------|
| 560nm Abs | 0.109, 0.027, 0.044, 0.050 |

*표 5. 뷰렛 반응 2 soil sample 결과*

Origin 2025를 사용한 Linear Fitting 결과 (예: 540nm, 560nm 각각의 slope, intercept, Adj R-square 등)는 표 6, 표 7, 표 8, 표 9에 상세히 기술되었으며, 이를 통해 계란흰자 농도와 soil 내 단백질 백분율을 산출하였다.

각 샘플별 단백질 백분율 (540nm, 560nm):

| Sample | 540nm | 560nm |
|--------|-------|-------|
| Soil 1 | 0.056%| 0.06% |
| Soil 2 | 0.022%| 0.025%|
| Soil 3 | 0.033%| 0.032%|
| Soil 4 | 0.035%| 0.035%|

*표 10. 뷰렛 반응 2 단백질 백분율*

## 3.2 로리 반응 결과

로리 반응은 blank 포함 1회 수행되었으며, 계란흰자 농도에 따른 620nm 및 750nm 파장에서의 흡광도가 측정되었다.

계란흰자 표준 결과:

| Egg albumin (%) | 620nm Abs | 750nm Abs |
|-----------------|-----------|-----------|
| 0.005           | 0.001     | 0.005     |
| 0.006           | 0.010     | 0.021     |
| 0.007           | 0.051     | 0.040     |
| 0.008           | 0.044     | 0.030     |
| 0.009           | 0.059     | 0.046     |

*표 11. 로리 반응 결과*

Soil sample (로리 반응):

| Sample   | 620nm Abs | 750nm Abs |
|----------|-----------|-----------|
| Sample 1 | 0.070     | 0.044     |
| Sample 2 | 0.042     | 0.028     |
| Sample 3 | 0.005     | -0.006    |
| Sample 4 | 0.313     | 0.302     |

*표 12. 로리 반응 soil sample 결과*

Origin 202 
