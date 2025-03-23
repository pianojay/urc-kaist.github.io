# URC KAIST Documentation / Website

HUGO라는 툴 써서 만들었어여

## HUGO 설치하기
이 [링크](https://gohugo.io/installation/) 따라하면 돼여

**세줄요약**
1. MacOS면그냥 `brew install hugo` 
2. 우분투(wsl)이면 `sudo apt install hugo` 
3. 윈도우를 왜써여

## 문서는 어떻게 써여?
`./content.en/`에는 영어 문서, `./content.kr/`에는 한국어 문서가 들어있어여. 
새롭게 만들고 싶으면 원하는 위치에 `<파일 이름>.md` 파일을 만들면 돼여.
보면 알겠지만 폴더 구조와 동일하게 문서가 홈페이지에 표시돼여.
세부 폴더를 만들 수 있고, 각 폴더의 `_index.md` 파일에는 그 폴더 자체의 내용을 담을 수 있어여.

각 문서 작성할 때 파일 이름은 영어로 동일하게, 파일 안에 문서 이름과 내용은 각각 한국어/영어로 해주면 돼여.

각 문서의 시작에는 다음과 같이 메타데이터를 달아주세여

```md
---
title: "제목(한글 혹은 영어)"
weight: 1 # 이건 폴더 순서인데, 적은게 더 먼저 나와여
draft: false
bookCollapseSection: true # 이거는 _index.md 에서 폴더를 접을 수 있는지 여부에여
---
```

질문 있으면 저를 갈궈주세여
