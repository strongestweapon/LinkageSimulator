# FingerSimulator

2D 평면 링키지 에디터 + 기구학 시뮬레이터. 로봇 손가락 메커니즘 분석용 (Nature Communications 2021 소프트 핑거 논문 기반). 단일 HTML 파일 — 빌드 도구, 외부 의존성 없음.

## 기능

**에디터 모드**
- 클릭으로 링크(폴리라인 강체) 점 단위로 그리기
- 점을 드래그해서 다른 링크에 스냅 → 조인트 생성
- 점 우클릭으로 역할 설정 (ground / input / friction)
- 링크 우클릭으로 삭제
- 그리드 스냅, 포인트/링크 레이블, 레이블 크기 토글
- 전체 undo 히스토리 (Ctrl+Z)
- 텍스트 패널: 메커니즘 전체를 텍스트로 직렬화/역직렬화 — 직접 편집 후 apply 가능

**시뮬레이션 모드**
- 최대 3개 rest 설정 저장 (R1 / R2 / R3) — 각각 정지 포즈 스냅샷
- `input` 또는 `friction` 포인트 드래그 → 실시간 기구학 솔버 동작
- "Go R1/R2/R3"로 저장된 rest 포즈 복원
- 올-오어-낫씽 프레임 롤백: 기구학적 데드존 진입 시 마지막 유효 프레임으로 즉시 복귀 (부분 이동 없음)
- 멀티 입력 지원: input 핸들 하나를 드래그하면 나머지 input 포인트는 메커니즘을 따라 자유롭게 이동

## 역할(Role) 종류

| 역할 | 표시 | 동작 |
|---|---|---|
| **ground** | 채워진 원 + 삼각형 + 해칭 | rest 위치에 고정. 2개 이상의 ground 포인트가 있는 바디는 완전 고정. 드래그 불가. |
| **input** | 파란 원 + 위쪽 화살표 | 기구학 드래그 핸들. 드래그 중 커서 위치에 강한 핀 (weight 100). |
| **friction** | 점선 원 | 기구학 드래그 핸들이면서 시뮬레이션 중 soft 저항. Weight 8 (input 100 대비 1/12) — 움직이지만 저항감 있음. |

## 솔버

`linkage-solver.js` — 2D 평면 링키지용 Levenberg-Marquardt 가중 최소제곱 솔버.

- 링크 = 3-DOF 강체 (x, y, angle). 포인트는 무게중심 기준 로컬 오프셋으로 저장.
- 모든 제약 = 가중 핀 방정식: `body.worldPt(p) == target` (핀당 2개 방정식)
- 조인트 = 두 바디의 공유 월드 포인트가 일치해야 한다는 핀 방정식 쌍
- `setFixed(body)`: 완전 고정 바디를 변수 벡터에서 제거

**가중치 설정**

| 제약 | Weight |
|---|---|
| ground 핀 | 100 |
| input 핀 (드래그 중) | 100 |
| 조인트 | 100 |
| friction 핀 | 8 |

ground / input / joint를 동일 weight(100)로 맞춰서 모든 하드 제약이 동등하게 경쟁. friction은 8로 약한 저항.

**범위 제한 (롤백 판단)**

솔버 residual이 아닌 물리적 드리프트(SVG 픽셀 단위)로 판단:
- Ground drift: ground 앵커 포인트가 rest 위치에서 얼마나 이탈했는지
- Joint separation: 조인트 양쪽 월드 포인트 간 거리

서브스텝 중 `max(ground drift, joint separation) > 2.0 px` 이면 → 프레임 전체 롤백. 서브스텝 단위는 커서 이동 8px로 제한해 솔버가 항상 작은 증분 단계를 밟도록 함.

> residual을 쓰지 않는 이유: residual은 제약 개수·weight에 비례해 커지므로 의미 있는 임계값 설정이 불가능. 픽셀 단위 직접 측정이 메커니즘 규모와 무관하게 일관됨.

## 발견한 이슈 및 해결 과정

### 조인트에 ground + input 동시 설정 버그

**원인**: 역할이 ref 단위(`state.roles[ref]`)로 저장됨. 조인트는 서로 다른 링크의 두 ref를 연결하는 구조. 사용자가 거의 같은 위치를 두 번 우클릭할 때 picker가 다른 ref를 집어 서로 다른 역할이 쌓일 수 있었음.

**해결**: 역할을 조인트 그룹 단위로 관리. `getNodeRole(members)` / `setNodeRole(members, role)` API 도입 — 역할 설정 시 그룹 전체 ref를 먼저 초기화한 뒤 canonical rep에만 저장. 렌더러도 `getNodeRole(jointGroup(ref))`로 읽어서 물리적 핀 하나당 마커 하나만 표시.

부가 효과: 조인트 생성 시 ground/input 충돌 검증 — ground와 input 역할을 가진 두 포인트는 조인트로 연결할 수 없음.

### Rest config 저장 시 조인트 드리프트 고착 문제

**원인**: `saveRestConfig`가 각 ref의 현재 `link.points[local]`을 독립적으로 저장. 시뮬레이션 후 같은 조인트의 두 ref가 서로 다른 바디에 있으면 soft constraint 드리프트로 미세하게 다른 위치에 있을 수 있음 (weight 100은 충분히 뻑뻑하지만 무한하지 않음). 이 드리프트가 rest 스냅샷에 굳어져 다음 시뮬레이션이 불일치 상태에서 시작됨.

**해결**: 저장 전 조인트 그룹별 canonical position(정렬 기준 첫 번째 멤버 위치)을 계산해 모든 멤버를 해당 위치로 통일. Rest config 내 조인트 파트너들은 항상 정확히 일치함.

### 메커니즘 일부 이동 후 멈추는 "creep" 현상

**원인**: 롤백이 서브스텝 단위였음. 서브스텝 하나가 drift 체크에 실패하면 그 직전 서브스텝 위치에 머뭄. 사용자 입장에서는 메커니즘이 조금 움직인 뒤 멈춘 것처럼 보여 유효한 포즈로 오해하기 쉬웠음.

**해결**: 각 마우스 이벤트 시작 시점에 전체 프레임 상태(`link.points` 전체 + `kinSolver.bodies` 전체) 스냅샷. 어느 서브스텝이든 실패하면 프레임 전체 복원. 메커니즘은 목표 위치까지 완전히 이동하거나 전혀 이동하지 않음.

### 멀티 input 시 비드래그 input 포인트가 잠기는 문제

**원인**: `solveKinematics`가 매 프레임마다 모든 input 역할 포인트에 high-weight 핀을 추가. 드래그하지 않는 두 번째 input 포인트가 rest 위치에 고정되어 메커니즘 이동을 방해함.

**해결**: 현재 드래그 중인 ref에만 full input 핀(weight 100) 적용. 나머지 input 포인트는 핀 없이 조인트 제약만으로 자유롭게 따라오게 함.

### friction 역할이 텍스트 패널에 출력되지 않는 문제

**원인**: `parse()` 정규식이 `ground`와 `input`만 인식. friction은 직렬화는 정상이었으나 apply 시 조용히 무시됨.

**해결**: 정규식을 `/(ground|input|friction)/`으로 수정.
