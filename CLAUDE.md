# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project 개요

OpenCV contrib의 surface matching ICP(Iterative Closest Point) algorithm을 Swift로 port한 library.
3D point cloud 간의 rigid body transformation(rotation + translation)을 추정하는 algorithm.

- 원본 참조: https://github.com/opencv/opencv_contrib/tree/master/modules/surface_matching
- Swift Package Manager 기반, Swift 5.10+

## Build / Test

```bash
swift build
swift test

# 특정 test class 실행
swift test --filter ICP_Test
swift test --filter ICP_Usage_Test
swift test --filter FLANN_Test
```

lint 설정 없음.

## Dependencies

- **LASwift** (0.3.2): linear algebra 연산 (least squares solving 등). `LASwift`의 `+` operator가 array concat이 아닌 elementwise addition이므로 주의
- **SwiftAnnoy** (1.0.1): approximate nearest neighbor search (FLANN 대체)

## Architecture

### 핵심 type

- **`PointCloud<T>`** (`PointCloud.swift`): generic point cloud container. `points`와 `normals` array로 구성
  - `PointCloud3f` = `PointCloud<simd_float3>` typealias로 주로 사용
  - `SCNGeometry`로부터 생성, PLY export, transform, sampling 등 지원
- **`ICP`** (`ICP.swift`): ICP algorithm class. `registerModelToScene(src:dst:)` 가 진입점
  - `src` = floating(model) point cloud, `dst` = reference(scene) point cloud
  - `dst`는 반드시 유효한 normals를 가져야 함 (point-to-plane metric 사용)
  - multi-level pyramid 방식으로 coarse-to-fine refinement 수행
  - Kok Lim Low의 point-to-plane linearization, picky ICP, MAD 기반 outlier rejection 적용
- **`TransformResult`** (`ICP.swift`): 결과값. `simd_double4x4` transformation matrix와 `residual`
- **`FLANN`** (`FLANN.swift`): `SwiftAnnoy` wrapper. nearest neighbor search 제공

### SIMD extensions (`Extensions/`)

- `simd+float.swift`, `simd+double.swift`: `simd_float3`/`simd_double3` array에 대한 연산 (`mean()`, `totalLength()`, `mean_distance()` 등), `eulerToRotation()`, matrix 분해(`rotation`, `translation` property)
- `simd+operators.swift`: array-vector 산술 operator (`-`, `*=`)

### Test 구조

- `ICP.swift`: `@testable import`로 internal API(sampling, minimization) 단위 test
- `ICP+usage.swift`: public API만으로 전체 registration 동작 검증 (의도적으로 `@testable` 미사용)
- `FLANN.swift`: nearest neighbor search test
- `TestHelper.swift`: OBJ file loading, CloudCompare format 변환 등 test utility
- `Resources/sample_skin.obj`: 10,002 vertex의 test 3D model

## 주요 제약사항

- `dst`(reference) point cloud는 유효한 normals 필수 (assert로 검증)
- 유효한 correspondence pair가 6개 이상이어야 least squares solution 가능
- zero vector normal은 sampling 시 기본적으로 필터링됨
- `Sampling.gelfand`는 미구현 상태
