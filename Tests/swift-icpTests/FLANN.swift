import simd
@testable import swift_icp
import XCTest

final class FLANN_Test: XCTestCase {
    let set1: [simd_float3] = [
        .init(0.500000, 0.500000, 0.500000),    // 0
        .init(-0.250000, -0.250000, -0.250000), // 1
        .init(0.500000, -0.400000, 0.500000),   // 2
        .init(0.500000, 0.500000, -0.400000),   // 3
        .init(0.000000, 0.000000, 0.000000),    // 4
        .init(-0.400000, -0.400000, 0.500000),  // 5
        .init(0.500000, -0.400000, -0.400000),  // 6
        .init(-0.400000, 0.500000, 0.500000),   // 7
        .init(-0.400000, 0.500000, -0.400000),  // 8
        .init(0.250000, 0.250000, 0.100000),    // 9
    ]

    override func setUp() {
    }

    func test_query() {
        let flann = FLANN(points: set1)
        let r = flann.query(point: .zero, neighbors: 3)
        XCTAssertEqual(r.count, 3)
        XCTAssertEqual(r.first!.index, 4) // 0,0,0

        let r2 = flann.query(index: 0, neighbors: 2)
        XCTAssertEqual(r2.count, 2)
        XCTAssertEqual(r2[0].index, 0) // self
        XCTAssertEqual(r2[1].index, 9) // nearest
    }

    func test_squaredDistance_exactMatch() {
        let flann = FLANN(points: set1)

        // 정확히 일치하는 point 의 squaredDistance 는 0
        let r = flann.query(point: set1[4], neighbors: 1) // (0,0,0)
        XCTAssertEqual(r.first!.index, 4)
        XCTAssertEqual(r.first!.squaredDistance, 0, accuracy: 1e-6)
    }

    func test_squaredDistance_knownValue() {
        // 거리를 계산하기 쉬운 점들로 검증
        let points: [simd_float3] = [
            .init(0, 0, 0), // 0
            .init(3, 0, 0), // 1: distance=3, squaredDistance=9
            .init(0, 4, 0), // 2: distance=4, squaredDistance=16
            .init(1, 1, 1), // 3: distance=sqrt(3), squaredDistance=3
        ]
        let flann = FLANN(points: points)

        let r = flann.query(point: .zero, neighbors: 4)
        XCTAssertEqual(r.count, 4)

        // 가까운 순서대로 반환
        XCTAssertEqual(r[0].index, 0) // self(0,0,0)
        XCTAssertEqual(r[0].squaredDistance, 0, accuracy: 1e-6)

        XCTAssertEqual(r[1].index, 3) // (1,1,1), squaredDistance = 3
        XCTAssertEqual(r[1].squaredDistance, 3, accuracy: 1e-4)

        XCTAssertEqual(r[2].index, 1) // (3,0,0), squaredDistance = 9
        XCTAssertEqual(r[2].squaredDistance, 9, accuracy: 1e-4)

        XCTAssertEqual(r[3].index, 2) // (0,4,0), squaredDistance = 16
        XCTAssertEqual(r[3].squaredDistance, 16, accuracy: 1e-4)
    }

    func test_squaredDistance_queryByIndex() {
        let points: [simd_float3] = [
            .init(0, 0, 0), // 0
            .init(1, 0, 0), // 1: squaredDistance from [0] = 1
            .init(0, 2, 0), // 2: squaredDistance from [0] = 4
        ]
        let flann = FLANN(points: points)

        let r = flann.query(index: 0, neighbors: 3)
        XCTAssertEqual(r.count, 3)

        // 자신 포함, squaredDistance 0
        XCTAssertEqual(r[0].index, 0)
        XCTAssertEqual(r[0].squaredDistance, 0, accuracy: 1e-6)

        XCTAssertEqual(r[1].index, 1)
        XCTAssertEqual(r[1].squaredDistance, 1, accuracy: 1e-4)

        XCTAssertEqual(r[2].index, 2)
        XCTAssertEqual(r[2].squaredDistance, 4, accuracy: 1e-4)
    }

    func test_squaredDistance_ordering() {
        let flann = FLANN(points: set1)

        // 여러 neighbor 반환 시 squaredDistance 가 오름차순으로 정렬되어야 함
        let r = flann.query(point: .zero, neighbors: set1.count)
        XCTAssertEqual(r.count, set1.count)

        for i in 1 ..< r.count {
            XCTAssertLessThanOrEqual(
                r[i - 1].squaredDistance, r[i].squaredDistance,
                "index \(i-1)(\(r[i-1].squaredDistance)) > index \(i)(\(r[i].squaredDistance))")
        }
    }

    func test_squaredDistance_matchesSimdDistanceSquared() {
        let flann = FLANN(points: set1)
        let queryPoint = simd_float3(0.1, 0.2, 0.3)
        let r = flann.query(point: queryPoint, neighbors: set1.count)

        for result in r {
            let expected = simd_distance_squared(queryPoint, set1[result.index])
            XCTAssertEqual(result.squaredDistance, expected, accuracy: 1e-4,
                "index \(result.index): squaredDistance \(result.squaredDistance) != expected \(expected)")
        }
    }
}
