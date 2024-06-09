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
        XCTAssert(r.count == 3)
        XCTAssert(r.first!.index == 4) // 0,0,0
        
        let r2 = flann.query(index: 0, neighbors: 2)
        XCTAssert(r2.count == 2)
        XCTAssert(r2[0].index == 0) // self
        XCTAssert(r2[1].index == 9) // nearest
    }
}
