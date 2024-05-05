import simd
@testable import swift_icp
import XCTest

final class KDTree_Test: XCTestCase {
    let set1: [simd_float3] = [
        .init(0.500000, 0.500000, 0.500000),
        .init(-0.250000, -0.250000, -0.250000),
        .init(0.500000, -0.400000, 0.500000),
        .init(0.500000, 0.500000, -0.400000),
        .init(-0.400000, -0.400000, 0.500000),
        .init(0.500000, -0.400000, -0.400000),
        .init(-0.400000, 0.500000, 0.500000),
        .init(-0.400000, 0.500000, -0.400000),
    ]

    var tree1: KDTree<simd_float3>!
    override func setUp() {
        tree1 = KDTree.Build(points: set1)
    }

    func test_nearestK() {
        let inds1 = tree1.nearestK(point: .one)
        XCTAssertEqual(inds1.count, 1)
        let i1 = inds1.first!.index
        XCTAssertEqual(i1, 0)

        let inds2 = tree1.nearestK(point: .one * -1, k: 2)
        XCTAssertEqual(inds2.count, 2)
        XCTAssertEqual(inds2[0].index, 1)
        XCTAssertEqual(inds2[1].index, 4)
    }
}
