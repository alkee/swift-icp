import simd
@testable import swift_icp
import XCTest

final class FLANN_Test: XCTestCase {
    let set1: [simd_float3] = [
        .init(0.500000, 0.500000, 0.500000),
        .init(-0.250000, -0.250000, -0.250000),
        .init(0.500000, -0.400000, 0.500000),
        .init(0.500000, 0.500000, -0.400000),
        .init(0.000000, 0.000000, 0.000000),
        .init(-0.400000, -0.400000, 0.500000),
        .init(0.500000, -0.400000, -0.400000),
        .init(-0.400000, 0.500000, 0.500000),
        .init(-0.400000, 0.500000, -0.400000),
        .init(0.250000, 0.250000, 0.100000),
    ]

//    var tree1: KDTree<simd_float3>!
    override func setUp() {
//        tree1 = KDTree.Build(points: set1)
    }

    func test_query() {
        let flann = FLANN(points: set1)
        let r = flann.query(point: .zero, neighbors: 3)
        print("**** r=\(r)")
        let rr = flann.query(index: 0, neighbors: 3)
        print("**** r=\(rr)")

    }
}
