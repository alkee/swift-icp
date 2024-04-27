import simd
@testable import swift_icp
import XCTest

final class swift_icpTests: XCTestCase {
    func testExample() throws {
        // XCTest Documentation
        // https://developer.apple.com/documentation/xctest

        // Defining Test Cases and Test Methods
        // https://developer.apple.com/documentation/xctest/defining_test_cases_and_test_methods
        
        let x = [1, 2, 3, 4, 5]
        var y = x
        y[2] = 10
        
        print("** x=\(x)")
        print("** y=\(y)")
        
        let m = matrix_double4x4(1.0) // eye
        print("** \(m)")
    }
    
    func test_KDTree() {
        let points = [
            simd_float3(repeating: -1),
            simd_float3(repeating: 1),
            simd_float3(repeating: 2),
            simd_float3(repeating: 3),
            simd_float3(repeating: 4),
            simd_float3(repeating: 5)
        ]
        let kdt = KDTree.Build(points: points)
        let result = kdt.nearestK(point: simd_float3(0, 0, 0), k: 3)
        print("** \(result)")
    }
}
