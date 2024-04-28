import simd
@testable import swift_icp
import XCTest
import LASwift

final class exercise: XCTestCase {
    func test_copying() throws {
        let x = [1, 2, 3, 4, 5]
        var y = x
        y[2] = 10
        
        print("** x=\(x)")
        print("** y=\(y)")
    }
    
    func test_matrix() {
        let m = matrix_double4x4(1.0) // eye
        print("** \(m)")
    }
    
    func test_array() {
        let a: [Double] = [1, 2, 3]
        let b: [Double] = [3, 2, 1]
        let c = a + b // concat 기대했으나.. LASwift 에 의해 elementwise +
        print("** \(c)")
    }
    
    func test_LASwift() {
        // (r, c) : a(m, k) * x(k, n) = b(m, n)
        let a: Matrix = .init(8, 1, 1) // r, c
        let b: Matrix = .init(8, 6, 1)
        let (x, r) = lstsqr(a, b)
        print("** x=(\(x.rows), \(x.cols)), r=\(r)")
    }
}
