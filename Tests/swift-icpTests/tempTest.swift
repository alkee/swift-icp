import simd
@testable import swift_icp
import XCTest

final class tempTest: XCTestCase {
    func test_getTransformMatrix() {
        let e = simd_double3(0, Double.pi * 0.5, 0)
        let t = simd_double3(1.5, 1.5, 1.5)
        let tm = ICP.getTransformMatrix(euler: e, t: t)
        print("** matrix = \(tm)")
    }
    
    func test_pointcloud_copy() {
        let points = [simd_float3(0,0,0), simd_float3(1,2,3), simd_float3(3,2,1) ]
        let normals = [simd_float3(1,0,0), simd_float3(0,1,0), simd_float3(0,0,1) ]
        let src = PointCloud3f(points:points, normals: normals)
        var cpy = src
        cpy.points[1] = simd_float3(-1, -2, -3)
        
        print("** src = \(src.points)")
        print("** cpy = \(cpy.points)")
    }
}
