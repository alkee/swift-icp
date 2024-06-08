import simd
import swift_icp // @testable 은 internal access 가능하므로 사용하지 않도록
import XCTest

final class ICP_Usage_Test: XCTestCase {
    func test_registerModelToScene() {
        let expected_translation = simd_double3(25, -25, -15)
        let expected_rotation = simd_double3(0.4, -0.35, 0.4).eulerToRotation()
        let expected = simd_double4x4(
            t: expected_translation,
            r: expected_rotation,
            s: .one)

        let pc = load_pointcloud(obj_name: "sample_skin") // floating
        let t_pc = pc.transform(matrix: expected) // reference

        let icp = ICP()
        let result = icp.registerModelToScene(model: pc, scene: t_pc)
        let result_translation = result.transformMatrix.translation
        let result_rotation = result.transformMatrix.rotation

        XCTAssertLessThan(simd_distance(expected_translation, result_translation), 0.1)
        let rotation_diff = simd_sub(expected_rotation, result_rotation).determinant
        XCTAssertLessThan(rotation_diff, 0.1)
    }
}
