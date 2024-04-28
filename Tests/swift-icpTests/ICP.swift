import simd
@testable import swift_icp
import XCTest

final class ICP_Test: XCTestCase {
    
    var modelBox: PointCloud3f!
    var sceneBox: PointCloud3f!
    let sampleTransform = simd_double4x4(
        t: .init(-0.2, -0.2, -0.2),
        r: matrix_identity_double3x3,
        s: .one)

    override func setUp() {
        sceneBox = PointCloud3f(
            points: [
                .init(0.500000, 0.500000, 0.500000),
                .init( -0.250000, -0.250000, -0.250000),
                .init(0.500000, -0.400000, 0.500000),
                .init(0.500000, 0.500000, -0.400000),
                .init(-0.400000, -0.400000, 0.500000),
                .init(0.500000, -0.400000, -0.400000),
                .init(-0.400000, 0.500000, 0.500000),
                .init(-0.400000, 0.500000, -0.400000),
            ],
            normals: [
                .init(0.577350, 0.577350, 0.577350),
                .init(0.000000, 0.000000, 0.000000),
                .init(0.577350, -0.577350, 0.577350),
                .init(0.577350, 0.577350, -0.577350),
                .init(0.600043, 0.600043, -0.529052),
                .init(-0.529052, 0.600043, 0.600043),
                .init(-0.577350, 0.577350, 0.577350),
                .init(0.600043, -0.529052, 0.600043),
            ]
        )
        modelBox = ICP.transformPCPose(pc: sceneBox, pose: sampleTransform)
    }
    
    func test_ICP() {
        let icp = ICP()
        icp.numLevels = 1
        let result = icp.registerModelToScene(model: modelBox, scene: sceneBox)
        print("** residual = \(result.residual)")
        print("** matrix = \(result.transformMatrix)")
    }
}
