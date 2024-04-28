import simd
@testable import swift_icp
import XCTest

final class ICP_Test: XCTestCase {
    
    var modelBox: PointCloud3f!
    var sceneBox: PointCloud3f!
    let sampleTransform = simd_double4x4(
        t: .init(-0.2, -0.2, -0.2),
//        r: matrix_identity_double3x3,
        r: simd_double3x3(simd_double3(0.1, 0.2, 0.3).eulerToQuaternion()),
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
                .init(0.000000, 0.000000, 0.000000), // invalid normal
                .init(0.577350, -0.577350, 0.577350),
                .init(0.577350, 0.577350, -0.577350),
                .init(0.600043, 0.600043, -0.529052),
                .init(-0.529052, 0.600043, 0.600043),
                .init(-0.577350, 0.577350, 0.577350),
                .init(0.600043, -0.529052, 0.600043),
            ]
        )
        modelBox = sampleTransform.toFloat4x4() * sceneBox
    }
    
    func test_registerModelToScene() {
        let md0 = mean_distance(modelBox.points, sceneBox.points)
        print("** md0 = \(md0)")

        let icp = ICP()
        icp.numLevels = 1
        let result = icp.registerModelToScene(model: modelBox, scene: sceneBox)
        print("** residual = \(result.residual)")
        print("** result = \(result.transformMatrix)")
        print("** expected = \(sampleTransform.inverse)")
        
        let transformed = result.transformMatrix.toFloat4x4() * modelBox
        let md = mean_distance(modelBox.points, transformed.points)
        print("** md = \(md)")
    }
    
    func test_minimizePointToPlaneMetric() {
        let md0 = mean_distance(modelBox.points, sceneBox.points)
        let (r0, t0) = ICP.minimizePointToPlaneMetric(src: modelBox, dst: sceneBox)
        print("** md0 = \(md0)")
        print("** r0 = \(r0)")
        print("** t0 = \(t0)")
        let transform0 = simd_double4x4(
            t: t0,
            r: simd_double3x3(r0.eulerToQuaternion()),
            s: .one)
        let it1 = transform0.toFloat4x4() * modelBox
        let md1 = mean_distance(it1.points, sceneBox.points)
        XCTAssertLessThan(md1, md0)
        
        let (r1, t1) = ICP.minimizePointToPlaneMetric(src: it1, dst: sceneBox)
        print("** md1 = \(md1)")
        print("** r1 = \(r1)")
        print("** t1 = \(t1)")

        let transform1 = simd_double4x4(
            t: t1,
            r: simd_double3x3(r1.eulerToQuaternion()),
            s: .one)
        let it2 = transform1.toFloat4x4() * it1
        let md2 = mean_distance(it2.points, sceneBox.points)
        
        print("** md2 = \(md2)")
        print("** reuslt: \(it2.points)")
        
        XCTAssertLessThan(md2, md1)
    }
}
