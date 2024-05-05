import ModelIO
import SceneKit
import simd
@testable import swift_icp // @testable 은 internal access 가능하도록 해줌
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
                .init(-0.250000, -0.250000, -0.250000),
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
            ])
        modelBox = sampleTransform.toFloat4x4() * sceneBox
    }
    
    func test_samplePCUniform() {
        let pc = load_pointcloud(obj_name: "sample_skin")
        XCTAssertEqual(pc.points.count, 10002)
        let halfSampled = ICP.samplePCUniform(pc: pc, sampleStep: 2)
        XCTAssertEqual(halfSampled.points.count, 5001)
        XCTAssertEqual(halfSampled.normals.count, 5001)
        let quaterSampled = ICP.samplePCUniform(pc: pc, sampleStep: 4)
        XCTAssertEqual(quaterSampled.points.count, 2501)
        XCTAssertEqual(quaterSampled.normals.count, 2501)
    }
    
    func test_transformPCPose() {
        let t_pc = ICP.transformPCPose(pc: sceneBox, pose: sampleTransform)
        let md0 = mean_distance(sceneBox.points, t_pc.points)
        print("** md0 = \(md0)")
        
        let md1 = mean_distance(t_pc.points, modelBox.points)
        print("** md1 = \(md1)") // should be almost 0
        let md2 = mean_distance(sceneBox.points, modelBox.points)
        print("** md2 = \(md2)")
        
        print("-------------------")
        
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
    
    func test_norm_l2() {
        let p1:[simd_float3] = [
            .init(0, 0, 0),
            .init(1, 1, 1),
        ]
        let p2:[simd_float3] = [
            .init(0, 1, 0),
            .init(1, 0, 1),
        ]
        let l2n = ICP.norm_l2(p1, p2)
        
        let distances = [1.0, 1.0]
        let sum = distances.reduce(0, +)
        
        XCTAssertEqual(l2n, sum, accuracy: 0.001)
    }
    
    
    func test_registerModelToScene() {
        let pc = load_pointcloud(obj_name: "sample_skin")
        let t_pc = sampleTransform.toFloat4x4() * pc

        let md0 = mean_distance(pc.points, t_pc.points)
        print("** md0 = \(md0)")

        let icp = ICP()
        let result = icp.registerModelToScene(model: pc, scene: t_pc)
        print("** residual = \(result.residual)")
        print("** result = \(result.transformMatrix)")
        print("** expected = \(sampleTransform)")

        let transformed = result.transformMatrix.toFloat4x4() * pc
        let md = mean_distance(t_pc.points, transformed.points)
        print("** md = \(md)")
    }

}
