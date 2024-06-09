import ModelIO
import SceneKit
import simd
@testable import swift_icp
import XCTest

final class ICP_Test: XCTestCase {
    var modelBox: PointCloud3f!
    var sceneBox: PointCloud3f!
    let sampleTransform = simd_double4x4(
        t: .init(5, -5, 0),
        r: simd_double3(0.1, 0.2, 0.3).eulerToRotation(),
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
        modelBox = sceneBox.transform(matrix: sampleTransform)
    }
    
    func test_samplePCUniform() {
        let pc = load_pointcloud(obj_name: "sample_skin")
        XCTAssertEqual(pc.points.count, 10002)
        let halfSampled = ICP.samplePCUniform(
            pc: pc,
            sampleStep: 2,
            includeInvalidNormal: true)
        XCTAssertEqual(halfSampled.points.count, 5001)
        let halfSampled_no_invalid_normal = ICP.samplePCUniform(
            pc: pc,
            sampleStep: 2,
            includeInvalidNormal: false)
        XCTAssertEqual(halfSampled_no_invalid_normal.normals.count, 4996)
        
        let quaterSampled = ICP.samplePCUniform(
            pc: pc,
            sampleStep: 4,
            includeInvalidNormal: true)
        XCTAssertEqual(quaterSampled.points.count, 2501)
        let quaterSampled_no_invalid_normal = ICP.samplePCUniform(
            pc: pc,
            sampleStep: 4,
            includeInvalidNormal: false)
        XCTAssertEqual(quaterSampled_no_invalid_normal.normals.count, 2499)
    }
    
//    func test_transformPCPose() {
//        let t_pc = ICP.transformPCPose(pc: sceneBox, pose: sampleTransform)
//        
//        let md0 = sceneBox.points.mean_distance(t_pc.points)
//        let md1 = t_pc.points.mean_distance(modelBox.points)
//        XCTAssertEqual(md1, 0, accuracy: 0.00001)
//        let md2 = sceneBox.points.mean_distance(modelBox.points)
//        XCTAssertEqual(md0 - md2, 0, accuracy: 0.00001)
//    }
        
    func test_minimizePointToPlaneMetric() {
        var model = modelBox!
        let stepCount = 50

        let md0 = sceneBox.points.mean_distance(model.points)
        print("** initial meandistance = \(md0)")

        for i in 0 ..< stepCount {
            let (r, t) = ICP.minimizePointToPlaneMetric(src: model, dst: sceneBox)
            let rot = r.eulerToRotation()
            
            let matrix = simd_double4x4(
                t: t,
                r: rot,
                s: .one)
            model = model.transform(matrix: matrix, ignoreInvalidNormal: true)
            let md = sceneBox.points.mean_distance(model.points)
            print("** [\(i)] meandistance = \(md)")
        }
        
        let (r0, t0) = ICP.minimizePointToPlaneMetric(src: modelBox, dst: sceneBox)
        print("** r0 = \(r0)")
        print("** t0 = \(t0)")
        let transform0 = simd_double4x4(
            t: t0,
            r: r0.eulerToRotation(),
            s: .one)
        let it1 = modelBox.transform(matrix: transform0)
        let md1 = it1.points.mean_distance(sceneBox.points)
        XCTAssertLessThan(md1, md0)

        let (r1, t1) = ICP.minimizePointToPlaneMetric(src: it1, dst: sceneBox)
        print("** md1 = \(md1)")
        print("** r1 = \(r1)")
        print("** t1 = \(t1)")
        
        let transform1 = simd_double4x4(
            t: t1,
            r: r1.eulerToRotation(),
            s: .one)
        let it2 = it1.transform(matrix: transform1)
        let md2 = it2.points.mean_distance(sceneBox.points)
        
        print("** md2 = \(md2)")
        print("** reuslt: \(it2.points)")
        
        XCTAssertLessThan(md2, md1)
    }
}
