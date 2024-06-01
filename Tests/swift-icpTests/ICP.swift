import ModelIO
import SceneKit
import simd
@testable import swift_icp // @testable 은 internal access 가능하도록 해줌
import XCTest

final class ICP_Test: XCTestCase {
    var modelBox: PointCloud3f!
    var sceneBox: PointCloud3f!
    let sampleTransform = simd_double4x4(
        t: .init(5, -5, 0),
        //        r: matrix_identity_double3x3,
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
        print("** md2 = \(md2)") // should be same as md0
        
        print("-------------------")
    }
    
    
    func getUnitXRotation(angle: Double) -> simd_double3x3 {
        let sx = sin(angle)
        let cx = cos(angle)
        
        var rx = simd_double3x3(1) // eye
        rx.columns.1.y = cx
        rx.columns.2.y = -sx
        rx.columns.1.z = sx
        rx.columns.2.z = cx
        return rx
    }
    
    func getUnitYRotation(angle: Double) -> simd_double3x3 {
        let sy = sin(angle)
        let cy = cos(angle)
        
        var ry = simd_double3x3(1) // eye
        ry.columns.0.x = cy
        ry.columns.2.x = sy
        ry.columns.0.z = -sy
        ry.columns.2.z = cy
        return ry
    }

    func getUnitZRotation(angle: Double) -> simd_double3x3 {
        let sz = sin(angle)
        let cz = cos(angle)
        
        var rz = simd_double3x3(1) // eye
        rz.columns.0.x = cz
        rz.columns.1.x = -sz
        rz.columns.0.y = sz
        rz.columns.1.y = cz
        return rz
    }

    
    func eulerToDCM(euler: simd_double3) -> simd_double3x3 {
        let rx = getUnitXRotation(angle: euler.x)
        let ry = getUnitYRotation(angle: euler.y)
        let rz = getUnitZRotation(angle: euler.z)
        
        return (rx*(ry*rz))
    }
    
    
    func test_minimizePointToPlaneMetric() {
        
        var model = modelBox!
        let stepCount = 50

        let md0 = mean_distance(sceneBox.points, model.points)
        print("** initial meandistance = \(md0)")

        // 경향이.. 줄어들어야 하는데.. 원본(opencv c++) 에서도 비슷한 결과가 나오는지를 모르니..
        for i in 0 ..< stepCount {
            let (r, t) = ICP.minimizePointToPlaneMetric(src: model, dst: sceneBox)
            let rot = r.eulerToRotation()
            
            let matrix = simd_double4x4(
                t: t,
                r: rot,
                s: .one
            )
            model = model.transform(matrix: matrix, ignoreInvalidNormal: true)
            let md = mean_distance(sceneBox.points, model.points)
            print("** [\(i)] meandistance = \(md)")
//            print("** [\(i)] r = \(r), t = \(t)")
        }
        
        
//        let md0 = mean_distance(modelBox.points, sceneBox.points)
//        let (r0, t0) = ICP.minimizePointToPlaneMetric(src: modelBox, dst: sceneBox)
//        print("** md0 = \(md0)")
//        print("** r0 = \(r0)")
//        print("** t0 = \(t0)")
//        let transform0 = simd_double4x4(
//            t: t0,
//            r: simd_double3x3(r0.eulerToQuaternion()),
//            s: .one)
//        let it1 = modelBox.transform(matrix: transform0)
//        let md1 = mean_distance(it1.points, sceneBox.points)
//        XCTAssertLessThan(md1, md0)
//
//        let (r1, t1) = ICP.minimizePointToPlaneMetric(src: it1, dst: sceneBox)
//        print("** md1 = \(md1)")
//        print("** r1 = \(r1)")
//        print("** t1 = \(t1)")
//        
//        let transform1 = simd_double4x4(
//            t: t1,
//            r: simd_double3x3(r1.eulerToQuaternion()),
//            s: .one)
//        let it2 = it1.transform(matrix: transform1)
//        let md2 = mean_distance(it2.points, sceneBox.points)
//        
//        print("** md2 = \(md2)")
//        print("** reuslt: \(it2.points)")
//        
//        XCTAssertLessThan(md2, md1)
    }
    
//    func test_minimizePointToPlaneMetric2() {
//        let t = simd_double4x4(
//            t: .init(2, 3, 4),
//            r: simd_double3x3(simd_double3(0.1, 0.2, 0.3).eulerToQuaternion()),
//            s: .one)
//
//        let src = sceneBox!
//        let dst = t.toFloat4x4() * src
//        let md0 = mean_distance(src.points, dst.points)
//        print("** md0 = \(md0)")
//
//        let (r0, t0) = ICP.minimizePointToPlaneMetric(src: src, dst: dst)
//        var transform = simd_double4x4(
//            t: t0,
//            r: simd_double3x3(r0.eulerToQuaternion()),
//            s: .one)
//        var src_moved = transform.toFloat4x4() * src
//        let md1 = mean_distance(src_moved.points, dst.points)
//        print("** md1 = \(md1)")
//    }
    
    func test_norm_l2() {
        let p1: [simd_float3] = [
            .init(0, 0, 0),
            .init(1, 1, 1),
        ]
        let p2: [simd_float3] = [
            .init(0, 1, 0),
            .init(1, 0, 1),
        ]
        let l2n = ICP.norm_l2(p1, p2)
        
        let distances = [1.0, 1.0]
        let sum = distances.reduce(0, +)
        
        XCTAssertEqual(l2n, sum, accuracy: 0.001)
    }
    
    func test_getTransformMatrix() {
        let t1 = ICP.getTransformMatrix(euler: .zero, t: .one)
        XCTAssertEqual(t1.translation, .one)
        XCTAssertEqual(t1.rotation, matrix_identity_double3x3)
    }
    
    func test_registerModelToScene() {
        // 왜 translation 결과 가 기대값보다 엄청 큰거냐!!!!!!!
        //   iteration 중에 scene-model 중심 거리가 점점 커지기도...
        
        let t = simd_double4x4(
            t: .init(150, 0, -150),
            //        r: matrix_identity_double3x3,
            r: simd_double3(0.1, -0.15, 0.2).eulerToRotation(),
            s: .one)

        let pc = load_pointcloud(obj_name: "sample_skin") // floating
//        for n in pc.normals { // test normal
//            assert(n.x.isNaN == false)
//            assert(n.y.isNaN == false)
//            assert(n.z.isNaN == false)
//            assert(n.x != 0 && n.y != 0 && n.z != 0)
//        }
        
        let t_pc = pc.transform(matrix: t) // reference

        let md0 = mean_distance(t_pc.points, pc.points)

        let icp = ICP()
        let result = icp.registerModelToScene(model: pc, scene: t_pc)
        print("** residual = \(result.residual)")
        print("** result = \(ccText(by: result.transformMatrix))")
        print("** expected = \(ccText(by: t))")

        let transformed = ICP.transformPCPose(pc: pc, pose: result.transformMatrix) // pc.transform(matrix: result.transformMatrix)
        let md = mean_distance(t_pc.points, transformed.points)
        print("** md0 = \(md0), md = \(md)")
    }
}
