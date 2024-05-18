import Foundation
import LASwift
import simd

public struct PointCloud<T> {
    public init() {
        self.init(points: [], normals: [])
    }

    public init(capacity: Int) {
        self.points = []
        points.reserveCapacity(capacity)
        self.normals = []
        normals.reserveCapacity(capacity)
    }

    public init(points: [T], normals: [T]) {
        assert(points.count == normals.count)
        self.points = points
        self.normals = normals
    }

    public var points: [T]
    public var normals: [T]
}

public typealias PointCloud3f = PointCloud<simd_float3>
func *(transform: simd_float4x4, self: PointCloud3f) -> PointCloud3f {
    var ret: PointCloud3f = .init()
    for p in self.points {
        let p2 = transform * simd_float4(p, 1)
        ret.points.append(simd_make_float3(p2))
    }
    for p in self.normals {
        let p2 = transform * simd_float4(p, 1)
        ret.normals.append(simd_make_float3(p2))
    }
    return ret
}

public struct TransformResult {
    var transformMatrix: simd_double4x4 = matrix_identity_double4x4
    var residual: Double = 0
}

public class ICP {
    public enum Sampling {
        case uniform
        case gelfand // not used.
    }
    
    var tolerance: Double = 0.005
    var rejectionScale: Double = 2.5
    var maxIterations: Int = 250
    var numLevels: Int = 6
    var sampleType: Sampling = .uniform
    var numNeighborsCorr: Int = 1
    
    func registerModelToScene(
        model: PointCloud3f, // floating
        scene: PointCloud3f // reference
    ) -> TransformResult {
        assert(model.normals.count > 0)
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L71
        // 원본에서 mat rows 가 개수, cols 가 point(0,1,2), normal(3,4,5)
        let n = model.points.count
        let useRobustReject = rejectionScale > 0
        let meanModelPoint = model.points.mean()
        let meanScenePoint = scene.points.mean()
        print("** mean  model: \(meanModelPoint) scene: \(meanScenePoint)")
        let meanAvg = 0.5 * (meanModelPoint + meanScenePoint)
        var tmpModelPoints = model.points - meanAvg
        var tmpScenePoints = scene.points - meanAvg
        let distModel = tmpModelPoints.totalLength()
        let distScene = tmpScenePoints.totalLength()
        let scale = Double(n) / ((distModel + distScene) * 0.5) // why ???
        tmpModelPoints *= scale
        tmpScenePoints *= scale
        print("** processing translation: \(meanAvg) scale : \(scale)")
        
        var pose = matrix_identity_double4x4
        var tempResidual: Double = 0
        var residual: Double = 0
        
        // walk the pyramid
        for level in (0 ..< numLevels).reversed() { // step -1
            let numSamples = divUp(n, 1 << level)
            let TolP = tolerance * Double((level + 1) * (level + 1))
            let MaxIterationsPyr = Int(round(Double(maxIterations / (level + 1))))
            
            // Obtain the sampled point clouds for this level : Also rotates the normals
            //  왜 sampling 을 먼저 하고 transform 하지 않지?
            var sampledModel = ICP.transformPCPose( // srcPCT
                pc: PointCloud3f(points: tmpModelPoints, normals: model.normals),
                pose: pose
            )
//            var sampledModel = pose.toFloat4x4()
//                * PointCloud3f(points: tmpModelPoints, normals: model.normals)
            let sampleStep = Int(round(Double(n) / Double(numSamples)))
            sampledModel = ICP.samplePCUniform(
                pc: sampledModel,
                sampleStep: sampleStep
            )
            
            /*
             Tolga Birdal thinks that downsampling the scene points might decrease the accuracy.
             Hamdi Sahloul, however, notied that accuracy increased (pose residual decreased slightly).
             */
            let sampledScene = ICP.samplePCUniform( // dstPCS
                pc: PointCloud3f(points: tmpScenePoints, normals: scene.normals),
                sampleStep: sampleStep
            )
            
            // void* flann = indexPCFlann(dstPCS); // distance 기반의 flann
            // 문제.. 가까운점 이 아니라 가까운 특징점을 찾아야 한다.!!
            let tree = KDTree.Build(points: sampledScene.points)
            
            var fval_old: Double = 9999999999
            var fval_perc: Double = 0
            var fval_min: Double = 9999999999
            
            var src_moved = sampledModel // copy
            
            var i = 0
            
            var numElSrc = sampledModel.points.count
            var distances: [Float] = .init(repeating: 0, count: numElSrc)
            var indices: [Int] = .init(repeating: 0, count: numElSrc)
            
            // use robust weighting for outlier treatment
            var indicesModel: [Int] = .init(repeating: 0, count: numElSrc)
            var indicesScene: [Int] = .init(repeating: 0, count: numElSrc)
            
            var newI: [Int] = .init(repeating: 0, count: numElSrc)
            var newJ: [Int] = .init(repeating: 0, count: numElSrc)
            var poseX = matrix_identity_double4x4
            
            while !(fval_perc < (1 + TolP) && fval_perc > (1 - TolP))
                && i < MaxIterationsPyr
            {
                let results = ICP.query(from: tree, points: src_moved.points)
                for (idx, r) in results.enumerated() {
                    newI[idx] = idx
                    newJ[idx] = r.index
                    indices[idx] = r.index
                    distances[idx] = sqrt(Float(r.squaredDistance))
                }
                
                if useRobustReject {
                    var numInliers = 0
                    let threshold = getRejectionThreshold(
                        r: distances,
                        outlierScale: Float(rejectionScale)
                    )
                    let acceptInd = distances.map { val in
                        val < threshold
                    }
                    for l in 0 ..< acceptInd.count {
                        if acceptInd[l] {
                            newI[numInliers] = l
                            newJ[numInliers] = indices[l]
                            numInliers += 1
                        }
                    }
                    numElSrc = numInliers
                }
                
                // Step 2: Picky ICP
                // Among the resulting corresponding pairs, if more than one scene point p_i
                // is assigned to the same model point m_j, then select p_i that corresponds
                // to the minimum distance
                
                // 중복제거하고 할당되지않은 가장 가까운 점 찾기..? 더 간단한 방법은 없나?
                //   hash key : data value,  hash value : data index
                //  최종 결과 indicesModel, indicesScene
                // 이상한 hashtabe .. getHashtable 할떄 key, data 를 +1 해서 hashtableInsertHashed
                //   그때문인지 매번 값 사용할 때, 검색(key)할 때 매번 -1 ..
                
                // 중복되는거면.. [value: [index]] 정도로 바꿔 사용하면..?
                
                
                var dict: [Int: [Int]] = [:]
                for i in 0 ..< numElSrc {
                    let v = newJ[i]
                    if dict[v] == nil {
                        dict[v] = []
                    }
                    dict[v]?.append(i)
                }
                var selInd = 0
                for (key, inds) in dict {
                    var minIdxD = 0
                    var minDist = Float.greatestFiniteMagnitude
                    for idx in inds { // 가장 가까운 거리의 index(minIdxD) 찾기
                        let dist = distances[idx]
                        if dist < minDist {
                            minDist = dist
                            minIdxD = idx
                        }
                    }
                    indicesModel[selInd] = newI[minIdxD]
                    indicesScene[selInd] = key
                    selInd += 1
                }
                
                if selInd > 6 { // = dict.count.. 중복 선택되지않은 6개 이상의 점?? 왜 6 ?
                    // least squares solution 사용하는데 제약조건.. 요소가 0이 포함되면 계산할 수 없음??
                    //  INFO parameter 참고 : https://www.netlib.org/clapack/CLAPACK-3.1.1/SRC/dgels.c
                    
                    // TODO: PointCloud3f 대신 Matrix 를 직접 사용?
                    var srcMatch = PointCloud3f(capacity: selInd) // Matrix(selInd, 6)
                    var dstMatch = PointCloud3f(capacity: selInd)
                    for i in 0 ..< selInd {
                        let indModel = indicesModel[i]
                        let indScene = indicesScene[i]
                        
                        let srcPt = sampledModel.points[indModel]
                        let dstPt = sampledScene.points[indScene]
                        srcMatch.points.append(srcPt)
                        dstMatch.points.append(dstPt)

                        let srcN = sampledModel.normals[indModel]
                        let dstN = sampledScene.normals[indScene]
                        srcMatch.normals.append(srcN)
                        dstMatch.normals.append(dstN)
                    }
//                    let (min_src, max_src) = minmax(srcMatch.points)
//                    let (min_dst, max_dst) = minmax(dstMatch.points)
//                    let (center_src, size_src) = bbox(min_src, max_src)
//                    let (center_dst, size_dst) = bbox(min_dst, max_dst)
//                    print("-- [\(level)-\(i)] src = \(center_src)/\(size_src), dst = \(center_dst)/\(size_dst)")

                    let (rpy, t) = ICP.minimizePointToPlaneMetric(src: srcMatch, dst: dstMatch)
                    print("-- [\(level)-\(i)] pose r=\(rpy), t=\(t)")
                    // 첫 minimizePointToPlaneMetric 이후 두번째부터 값이 커진다!!
                    //   src_moved 반영 이후에 srcMatch, dsstMatch 설정확인이 필요..???
                    
                    if rpy.hasNan() || t.hasNan() {
                        // TODO: confirm same validation (cvIsNaN(cv::trace(rpy)) || cvIsNaN(cv::norm(t)))
                        print("------ has NAN value in transform ------")
                        break
                    }

//                    poseX = simd_double4x4(
//                        t: t,
//                        r: simd_double3x3(rpy.eulerToQuaternion()),
//                        s: .one)
//                    src_moved = poseX.toFloat4x4() * sampledModel
                    poseX = ICP.getTransformMatrix(euler: rpy, t: t)

                    src_moved = ICP.transformPCPose(pc: sampledModel, pose: poseX)
//                    print("-- transformed distance : sampledModel=\(mean_distance(sampledModel.points, src_moved.points)), sampledScene=\(mean_distance(sampledScene.points, src_moved.points))")

                    let fval = ICP.norm_l2(srcMatch.points, dstMatch.points) / Double(src_moved.points.count)

                    // calculate change in error between iterations
                    fval_perc = fval / fval_old
                    
                    // store error value
                    fval_old = fval
                    
                    if fval < fval_min {
                        fval_min = fval
                    }
                    print("-- [\(level)-\(i)] faval_min = \(fval_min), fval=\(fval)")
                } else { // selInd <= 6 ; 가장가까운 점들이 한곳으로 몰리는 경우.. 너무 너무 거리가 먼 경우??
                    print("-- [\(level)-\(i)] ** selInd <= 6")
                    break
                }
                i += 1
            } // while iteration
            
            pose = poseX * pose
            // 왜.. residual 이.. pose 의 마지막 정보(fval)가 아니라 min value 이지?
            //      그리고 왜 fval_min 이 아니라 이전(tempResidual) 값이지? pose 는 이미 반영이 되었는데..
            residual = tempResidual //  > 0 ? tempResidual : fval_min
            tempResidual = fval_min
//            print("-- [\(level)], residual = \(residual), pose = \(pose), poseX = \(poseX)")
            print("-- [\(level)], residual = \(residual)")
        } // for level
        
        // scale, translation 등 preprocessing 했던 정보 복원
        var (r, t) = (pose.rotation, pose.translation)
        t = t / scale + meanAvg - r * meanAvg
        
        // residual 에는 scale 안나눠주나?
        residual = tempResidual
        
        return TransformResult(
            transformMatrix: simd_double4x4(t: t, r: r, s: .one),
            residual: residual
        )
    }
    
    /// 각 paried point 의 거리 합
    static func norm_l2(
        _ src1: [simd_float3],
        _ src2: [simd_float3]
    ) -> Double {
        assert(src1.count == src2.count)
        var n = 0.0
        for i in 0 ..< src1.count {
            n += simd_distance(src1[i].toDobule(), src2[i].toDobule())
        }
        return n
    }
    
    static func getTransformMatrix(
        euler: simd_double3,
        t: simd_double3
    ) -> simd_double4x4 {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L222
        // iOS 16 이상에서는 EulerAngles, Rotation3D 등 사용 가능한데.. https://developer.apple.com/documentation/spatial/rotation3d
        let q = euler.eulerToQuaternion()
        var r = simd_double4x4(q)
        r[3] = t.toDouble4(1)
        return r
    }
    
    // Kok Lim Low's linearization
    static func minimizePointToPlaneMetric(
        src: PointCloud3f,
        dst: PointCloud3f
    ) -> (simd_double3, simd_double3) { // (euler rotation, translation)
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L195
        assert(src.points.count == dst.points.count)
        assert(src.points.count == dst.normals.count)
        
        let a = Matrix(src.points.count, 6)
        let b = Matrix(src.points.count, 1)
        for i in 0 ..< src.points.count {
            let srcPt = src.points[i].toDobule()
            let dstPt = dst.points[i].toDobule()
            let normal = dst.normals[i].toDobule()
            let sub = dstPt - srcPt
            let axis = simd_cross(srcPt, normal)
            // LASwift 의 operator + 에 의해 [double] 이 concat 되지 않고 elementwise + 되어버림 주의
            var row = axis.toArray()
            row.append(contentsOf: normal.toArray())
            assert(row.count == 6)
            a[row: i] = row
            b[i, 0] = simd_dot(sub, normal)
        }
        // cv::solve(A, b, rpy_t, DECOMP_SVD);
        //   https://github.com/opencv/opencv/blob/12e2cc9502bc51bb01ed3fdd2f39ce1533c8236e/modules/core/src/lapack.cpp#L1032
        //   a(rows, 6) * x(6, 1) = b(rows, 1)
        // https://developer.apple.com/documentation/accelerate/solving_systems_of_linear_equations_with_lapack
        //   -> macos 에서만 지원하는 듯해 여기에서는 사용하기 어려움. => LASwift 사용
        
        assert(a.rows > 6) // full rank 문제(triangular factor zero) 피하기 위해. (lstsqr 함수 내 info == 6 "Error". 참고: https://www.netlib.org/clapack/CLAPACK-3.1.1/SRC/dgels.c
        let (x, _) = lstsqr(a, b) // solve equation(Ax=B)
        let rpy_t = x.T[row: 0] // rows to array

        assert(rpy_t.count == 6)
        let r_euler = simd_double3(rpy_t[0 ..< 3])
        let t = simd_double3(rpy_t[3 ..< 6])
        return (r_euler, t)
    }
    
    func getRejectionThreshold(
        r: [Float],
//        m: Int,
        outlierScale: Float
    ) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L174
        let m = r.count
        var t = r // copy
        let medR = medianF(arr: &t)
        
        for i in 0 ..< m {
            t[i] = fabsf(r[i] - medR)
        }
        let s = 1.48257968 * medianF(arr: &t) // 왜 다시 sort ?
        let threshold = outlierScale * s + medR
        return threshold
    }
    
    func medianF(
        arr: inout [Float]
//        , n: Int
    ) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L111
        let n = arr.count
        var low = 0
        var high = n - 1
        let median = (low + high) >> 1
        
        while true {
            if high <= low {
                return arr[median]
            }
            if high == low + 1 {
                /* two elements only */
                if arr[low] > arr[high] {
                    arr.swapAt(low, high)
                }
                return arr[median]
            }
            /* Find median of low, middle and high items; swap into position low */
            let middle = (low + high) >> 1
            if arr[middle] > arr[high] {
                arr.swapAt(middle, high)
            }
            if arr[low] > arr[high] {
                arr.swapAt(low, high)
            }
            if arr[middle] > arr[low] {
                arr.swapAt(middle, low)
            }
            /* low item is now in position middle */
            arr.swapAt(middle, low + 1)
            
            /* Nibble from each end towards middle, swapping items when stuck */
            var ll = low + 1
            var hh = high
            while true {
                repeat {
                    ll += 1
                } while arr[low] > arr[ll]
                repeat {
                    hh -= 1
                } while arr[hh] > arr[low]
                if hh < ll {
                    break
                }
                arr.swapAt(ll, hh)
            }
            
            /* Swap middle item (in position low) back into correct position */
            arr.swapAt(low, hh)
            
            /* Re-set active partition */
            if hh <= median {
                low = ll
            }
            if hh >= median {
                high = hh - 1
            }
        }
    }
    
    static func query<Element>(
        from: KDTree<Element>,
        points: [Element]
    ) -> [KDTree<Element>.Result] {
        let start = Date()

        var results: [KDTree<Element>.Result] = .init(
            repeating: .init(),
            count: points.count
        )

        for i in 0 ..< points.count {
        // 왜 concurrentPerform 가 더 느리지? (10_000 정도 테스트)
//        DispatchQueue.concurrentPerform(iterations: points.count) { i in
            if let result = from.nearestK(point: points[i]).first {
                results[i] = result
            }
        }
        results.removeAll(where: { $0.index == -1 })

        let end = Date()
        print("query(\(points.count) points estimated : \(end.timeIntervalSince(start))")
        
        return results
    }
    
    static func transformPCPose(
        pc: PointCloud3f,
        pose: simd_double4x4
    ) -> PointCloud3f {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L568
        // ?? 왜 단순히 matrix multiply 하지 않지???? scale, translation 은 ??
        var pct = pc.points
        var pctN = pc.normals
        let r = pose.rotation
        for i in 0 ..< pc.points.count {
            let data = pc.points[i]
            let n1 = pc.normals[i]
            
            let p = pose * data.toDobule().toDouble4(1)
            let p2 = p.toDouble3()
            
            // p2[3] should normally be 1
            if fabs(p.w) > .ulpOfOne { // EPS
                pct[i] = simd_float3(p2 * (1.0 / p.w))
            }
            
            // if the point cloud has normals,
            // then rotate them as well
            let n2 = r * n1.toDobule()
            let norm = simd_length(n2) // L2 norm
            if norm > .ulpOfOne {
                pctN[i] = simd_float3(n2 * (1.0 / norm))
            }
        }
        
        return PointCloud3f(points: pct, normals: pctN)
    }
    
    static func samplePCUniform(
        pc: PointCloud3f,
        sampleStep: Int
    ) -> PointCloud3f {
        // mhttps://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L251
        assert(pc.points.count == pc.normals.count)
        assert(sampleStep > 0)
        
        let reserveRows = (pc.points.count / sampleStep) + 1
        var sampledPC = PointCloud3f(capacity: reserveRows)

        for i in stride(from: 0, to: pc.points.count, by: sampleStep) {
            sampledPC.points.append(pc.points[i])
            sampledPC.normals.append(pc.normals[i])
        }
        return sampledPC
    }
}
