import Foundation
import KDTree
import simd
import LASwift

public struct PointCloud<T> {
    var points: [T] = []
    var normals: [T] = [] // points 숫자와 같도록 강제 가능?
}

public typealias PointCloud3f = PointCloud<simd_float3>

public struct TransformResult {
    var transformMatrix: simd_double4x4 = matrix_identity_double4x4
    var residual: Double = 0
}

public class ICP {
    public enum Sampling {
        case uniform
        case gelfand
    }
    
    var tolerance: Double = 0.005
    var rejectionScale: Double = 2.5
    var maxIterations: Int = 250
    var numLevels: Int = 6
    var sampleType: Sampling = .uniform
    var numNeighborsCorr: Int = 1
    
    func registerModelToScene(
        model: PointCloud3f,
        scene: PointCloud3f
    ) -> TransformResult {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L71
        // 원본에서 mat rows 가 개수, cols 가 point(0,1,2), normal(3,4,5)
        let n = model.points.count
        let useRobustReject = rejectionScale > 0
        let meanModelPoint = model.points.mean()
        let meanScenePoint = scene.points.mean()
        let meanAvg = 0.5 * (meanModelPoint + meanScenePoint)
        var tmpModelPoints = model.points - meanAvg
        var tmpScenePoints = scene.points - meanAvg
        let distModel = tmpModelPoints.totalLength()
        let distScene = tmpScenePoints.totalLength()
        let scale = Double(n) / ((distModel + distScene) * 0.5)
        tmpModelPoints *= scale
        tmpScenePoints *= scale
        let tmpModelNormals = model.normals
//        var tmpSceneNormals = scene.normals
        
        var pose = matrix_identity_double4x4
//        var M = matrix_identity_double4x4
        var tempResidual: Double = 0
        var residual: Double = 0
        
        // walk the pyramid
        for level in (0 ..< numLevels).reversed() {
            let numSamples = divUp(n, 1 << level)
            let TolP = tolerance * Double((level + 1) * (level + 1))
            let MaxIterationsPyr = Int(round(Double(maxIterations / (level + 1))))
            
            // Obtain the sampled point clouds for this level : Also rotates the normals
            var modelPct = ICP.transformPCPose(
                pc: PointCloud3f(points: tmpModelPoints, normals: tmpModelNormals),
                pose: pose
            )
            let sampleStep = Int(round(Double(n) / Double(numSamples)))
            modelPct = ICP.samplePCUniform(
                pc: modelPct,
                sampleStep: sampleStep
            )
            
            /*
             Tolga Birdal thinks that downsampling the scene points might decrease the accuracy.
             Hamdi Sahloul, however, notied that accuracy increased (pose residual decreased slightly).
             */
            let scenePCS = ICP.samplePCUniform(
                pc: scene,
                sampleStep: sampleStep
            )
            
            // flann for knnSearch
            let tree = KDTree.Build(points: scenePCS.points)
            
            var fval_old: Double = 9999999999
            var fval_perc: Double = 0
            var fval_min: Double = 9999999999
            
            var src_moved = modelPct // copy
            
            var i = 0
            
            var numElSrc = modelPct.points.count
            let distances: [Float] = .init(repeating: 0, count: numElSrc)
            let indices: [Int] = .init(repeating: 0, count: numElSrc)
            
            // use robust weighting for outlier treatment
            var indicesModel: [Int] = .init(repeating: 0, count: numElSrc)
            var indicesScene: [Int] = .init(repeating: 0, count: numElSrc)
            
            var newI: [Int] = .init(repeating: 0, count: numElSrc)
            var newJ: [Int] = .init(repeating: 0, count: numElSrc)
            var poseX = matrix_identity_double4x4
            
            while !(fval_perc < (1 + TolP) && fval_perc > (1 - TolP))
                && i < MaxIterationsPyr {
                let results = ICP.query(from: tree, points: src_moved.points)
                newI = Array(0 ..< numElSrc)
                for r in results {
                    newJ.append(r.index)
                }
                
                if useRobustReject {
                    var numInliers = 0
                    let threshold = getRejectionThreshold(
                        r: distances,
                        m: distances.count,
                        outlierScale: Float(rejectionScale))
                    let acceptInd = distances.map { val in
                        return val < threshold
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
                
                // 중목제거하고 할당되지않은 가장 가까운 점 찾기..? 더 간단한 방법은 없나?
                //   hash key : data value,  hash value : data index
                //  최종 결과 indicesModel, indicesScene
                // 이상한 hashtabe .. getHashtable 할떄 key, data 를 +1 해서 hashtableInsertHashed
                //   그때문인지 매번 값 사용할 때, 검색(key)할 때 매번 -1 ..
                
                // 중복되는거면.. [value: [index]] 정도로 바꿔 사용하면..?
                var dict: [Int:[Int]] = [:]
                for (i, v) in newJ.enumerated() {
                    if dict[v] == nil {
                        dict[v] = []
                    }
                    dict[v]?.append(i)
                }
                var selInd = 0
                for (key, inds) in dict {
                    var minIdxD = 0
                    var minDist = Float.greatestFiniteMagnitude
                    for ind in inds {
                        let dist = distances[ind]
                        if dist < minDist {
                            minDist = dist
                            minIdxD = ind
                        }
                    }
                    indicesModel[selInd] = newI[minIdxD]
                    indicesScene[selInd] = key
                    selInd += 1
                }
                
                if selInd >= 6 {
                    // TODO: PointCloud3f 대신 Matrix 를 직접 사용?
                    var srcMatch = PointCloud3f() // Matrix(selInd, 6)
                    var dstMatch = PointCloud3f()
                    for i in 0 ..< selInd {
                        let indModel = indicesModel[i]
                        let indScene = indicesScene[i]
                        let srcPt = modelPct.points[indModel]
                        let srcN = modelPct.normals[indModel]
                        let dstPt = scenePCS.points[indScene]
                        let dstN = scenePCS.normals[indScene]

                        srcMatch.points.append(srcPt)
                        srcMatch.normals.append(srcN)
                        dstMatch.points.append(dstPt)
                        dstMatch.normals.append(dstN)
                    }
                    let (rpy, t) = ICP.minimizePointToPlaneMetric(src: srcMatch, dst: dstMatch)
                    // TODO: confirm same validation (cvIsNaN(cv::trace(rpy)) || cvIsNaN(cv::norm(t)))
                    if rpy.hasNan() || t.hasNan() {
                        break
                    }
                    poseX = ICP.getTransformMatrix(euler: rpy, t: t)
                    src_moved = ICP.transformPCPose(pc: modelPct, pose: poseX)
                    
                    let fval = ICP.norm_l2(srcMatch.points, dstMatch.points) / Double(srcMatch.points.count)
                    
                    // calculate change in error between iterations
                    fval_perc = fval / fval_old
                    
                    // store error value
                    fval_old = fval
                    
                    if fval < fval_min {
                        fval_min = fval
                    }
                }
                else {
                    break
                }
                i += 1
            }
            
            pose = poseX * pose
            residual = tempResidual
            tempResidual = fval_min
        }
        
        // sampling 등 preprocessing 했던 정보 복원
        var (r, t) = pose.poseToRT()
        t = t / scale + meanAvg - r * meanAvg
        
        // residual 에는 meanAvg 안곱해주나?
        
        return TransformResult(
            transformMatrix: simd_double4x4(t: t, r: r, s: .one),
            residual: residual
        )
    }
        
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
    
//    static func eulerToDCM(
//        euler: simd_double3
//    ) -> simd_double3x3 {
//        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/c_utils.hpp#L178
//        
//    }
    
    
    
    static func minimizePointToPlaneMetric(
        src: PointCloud3f,
        dst: PointCloud3f
    ) -> (simd_double3, simd_double3) { // (rpy, t)
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
            a[row: i] = axis.toArray() + normal.toArray()
            b[i, 0] = simd_dot(sub, normal)
        }
        // cv::solve(A, b, rpy_t, DECOMP_SVD);
        //   [rows, 6] * x([6, 1]) = [rows, 1]
        let (x, _) = lstsqr(a, b)
        let rpy = x[row: 0]
        assert(rpy.count == 6)
        return (simd_double3(rpy[0 ..< 3]), simd_double3(rpy[3 ..< 6]))
    }
    
    // https://developer.apple.com/documentation/accelerate/solving_systems_of_linear_equations_with_lapack
    //   -> macos 에서만 지원하는 듯
    
    
    func getRejectionThreshold(
        r:[Float],
        m: Int,
        outlierScale: Float
    ) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L174
        var t = r // copy
        let medR = medianF(arr: &t, n: m)
        
        for i in 0 ..< m {
            t[i] = fabsf(r[i] - medR)
        }
        let s = 1.48257968 * medianF(arr: &t, n: m) // 왜 다시 sort ?
        let threshold = outlierScale * s + medR
        return threshold
    }
    
    func medianF(arr: inout [Float], n: Int) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L111
        var low = 0
        var high = n - 1
        let median = (low + high) >> 1
        
        while(true) {
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
            let middle = (low + high) >> 1;
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
            arr.swapAt(middle, low+1)
            
            /* Nibble from each end towards middle, swapping items when stuck */
            var ll = low + 1
            var hh = high
            while(true) {
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
        var results: [KDTree<Element>.Result] = .init(unsafeUninitializedCapacity: points.count) { _, _ in }
        for p in points {
            if let result = from.nearestK(point: p).first {
                results.append(result)
            }
        }
        return results
    }
    
    static func transformPCPose(
        pc: PointCloud3f,
        pose: simd_double4x4
    ) -> PointCloud3f {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L568
        // ?? 왜 단순히 곱하기만 하지 않지????
        var pct = pc.points
        var pctN = pc.normals
        let (r, _) = pose.poseToRT()
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
        
        let numRows = pc.points.count / sampleStep
        var sampledPC = PointCloud3f()
        sampledPC.points = .init(unsafeUninitializedCapacity: numRows) { _, _ in }
        sampledPC.normals = .init(unsafeUninitializedCapacity: numRows) { _, _ in }

        for i in stride(from: 0, to: pc.points.count, by: sampleStep) {
            sampledPC.points.append(pc.points[i])
            sampledPC.normals.append(pc.normals[i])
        }
        
        assert(sampledPC.points.count == numRows)
        return sampledPC
    }
    
}

