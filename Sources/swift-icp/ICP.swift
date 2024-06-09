import Foundation
import LASwift
import simd

public struct TransformResult {
    public var transformMatrix: simd_double4x4 = matrix_identity_double4x4
    public var residual: Double = 0
}

public class ICP {
    public enum Sampling {
        case uniform
        case gelfand // not used.
    }
    
    public init(
        tolerance: Double = 0.005,
        rejectionScale: Double = 2.5,
        maxIterations: Int = 250,
        numLevels: Int = 6,
        sampleType: Sampling = .uniform,
        numNeighborsCorr: Int = 1
    ) {
        self.tolerance = tolerance
        self.rejectionScale = rejectionScale
        self.maxIterations = maxIterations
        self.numLevels = numLevels
        self.sampleType = sampleType
        self.numNeighborsCorr = numNeighborsCorr
    }
    
    let tolerance: Double
    let rejectionScale: Double
    let maxIterations: Int
    let numLevels: Int
    let sampleType: Sampling
    let numNeighborsCorr: Int
    
    public func registerModelToScene(
        model: PointCloud3f, // floating - source
        scene: PointCloud3f // reference - target
    ) -> TransformResult {
        assert(model.normals.count > 0)
        // TODO: normals in point cloud validation
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L246
        // Mat(0,1,2:point, 3,4,5:normal)
        let n = model.points.count
        let useRobustReject = rejectionScale > 0
        
        // preprocessing
        let meanSrc = model.points.mean()
        let meanDst = scene.points.mean()
        
        let meanAvg = (meanSrc + meanDst) * 0.5
        var srcTemp = model.points - meanAvg
        var dstTemp = scene.points - meanAvg
        
        let distSrc = srcTemp.totalLength()
        let distDst = dstTemp.totalLength()
        let scale = Double(n) / ((distSrc + distDst) * 0.5)
        srcTemp *= scale
        dstTemp *= scale
        
        // return values
        var pose = matrix_identity_double4x4
        var tempResidual: Double = 0
        var residual: Double = 0
        
        // walk the pyramid
        for level in (0 ..< numLevels).reversed() { // step -1
            let numSamples = Self.divUp(n, 1 << level)
            let TolP = tolerance * Double((level + 1) * (level + 1))
            let MaxIterationsPyr = Int(round(Double(maxIterations / (level + 1))))
            
            // Obtain the sampled point clouds for this level : Also rotates the normals
            // TODO: sampling before transform(less calculation)
            var sampledSrc = PointCloud3f( // srcPCT
                points: srcTemp,
                normals: model.normals
            ).transform(matrix: pose)
            let sampleStep = Int(round(Double(n) / Double(numSamples)))
            sampledSrc = ICP.samplePCUniform(
                pc: sampledSrc,
                sampleStep: sampleStep
            )
            
            /*
             Tolga Birdal thinks that downsampling the scene points might decrease the accuracy.
             Hamdi Sahloul, however, notied that accuracy increased (pose residual decreased slightly).
             */
            let sampledDst = ICP.samplePCUniform( // dstPCS
                pc: PointCloud3f(points: dstTemp, normals: scene.normals),
                sampleStep: sampleStep
            )
            
            let flann = FLANN(points: sampledDst.points)
            var fval_old: Double = 9999999999
            var fval_perc: Double = 0
            var fval_min: Double = 9999999999
            
            var src_moved = sampledSrc // copy
            var i = 0
            
            var numElSrc = src_moved.points.count
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
                let results = ICP.query(from: flann, points: src_moved.points)
                for (idx, r) in results.enumerated() {
                    newI[idx] = idx
                    newJ[idx] = r.index
                    indices[idx] = r.index
                    distances[idx] = r.distance
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
                
                // find unpaired(no duplication) nearest point
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
                
                if selInd > 6 { // least squares solution requirements. see INFO parameter : https://www.netlib.org/clapack/CLAPACK-3.1.1/SRC/dgels.c
                    var srcMatch = PointCloud3f(capacity: selInd) // Matrix(selInd, 6)
                    var dstMatch = PointCloud3f(capacity: selInd)
                    for i in 0 ..< selInd {
                        let indModel = indicesModel[i]
                        let indScene = indicesScene[i]
                        
                        let srcPt = sampledSrc.points[indModel]
                        let dstPt = sampledDst.points[indScene]
                        srcMatch.points.append(srcPt)
                        dstMatch.points.append(dstPt)
                        
                        let srcN = sampledSrc.normals[indModel]
                        let dstN = sampledDst.normals[indScene]
                        srcMatch.normals.append(srcN) // 사용하지는 않음
                        dstMatch.normals.append(dstN)
                    }
                    
                    let (rpy, t) = ICP.minimizePointToPlaneMetric(src: srcMatch, dst: dstMatch)
                    
                    if rpy.hasNan() || t.hasNan() { // (cvIsNaN(cv::trace(rpy)) || cvIsNaN(cv::norm(t)))
                        // invalid normal contains.
                        break
                    }
                    
                    poseX = simd_double4x4( // getTransformMatrix
                        t: t,
                        r: rpy.eulerToRotation(),
                        s: .one
                    )
                    src_moved = sampledSrc.transform(matrix: poseX)
                    
                    let fval = srcMatch.points.mean_distance(dstMatch.points)
                    // calculate change in error between iterations
                    fval_perc = fval / fval_old
                    // store error value
                    fval_old = fval
                    if fval < fval_min {
                        fval_min = fval
                    }
                    
                } else { // selInd <= 6 ; is it too far ?
                    break
                }
                i += 1
            } // while iteration
            
            pose = poseX * pose
            residual = tempResidual
            tempResidual = fval_min
        } // for level(pyramid)
        
        // TODO: recalculate real residual(inverse preprocess)
        residual = tempResidual
        
        // restore preprocessing. (scale, meanAvg)
        let (r, t) = (pose.rotation, pose.translation) // poseToRT
        // TODO: check difference to original source
        //  https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L461
        let restored_t = t / scale - meanAvg + r * meanAvg
        return TransformResult(
            transformMatrix: simd_double4x4(t: restored_t, r: r, s: .one),
            residual: residual
        )
    }
    
    // Kok Lim Low's linearization
    static func minimizePointToPlaneMetric(
        src: PointCloud3f,
        dst: PointCloud3f
    ) -> (simd_double3, simd_double3) { // (euler rotation, translation)
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L195
        assert(src.points.count == dst.points.count)
        assert(dst.points.count == dst.normals.count)
        
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
        //   A(rows, 6) * x(6, 1) = B(rows, 1)
        // LASwift for esasy to use. instead of https://developer.apple.com/documentation/accelerate/solving_systems_of_linear_equations_with_lapack
        assert(a.rows > 6) // to avoid full rank problem(triangular factor zero). see the INFO in  https://www.netlib.org/clapack/CLAPACK-3.1.1/SRC/dgels.c
        let (x, _) = lstsqr(a, b) // solve equation(Ax=B)
        let rpy_t = x.T[row: 0] // rows to array
        
        assert(rpy_t.count == 6)
        let r_euler = simd_double3(rpy_t[0 ..< 3])
        let t = simd_double3(rpy_t[3 ..< 6])
        return (r_euler, t)
    }
    
    func getRejectionThreshold(
        r: [Float],
        outlierScale: Float
    ) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L174
        let m = r.count
        var t = r // copy
        let medR = medianF(arr: &t)
        
        for i in 0 ..< m {
            t[i] = fabsf(r[i] - medR)
        }
        let s = 1.48257968 * medianF(arr: &t) // sort again? why?
        let threshold = outlierScale * s + medR
        return threshold
    }
    
    // From numerical receipes: Finds the median of an array
    func medianF(
        arr: inout [Float]
    ) -> Float {
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L111
        let n = arr.count
        var low = 0
        var high = n - 1
        let median = (low + high) >> 1
        
        while true {
            if high <= low { /* One element only */
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
    
    static func query(
        from: FLANN,
        points: [simd_float3]
    ) -> [FLANN.Result] {
        //        let start = Date()
        var results: [FLANN.Result] = .init()
        results.reserveCapacity(points.count)
        
        for i in 0 ..< points.count {
            let r = from.query(point: points[i])
            if r.count == 0 { continue }
            results.append(r[0])
        }
        return results
    }
    
    /// - remark: ICP would fail(Nan) by invalid normals like (0,0,0)
    static func samplePCUniform(
        pc: PointCloud3f,
        sampleStep: Int,
        includeInvalidNormal: Bool = false // TODO: recalculation support.
    ) -> PointCloud3f {
        // mhttps://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L251
        assert(pc.points.count == pc.normals.count)
        assert(sampleStep > 0)
        
        let reserveRows = (pc.points.count / sampleStep) + 1
        var sampledPC = PointCloud3f(capacity: reserveRows)
        
        for i in stride(from: 0, to: pc.points.count, by: sampleStep) {
            if includeInvalidNormal || simd_length(pc.normals[i]) > 0 {
                sampledPC.points.append(pc.points[i])
                sampledPC.normals.append(pc.normals[i])
            } else {
                // TODO: log warning
            }
        }
        return sampledPC
    }
    
    static func divUp(_ a: Int, _ b: Int) -> Int {
        return (a + b - 1) / b
    }
}
