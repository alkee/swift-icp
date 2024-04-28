import Foundation
import simd

// only works for paried points
func mean_distance(_ points1: [simd_float3], _ points2: [simd_float3]) -> Double {
    assert( points1.count == points2.count )
    let cnt = Float(points1.count)
    var ret = 0.0
    for i in 0 ..< points1.count {
        ret += Double(simd_distance(points1[i], points2[i]) / cnt)
    }
    return ret
}
