import simd

public extension [simd_float3] {
    func mean() -> simd_double3 {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L63
        guard self.count > 0 else {
            return .zero
        }
        var acc = simd_double3(0, 0, 0)
        for x in self {
            acc += x.toDobule()
        }
        return acc / Double(self.count)
    }

    func totalLength() -> Double {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L96
        guard self.count > 0 else {
            return 0
        }
        var dist: Double = 0
        for x in self {
            dist += simd_length(x.toDobule())
        }
        return dist
    }
    
    func totalDistance(targets: [simd_float3]) -> Double { // l2 norm
        assert(self.count == targets.count)
        var n = 0.0
        for i in 0 ..< self.count {
            n += simd_distance(self[i].toDobule(), targets[i].toDobule())
        }
        return n
    }

    /// - Remark: only works for paired points
    func mean_distance(
        _ pairedPoints: [simd_float3]
    ) -> Double {
        assert(self.count == pairedPoints.count)
        return totalDistance(targets: pairedPoints) / Double(self.count)
    }
}

public extension simd_float3 {
    func toDobule() -> simd_double3 {
        return simd_double3(x: Double(self.x), y: Double(self.y), z: Double(self.z))
    }

    init(_ from: simd_double3) {
        self.init(x: Float(from.x), y: Float(from.y), z: Float(from.z))
    }
}

public extension simd_float4x4 {
    func toDouble4x4() -> simd_double4x4 {
        .init(
            simd_double4(columns.0),
            simd_double4(columns.1),
            simd_double4(columns.2),
            simd_double4(columns.3)
        )
    }
}
