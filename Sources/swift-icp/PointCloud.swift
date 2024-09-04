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
        assert(normals.count == 0 || points.count == normals.count)
        self.points = points
        self.normals = normals
    }

    public var points: [T]
    public var normals: [T]
}

public typealias PointCloud3f = PointCloud<simd_float3>

public extension PointCloud3f {
    func transform(
        matrix: simd_double4x4,
        ignoreInvalidNormal: Bool = false
    ) -> PointCloud3f {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/ppf_helpers.cpp#L568
        // transformPCPose
        let rot = matrix.rotation
        var ret: PointCloud3f = .init()
        for p in points {
            let p2d = matrix * simd_double4(simd_float4(p, 1))
            let p2 = simd_float3(simd_double3(p2d.x, p2d.y, p2d.z))
            ret.points.append(p2)
        }
        for p in normals {
            // 회전만 적용
            guard p != .zero, ignoreInvalidNormal else {
                ret.normals.append(p)
                continue
            }
            let p2 = rot * simd_double3(p)
            assert(p2.hasNan() == false)
            ret.normals.append(simd_normalize(simd_float3(p2)))
        }
        return ret
    }

    func transform(matrix: simd_float4x4) -> PointCloud3f {
        return transform(matrix: matrix.toDouble4x4())
    }
}
