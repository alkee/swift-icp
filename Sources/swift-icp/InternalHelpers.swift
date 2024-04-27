import simd

func +(
    left: simd_double3,
    right: simd_float3
) -> simd_double3 {
    return simd_double3(
        x: left.x + Double(right.x),
        y: left.y + Double(right.y),
        z: left.z + Double(right.z)
    )
}

func +=(
    left: inout simd_double3,
    right: simd_float3
) {
    left = left + right
}

func -(
    left: simd_float3,
    right: simd_double3
) -> simd_float3 {
    return simd_float3(
        x: left.x + Float(right.x),
        y: left.y + Float(right.y),
        z: left.z + Float(right.z)
    )
}

func -(
    left: [simd_float3],
    right: simd_double3
) -> [simd_float3] {
    var points: [simd_float3] = .init(unsafeUninitializedCapacity: left.count) { _, _ in }
    for p in left {
        points.append(p - right)
    }
    return points
}

// func -=(
//    left: inout [simd_float3],
//    right: simd_double3
// ) {
//    let count = left.count
//    for i in 0 ..< count {
//        left[i] = left[i] - right
//    }
// }

func /(
    left: simd_double3,
    right: Int
) -> simd_double3 {
    return simd_double3(
        x: left.x / Double(right),
        y: left.y / Double(right),
        z: left.z / Double(right)
    )
}

func *=(
    left: inout simd_float3,
    right: Double
) {
    left.x *= Float(right)
    left.y *= Float(right)
    left.z *= Float(right)
}

func *=(
    left: inout [simd_float3],
    right: Double
) {
    for i in 0 ..< left.count {
        left[i] *= right
    }
}

func divUp(_ a: Int, _ b: Int) -> Int {
    return (a + b - 1) / b
}

extension [simd_float3] {
    func mean() -> simd_double3 {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L63
        guard self.count > 0 else {
            return simd_double3()
        }
        var acc = simd_double3()
        for x in self {
            acc += x
        }
        return acc / self.count
    }

    func totalLength() -> Double {
        // 주석에는 average 인데.. 실제 코드는 sum

        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/icp.cpp#L96
        guard self.count > 0 else {
            return 0
        }
        var dist: Double = 0
        for x in self {
            dist += Double(sqrt(x.x*x.x + x.y*x.y + x.z*x.z))
        }
        return dist
    }
}

extension simd_double4 {
    func toDouble3() -> simd_double3 {
        return simd_double3(x: self.x, y: self.y, z: self.z)
    }
}

extension simd_double3 {
    func toDouble4(_ w: Double = 1) -> simd_double4 {
        return simd_double4(x: self.x, y: self.y, z: self.z, w: w)
    }
    func toArray() -> [Double] {
        return [ x, y, z ]
    }
    func hasNan() -> Bool {
        return x.isNaN || y.isNaN || z.isNaN
    }
    func eulerToQuaternion() -> simd_quatd {
        // pitch : x axis, roll : y axis, yaw : z axis
        // https://math.stackexchange.com/a/2975462
        let qx = sin(y*0.5) * cos(x*0.5) * cos(z*0.5) - cos(y*0.5) * sin(x*0.5) * sin(z*0.5)
        let qy = cos(y*0.5) * sin(x*0.5) * cos(z*0.5) + sin(y*0.5) * cos(x*0.5) * sin(z*0.5)
        let qz = cos(y*0.5) * cos(x*0.5) * sin(z*0.5) - sin(y*0.5) * sin(x*0.5) * cos(z*0.5)
        let qw = cos(y*0.5) * cos(x*0.5) * cos(z*0.5) + sin(y*0.5) * sin(x*0.5) * sin(z*0.5)
        return simd_quatd(ix: qx, iy: qy, iz: qz, r: qw)
    }
}

extension simd_float3 {
    func toDobule() -> simd_double3 {
        return simd_double3(x: Double(self.x), y: Double(self.y), z: Double(self.z))
    }
    init(_ from: simd_double3) {
        self.init(x: Float(from.x), y: Float(from.y), z: Float(from.z))
    }
}

extension simd_double4x4 {
    func poseToR() -> simd_double3x3 {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/c_utils.hpp#L92
        let (col1, col2, col3, _) = self.columns

        return simd_double3x3(columns: (
            col1.toDouble3(),
            col2.toDouble3(),
            col3.toDouble3()
        ))
    }

    func poseToRT() -> (simd_double3x3, simd_double3) {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/c_utils.hpp#L97
        let r = self.poseToR()
        let (_, _, _, c) = self.columns
        let t = c.toDouble3()
        return (r, t)
    }
    
    init(t: simd_double3, r: simd_double3x3, s: simd_double3) {
        self.init(columns: (
            simd_double4(s.x * r[0,0], s.x * r[0,1], s.x * r[0,2], 0),
            simd_double4(s.y * r[1,0], s.y * r[1,1], s.y * r[1,2], 0),
            simd_double4(s.z * r[2,0], s.z * r[2,1], s.z * r[2,2], 0),
            simd_double4(t.x, t.y, t.z, 1)
        ))
    }
}

// MARK: KDTree

extension simd_float3: KDTreePoint {
    public static var dimensions: Int {
        return 3
    }

    public func kdDimension(_ dimension: Int) -> Double {
        return dimension == 0
            ? Double(self.x)
            : (dimension == 1)
            ? Double(self.y)
            : Double(self.z)
    }

    public func squaredDistance(to otherPoint: SIMD3<Scalar>) -> Double {
        return Double(simd_distance_squared(self, otherPoint))
    }
}


class HashNode_i<T: Numeric> {
    var key: UInt32 = 0
    var data: T = 0
    var next: HashNode_i? = nil
}

class HashTable_i {
    var size: Int = 0
    
}
