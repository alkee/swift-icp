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
    var points: [simd_float3] = []
    points.reserveCapacity(left.count)
    for p in left {
        points.append(p - right)
    }
    return points
}

func -(
    left: [simd_float3],
    right: simd_float3
) -> [simd_float3] {
    var points: [simd_float3] = []
    points.reserveCapacity(left.count)
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

// func *(
//    left: simd_double3,
//    right: Double
// ) -> simd_double3 {
//    return .init(
//        left.x * right,
//        left.y * right,
//        left.z * right
//    )
// }

//func /(
//    left: simd_double3,
//    right: Int
//) -> simd_double3 {
//    return simd_double3(
//        x: left.x / Double(right),
//        y: left.y / Double(right),
//        z: left.z / Double(right)
//    )
//}

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
            return .zero
        }
        var acc = simd_double3(0, 0, 0)
        for x in self {
            acc += x.toDobule()
        }
        return acc / Double(self.count)
    }

    func totalLength() -> Double {
        // 주석에는 average 인데.. 실제 코드는 sum

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
}

extension simd_double4 {
    func toDouble3() -> simd_double3 {
        return simd_double3(x: self.x, y: self.y, z: self.z)
    }

    func toFloat4() -> simd_float4 {
        return .init(Float(x), Float(y), Float(z), Float(w))
    }
}

extension simd_double3 {
    func toDouble4(_ w: Double = 1) -> simd_double4 {
        return simd_double4(x: self.x, y: self.y, z: self.z, w: w)
    }

    func toArray() -> [Double] {
        return [x, y, z]
    }

    func hasNan() -> Bool {
        return x.isNaN || y.isNaN || z.isNaN
    }
    
    func eulerToRotation() -> simd_double3x3 {
        let sx = sin(x)
        let cx = cos(x)
        var rx = simd_double3x3(1) // eye
        rx.columns.1.y = cx
        rx.columns.2.y = -sx
        rx.columns.1.z = sx
        rx.columns.2.z = cx
        
        let sy = sin(y)
        let cy = cos(y)
        var ry = simd_double3x3(1) // eye
        ry.columns.0.x = cy
        ry.columns.2.x = sy
        ry.columns.0.z = -sy
        ry.columns.2.z = cy
        
        let sz = sin(z)
        let cz = cos(z)
        var rz = simd_double3x3(1) // eye
        rz.columns.0.x = cz
        rz.columns.1.x = -sz
        rz.columns.0.y = sz
        rz.columns.1.y = cz
        
        return (rx*(ry*rz))
    }

//    func eulerToQuaternion() -> simd_quatd {
//        // 이거.. 근사값인지.. 반복되는 rotation 에서(minimizePointToPlaneMetric) 수렴이 안됨
//        fatalError()
//
//        
////        // pitch : x axis, roll : y axis, yaw : z axis
////        // https://math.stackexchange.com/a/2975462
////        let qx = sin(y*0.5)*cos(x*0.5)*cos(z*0.5) - cos(y*0.5)*sin(x*0.5)*sin(z*0.5)
////        let qy = cos(y*0.5)*sin(x*0.5)*cos(z*0.5) + sin(y*0.5)*cos(x*0.5)*sin(z*0.5)
////        let qz = cos(y*0.5)*cos(x*0.5)*sin(z*0.5) - sin(y*0.5)*sin(x*0.5)*cos(z*0.5)
////        let qw = cos(y*0.5)*cos(x*0.5)*cos(z*0.5) + sin(y*0.5)*sin(x*0.5)*sin(z*0.5)
////        return simd_quatd(ix: qx, iy: qy, iz: qz, r: qw)
//
////        // https://math.stackexchange.com/a/2975462
////        //  let (yaw, pitch, roll) = (x, y, z)
////        let qx = sin(z*0.5)*cos(y*0.5)*cos(x*0.5) - cos(z*0.5)*sin(y*0.5)*sin(x*0.5)
////        let qy = cos(z*0.5)*sin(y*0.5)*cos(x*0.5) + sin(z*0.5)*cos(y*0.5)*sin(x*0.5)
////        let qz = cos(z*0.5)*cos(y*0.5)*sin(x*0.5) - sin(z*0.5)*sin(y*0.5)*cos(x*0.5)
////        let qw = cos(z*0.5)*cos(y*0.5)*cos(x*0.5) + sin(z*0.5)*sin(y*0.5)*sin(x*0.5)
////        return simd_quatd(ix: qx, iy: qy, iz: qz, r: qw)
//    }
    
}

extension simd_float3 {
    func toDobule() -> simd_double3 {
        return simd_double3(x: Double(self.x), y: Double(self.y), z: Double(self.z))
    }

    init(_ from: simd_double3) {
        self.init(x: Float(from.x), y: Float(from.y), z: Float(from.z))
    }
}

extension simd_float4x4 {
    func toDouble4x4() -> simd_double4x4 {
        .init(
            simd_double4(columns.0),
            simd_double4(columns.1),
            simd_double4(columns.2),
            simd_double4(columns.3)
        )
    }
}

extension simd_double4x4 {
    func toFloat4x4() -> simd_float4x4 {
        .init(
            columns.0.toFloat4(),
            columns.1.toFloat4(),
            columns.2.toFloat4(),
            columns.3.toFloat4()
        )
    }

    var translation: simd_double3 {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/c_utils.hpp#L97
        return .init(columns.3.x, columns.3.y, columns.3.z)
    }

    var scale: simd_double3 {
        let a = columns.0.x
        let b = columns.1.x
        let c = columns.2.x
        let e = columns.0.y
        let f = columns.1.y
        let g = columns.2.y
        let i = columns.0.z
        let j = columns.1.z
        let k = columns.2.z
        let xScale = sqrt((a*a) + (e*e) + (i*i))
        let yScale = sqrt((b*b) + (f*f) + (j*j))
        let zScale = sqrt((c*c) + (g*g) + (k*k))
        return .init(xScale, yScale, zScale)
    }

    var rotation: simd_double3x3 {
        // https://github.com/opencv/opencv_contrib/blob/b042744ae4515c0a7dfa53bda2d3a22f2ec87a68/modules/surface_matching/src/c_utils.hpp#L92
        let (col1, col2, col3, _) = columns
        return simd_double3x3(columns: (
            col1.toDouble3(),
            col2.toDouble3(),
            col3.toDouble3()
        ))
    }

    init(t: simd_double3, r: simd_double3x3, s: simd_double3) {
        self.init(columns: (
            simd_double4(s.x*r[0, 0], s.x*r[0, 1], s.x*r[0, 2], 0),
            simd_double4(s.y*r[1, 0], s.y*r[1, 1], s.y*r[1, 2], 0),
            simd_double4(s.z*r[2, 0], s.z*r[2, 1], s.z*r[2, 2], 0),
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

// only works for paried points
func mean_distance(_ points1: [simd_float3], _ points2: [simd_float3]) -> Double {
    assert(points1.count == points2.count)
    let cnt = Float(points1.count)
    var ret = 0.0
    for i in 0 ..< points1.count {
        ret += Double(simd_distance(points1[i], points2[i]) / cnt)
    }
    return ret
}

func minmax(_ points: [simd_float3]) -> (simd_float3, simd_float3) {
    assert(points.count > 0)
    var min: simd_float3 = .one * .greatestFiniteMagnitude
    var max: simd_float3 = -min
    for p in points {
        if min.x > p.x { min.x = p.x }
        if min.y > p.y { min.y = p.y }
        if min.z > p.z { min.z = p.z }
        if max.x < p.x { max.x = p.x }
        if max.y < p.y { max.y = p.y }
        if max.z < p.z { max.z = p.z }
    }
    return (min, max)
}

// minmax to center, size
func bbox(
    _ min: simd_float3,
    _ max: simd_float3
) -> (simd_float3, simd_float3) {
    let center = (max + min) * 0.5
    let size = simd_float3(
        abs(max.x - min.x),
        abs(max.y - min.y),
        abs(max.z - min.z)
    )
    return (center, size)
}

func bbox(
    _ minmax: (min: simd_float3, max: simd_float3)
) -> (simd_float3, simd_float3) {
    return bbox(minmax.min, minmax.max)
}

func bbox(
    _ points: [simd_float3]
) -> (simd_float3, simd_float3) {
    return bbox(minmax(points))
}
