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
