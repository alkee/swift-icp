import simd

public extension simd_double4 {
    func toDouble3() -> simd_double3 {
        return simd_double3(x: self.x, y: self.y, z: self.z)
    }

    func toFloat4() -> simd_float4 {
        return .init(Float(x), Float(y), Float(z), Float(w))
    }
}

public extension simd_double3 {
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
}

public extension simd_double4x4 {
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
        // https://github.com/opencv/opencv_contrib/blob/ac994ed2b5b6dd37d60ae5cd4267b61ceefa052d/modules/surface_matching/src/icp.cpp#L222
        self.init(columns: (
            simd_double4(s.x*r[0, 0], s.x*r[0, 1], s.x*r[0, 2], 0),
            simd_double4(s.y*r[1, 0], s.y*r[1, 1], s.y*r[1, 2], 0),
            simd_double4(s.z*r[2, 0], s.z*r[2, 1], s.z*r[2, 2], 0),
            simd_double4(t.x, t.y, t.z, 1)
        ))
    }
}

