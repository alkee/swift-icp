import simd
import Foundation
import SceneKit

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
    
    init(geometry: SCNGeometry, removeNaN: Bool = false) {
        self.init()
        guard let vertexSrc = geometry.sources(for: .vertex).first else {
            return
        }
        points = Self.toPoints(from: vertexSrc)
            
        guard let normalSrc = geometry.sources(for: .normal).first else {
            if removeNaN {
                points = points.filter { p in
                    !(p.x.isNaN || p.y.isNaN || p.z.isNaN)
                }
            }
            return
        }
        normals = Self.toPoints(from: normalSrc)
        assert(points.count == normals.count)
        if removeNaN {
            var ps: [simd_float3] = []
            var ns: [simd_float3] = []
            for i in 0..<points.count {
                let p = points[i]
                let n = normals[i]
                if p.x.isNaN || p.y.isNaN || p.z.isNaN ||
                    n.x.isNaN || n.y.isNaN || n.z.isNaN
                {
                    continue
                }
                ps.append(p)
                ns.append(n)
            }
            points = ps
            normals = ns
        }
    }
        
    func sample(uniformStepSize: Int) -> PointCloud3f {
        let pc = self
        assert(pc.normals.count == 0 || pc.points.count == pc.normals.count)
        assert(uniformStepSize > 0)
            
        let hasNormal = pc.normals.count > 0
        let reserveRows = (pc.points.count / uniformStepSize) + 1
        var sampledPC = PointCloud3f(capacity: reserveRows)
            
        for i in stride(from: 0, to: pc.points.count, by: uniformStepSize) {
            sampledPC.points.append(pc.points[i])
            if hasNormal {
                sampledPC.normals.append(pc.normals[i])
            }
        }
        return sampledPC
    }
    
    func saveAs(plyPath: URL) throws {
        var fileToWrite = ""
        let pointCount = points.count
        let hasNormal = normals.count == points.count
        
        var headers = ["ply", "format ascii 1.0",
                         "element vertex \(pointCount)", "property float x", "property float y", "property float z"]
        if hasNormal {
            headers += ["property float nx", "property float ny", "property float nz"]
        }
        headers += ["element face 0", "property list uchar int vertex_indices",
                    "end_header"]
                               
        for header in headers {
            fileToWrite += header
            fileToWrite += "\r\n"
        }
        
        for i in 0..<pointCount {
            let point = points[i]
            var pvValue = "\(point.x) \(point.y) \(point.z)"
            if hasNormal {
                let normal = normals[i]
                pvValue += " \(normal.x) \(normal.y) \(normal.z)"
            }
            fileToWrite += pvValue
            fileToWrite += "\r\n"
        }
        
        try fileToWrite.write(to: plyPath, atomically: true, encoding: .utf8)
    }
    
    private static func toPoints(from: SCNGeometrySource) -> [simd_float3] {
        assert(from.componentsPerVector * from.bytesPerComponent == MemoryLayout<SCNVector3>.size)
        
        let stride = from.dataStride //
        
        let offset = from.dataOffset
        let vectorCount = from.vectorCount
        return from.data.withUnsafeBytes { buffer in
            var result: [simd_float3] = []
            for i in 0..<vectorCount {
                let start = i * stride + offset
                // memory layout 크기 때문에(simd_float3 == 16, SCNVector3 == 12)
                let p = buffer.loadUnaligned(fromByteOffset: start, as: SCNVector3.self)
                result.append(simd_float3(p.x, p.y, p.z))
            }
            return result
        }
    }
}

// static operation
public extension PointCloud3f {
    static func *(left: PointCloud3f, right: Float) -> PointCloud3f {
        return PointCloud3f(
            points: left.points.map { p in
                p * right
            },
            normals: left.normals.map { n in
                n * right
            }
        )
    }

    static func *(left: PointCloud3f, right: simd_float4x4) -> PointCloud3f {
        return left.transform(matrix: right)
    }

    static func *(left: PointCloud3f, right: simd_double4x4) -> PointCloud3f {
        return left.transform(matrix: right)
    }
}
