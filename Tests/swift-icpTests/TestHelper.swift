import Foundation
import ModelIO
import simd
import swift_icp

func load_pointcloud(obj_name: String) -> PointCloud3f {
    let url = Bundle.module.url(forResource: obj_name, withExtension: "obj")!
    let asset = MDLAsset(url: url)
    guard let mesh = asset.object(at: 0) as? MDLMesh else {
        fatalError("not found resource '\(obj_name).obj'")
    }
    print("** vertext count = \(mesh.vertexCount)")
    print("** vertext buffers = \(mesh.vertexBuffers.count)")
    let buffer = mesh.vertexBuffers.first!
    // floatbytes * (position dim + normal dim)
    let stride = 4 * (3 + 3)
    assert(buffer.length == mesh.vertexCount * stride)

    let rawPointer = buffer.map().bytes
    var points: [simd_float3] = []
    var normals: [simd_float3] = []
    for i in 0 ..< mesh.vertexCount {
        // "load from misaligned raw pointer" 오류때문에 simd_float3 을 직접 사용할 수 없음
        let px = rawPointer.load(fromByteOffset: i * stride, as: Float.self)
        let py = rawPointer.load(fromByteOffset: i * stride + 4, as: Float.self)
        let pz = rawPointer.load(fromByteOffset: i * stride + 8, as: Float.self)
        let nx = rawPointer.load(fromByteOffset: i * stride + 12, as: Float.self)
        let ny = rawPointer.load(fromByteOffset: i * stride + 16, as: Float.self)
        let nz = rawPointer.load(fromByteOffset: i * stride + 20, as: Float.self)
        points.append(simd_float3(px, py, pz))
        normals.append(simd_float3(nx, ny, nz))
    }
    return PointCloud3f(points: points, normals: normals)
}

// MARK: converting to cloud compare transform text

func ccText(by: simd_double4) -> String {
    return "\(by.x) \(by.y) \(by.z) \(by.w)"
}

func ccText(by: simd_double4x4, cmajor: Bool = true) -> String {
    var result = ""
    let by = cmajor ? by.transpose : by
    result += "\(ccText(by: by.columns.0))\n"
    result += "\(ccText(by: by.columns.1))\n"
    result += "\(ccText(by: by.columns.2))\n"
    result += "\(ccText(by: by.columns.3))"
    return result
}
