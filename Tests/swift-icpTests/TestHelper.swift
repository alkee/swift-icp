import Foundation
import ModelIO
import simd
import swift_icp

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
