import Foundation
import simd
import SwiftAnnoy

class FLANN {
    struct Result {
        let index: Int
        let distance: Float
    }

    // https://github.com/jbadger3/SwiftAnnoy
    let index: AnnoyIndex<Float> = .init(itemLength: 3)

    init(
        points: [simd_float3]
    ) {
        for (i, p) in points.enumerated() {
            var point = [p.x, p.y, p.z]
            try! index.addItem(index: i, vector: &point)
        }
        try! index.build(numTrees: 1)
    }

    func query(
        point: simd_float3,
        neighbors: Int = 1
    ) -> [Result] {
        var p = [point.x, point.y, point.z]
        guard let fr = index.getNNsForVector(
            vector: &p,
            neighbors: neighbors
        ) else {
            return []
        }
        let cnt = fr.indices.count
        var result: [Result] = []
        result.reserveCapacity(cnt)
        for i in 0 ..< cnt {
            result.append(
                Result(
                    index: fr.indices[i],
                    distance: fr.distances[i]
                )
            )
        }
        return result
    }

    /// - remark: 결과안에 자신의 index(distance 0)가 포함될 수 있음 주의
    func query(
        index: Int,
        neighbors: Int = 1
    ) -> [Result] {
        guard let fr = self.index.getNNsForItem(
            item: index,
            neighbors: neighbors
        ) else {
            return []
        }
        let cnt = fr.indices.count
        var result: [Result] = []
        result.reserveCapacity(cnt)
        for i in 0 ..< cnt {
            result.append(
                Result(
                    index: fr.indices[i],
                    distance: fr.distances[i]
                )
            )
        }
        return result
    }
}
