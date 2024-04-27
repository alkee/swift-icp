// https://github.com/alkee/pointpoint/blob/main/Assets/Scripts/3rdParty/KDTree.cs
// https://github.com/Bersaelor/KDTree

import Foundation

public protocol KDTreePoint: Equatable {
    static var dimensions: Int { get }
    func kdDimension(_ dimension: Int) -> Double
    func squaredDistance(to otherPoint: Self) -> Double
}

public class KDTree<Element: KDTreePoint> {
    public struct Result {
        var index: Int = 0
        var squaredDistance: Double = 0
        var point: Element
    }
    
    let pivot: Element
    let pivotIndex: Int
    let axis: Int
    let left: KDTree<Element>?
    let right: KDTree<Element>?
    
    private init(
        pivot: Element,
        pivotIndex: Int,
        axis: Int,
        left: KDTree<Element>?,
        right: KDTree<Element>?
    ) {
        self.pivot = pivot
        self.pivotIndex = pivotIndex
        self.axis = axis
        self.left = left
        self.right = right
    }
    
    // 주의 : point 가 일치하는 점인경우 찾지 못함
    public func nearestK(point: Element, k: Int = 1) -> [Result] {
        assert(k>0)
        var bestSqDist: Double = Double.greatestFiniteMagnitude
        var minSqDist: Double = 0
        var bestIndex: Int = -1
        var bestIndexK: [Result] = .init(unsafeUninitializedCapacity: k) { _, _ in }
        var bsetPoint: Element = point
        
        for _ in 0 ..< k {
            searchK(point, &bestSqDist, &minSqDist, &bestIndex, &bsetPoint)
            if bestIndex < 0 {
                break
            }
            bestIndexK.append(
                Result(index: bestIndex, squaredDistance: bestSqDist, point: bsetPoint)
            )
            minSqDist = bestSqDist;
            bestSqDist = Double.greatestFiniteMagnitude
            bestIndex = -1;
        }
        return bestIndexK;
    }
    
    // MARK: internal helpers
    private func searchK(
        _ pt: Element,
        _ bestSqSoFar: inout Double,
        _ minSqDist: inout Double,
        _ bestIndex: inout Int,
        _ bestPt: inout Element
    ) {
        let sqd = pt.squaredDistance(to: pivot)
        if sqd < bestSqSoFar {
            if sqd > minSqDist {
                bestSqSoFar = sqd
                bestIndex = pivotIndex
                bestPt = pivot
            }
        }
        let planeDist = pt.kdDimension(axis) - pivot.kdDimension(axis)
        let node = planeDist > 0 ? right : left
        node?.searchK(pt, &bestSqSoFar, &minSqDist, &bestIndex, &bestPt)

        let planeDistSq = planeDist * planeDist
        if bestSqSoFar > planeDistSq {
            let node = (((planeDist > 0 ? 1 : 0) + 1) % 2 == 0)
                ? left
                : right
            node?.searchK(pt, &bestSqSoFar, &minSqDist, &bestIndex, &bestPt)
        }
    }

    public static func Build(
        points: [Element]
    ) -> KDTree<Element> {
        var indices = Array(0 ..< points.count)
        return MakeFromPointsInner(0, 0, points.count - 1, points, &indices)
    }

    private static func MakeFromPointsInner(
        _ depth: Int,
        _ stIndex: Int,
        _ enIndex: Int,
        _ points: [Element],
        _ inds: inout [Int]
    ) -> KDTree<Element> {
        let numDims = Element.dimensions
        let axis = depth % numDims
        let splitPoint = FindPivotIndex(
            points,
            &inds,
            stIndex,
            enIndex,
            axis
        )
        let pivotIndex = inds[splitPoint]
        let pivot = points[pivotIndex]

        let leftEndIndex = splitPoint - 1
        let leftNode = leftEndIndex < stIndex
            ? nil
            : MakeFromPointsInner(depth + 1, stIndex, leftEndIndex, points, &inds)
        let rightStartIndex = splitPoint + 1
        let rightNode = rightStartIndex > enIndex
            ? nil
            : MakeFromPointsInner(depth + 1, rightStartIndex, enIndex, points, &inds)

        return KDTree(
            pivot: pivot,
            pivotIndex: pivotIndex,
            axis: axis,
            left: leftNode,
            right: rightNode
        )
    }

    private static func FindPivotIndex(
        _ points: [Element],
        _ inds: inout [Int],
        _ stIndex: Int,
        _ enIndex: Int,
        _ axis: Int
    ) -> Int {
        let splitPoint = FindSplitPoint(
            points,
            inds,
            stIndex,
            enIndex,
            axis
        )
        let pivot = points[inds[splitPoint]]
        SwapElement(&inds, stIndex, splitPoint)

        var curPt = stIndex + 1
        var endPt = enIndex
        while curPt <= endPt {
            let curr = points[inds[curPt]]
            if curr.kdDimension(axis) > pivot.kdDimension(axis) {
                SwapElement(&inds, curPt, endPt)
                endPt -= 1
                continue
            }
            SwapElement(&inds, curPt - 1, curPt)
            curPt += 1
        }
        return curPt - 1
    }

    private static func FindSplitPoint(
        _ points: [Element],
        _ inds: [Int],
        _ stIndex: Int,
        _ enIndex: Int,
        _ axis: Int
    ) -> Int {
        let a = points[inds[stIndex]].kdDimension(axis)
        let b = points[inds[enIndex]].kdDimension(axis)
        let midIndex = (stIndex + enIndex) / 2
        let m = points[inds[midIndex]].kdDimension(axis)

        if a > b {
            if m > a {
                return stIndex
            }
            if b > m {
                return enIndex
            }
            return midIndex
        }
        if a > m {
            return stIndex
        }
        if m > b {
            return enIndex
        }
        return midIndex
    }

    private static func SwapElement(
        _ inds: inout [Int],
        _ a: Int,
        _ b: Int
    ) {
        let tmp = inds[a]
        inds[a] = inds[b]
        inds[b] = tmp
    }
}

