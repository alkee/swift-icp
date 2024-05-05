// swift-tools-version: 5.10
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "swift-icp",
    products: [
        // Products define the executables and libraries a package produces, making them visible to other packages.
        .library(
            name: "swift-icp",
            targets: ["swift-icp"]
        ),
    ],
    dependencies: [
        .package(url: "https://github.com/AlexanderTar/LASwift", exact: "0.3.2")
    ],
    targets: [
        // Targets are the basic building blocks of a package, defining a module or a test suite.
        // Targets can depend on other targets in this package and products from dependencies.
        .target(
            name: "swift-icp",
            dependencies: ["LASwift"]
        ),
        .testTarget(
            name: "swift-icpTests",
            dependencies: ["swift-icp"],
            resources: [
                .process("Resources"),
            ]
        ),
    ]
)
