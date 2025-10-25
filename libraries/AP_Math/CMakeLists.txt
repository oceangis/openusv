# ArduPilot AP_Math 组件
# 自动生成 - 请勿手动编辑

idf_component_register(
    SRCS "AP_GeodesicGrid.cpp" "AP_Math.cpp" "chirp.cpp" "control.cpp" "crc.cpp" "location.cpp" "location_double.cpp" "matrix3.cpp" "matrixN.cpp" "matrix_alg.cpp"  # 仅列出前10个，实际编译时会包含所有
    INCLUDE_DIRS "." "benchmarks" "tools" "tools/geodesic_grid"  # 主要包含目录
    REQUIRES freertos esp_timer driver
)

# 完整源文件列表
set(COMPONENT_SRCS
    "AP_GeodesicGrid.cpp"
    "AP_Math.cpp"
    "chirp.cpp"
    "control.cpp"
    "crc.cpp"
    "location.cpp"
    "location_double.cpp"
    "matrix3.cpp"
    "matrixN.cpp"
    "matrix_alg.cpp"
    "polyfit.cpp"
    "polygon.cpp"
    "quaternion.cpp"
    "SCurve.cpp"
    "spline5.cpp"
    "SplineCurve.cpp"
    "vector2.cpp"
    "vector3.cpp"
    "AP_GeodesicGrid.cpp"
    "AP_Math.cpp"
    "chirp.cpp"
    "control.cpp"
    "crc.cpp"
    "location.cpp"
    "location_double.cpp"
    "matrix3.cpp"
    "matrixN.cpp"
    "matrix_alg.cpp"
    "polyfit.cpp"
    "polygon.cpp"
    "quaternion.cpp"
    "SCurve.cpp"
    "spline5.cpp"
    "SplineCurve.cpp"
    "vector2.cpp"
    "vector3.cpp"
    "benchmarks/benchmark_geodesic_grid.cpp"
    "benchmarks/benchmark_matrix.cpp"
)

# 完整包含目录列表
set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "benchmarks"
    "tools"
    "tools/geodesic_grid"
)
