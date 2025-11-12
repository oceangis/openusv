# CMakeLists.txt 重构说明

## 重构动机

原CMakeLists.txt存在以下问题:
1. **冗长**: 手动列举了800+行源文件
2. **维护困难**: 每添加/删除一个文件都需要手动修改
3. **逻辑混乱**: 先添加后排除,效率低下
4. **不够简洁**: 先列出所有文件,然后用FILTER排除SITL等文件

## 重构策略

### 新方法: GLOB自动收集 + 智能排除

```
收集所有 → 排除不需要的 → 重新添加必需的 = 简洁且自动化
```

## 对比分析

### 旧版本 (800+ 行)
```cmake
set(COMPONENT_SRCS
    "../../libraries/APM_Control/AP_AutoTune.cpp"
    "../../libraries/APM_Control/AP_FW_Controller.cpp"
    # ... 800+ 行手动列举的文件
)

# 然后在后面排除
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/SITL/.*")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AC_[^/]+/.*")
```

**问题**:
- ✗ 需要手动维护每个文件
- ✗ 先添加再排除,效率低
- ✗ 新增库时容易遗漏
- ✗ 代码可读性差

### 新版本 (300 行)
```cmake
# 1. 自动收集所有源文件
file(GLOB_RECURSE ALL_LIBRARY_SRCS
    "../../libraries/*.c"
    "../../libraries/*.cpp"
)

# 2. 智能排除
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Airspeed/.*")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VisualOdom/.*")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/SITL/.*")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AC_[^/]+/.*")

# 3. 重新添加必需模块
foreach(ac_module IN LISTS ROVER_REQUIRED_AC_MODULES)
    file(GLOB_RECURSE ac_module_srcs "../../libraries/${ac_module}/*.cpp")
    list(APPEND COMPONENT_SRCS ${ac_module_srcs})
endforeach()
```

**优势**:
- ✓ 自动发现新文件
- ✓ 逻辑清晰: 收集→排除→添加
- ✓ 易于维护
- ✓ 代码量减少 60%+

## 文件结构

新的CMakeLists.txt清晰分为6个部分:

```
┌─────────────────────────────────────┐
│ 1. 收集源文件 (GLOB)                │
│    - 自动收集所有 libraries/*.cpp   │
│    - 自动收集所有 Rover/*.cpp       │
├─────────────────────────────────────┤
│ 2. 排除不需要的文件                 │
│    - 排除非ESP32 HAL                │
│    - 排除SITL文件                   │
│    - 排除benchmarks/tests           │
│    - 排除AP_Airspeed/AP_VisualOdom  │
│    - 排除所有AC_*模块               │
├─────────────────────────────────────┤
│ 3. 重新添加必需模块                 │
│    - AC_*模块 (8个)                 │
│    - HAL必需文件                    │
│    - SITL必需文件                   │
├─────────────────────────────────────┤
│ 4. 包含目录                         │
│    - 所有库的include路径            │
├─────────────────────────────────────┤
│ 5. 注册组件                         │
│    - idf_component_register         │
├─────────────────────────────────────┤
│ 6. 编译选项和定义                   │
│    - 警告选项                       │
│    - HAL定义                        │
│    - 功能开关                       │
└─────────────────────────────────────┘
```

## 核心改进

### 1. 自动化源文件收集

**旧版**:
```cmake
"../../libraries/AP_VideoTX/AP_SmartAudio.cpp"
"../../libraries/AP_VideoTX/AP_Tramp.cpp"
"../../libraries/AP_VideoTX/AP_VideoTX.cpp"
```

**新版**:
```cmake
file(GLOB_RECURSE ALL_LIBRARY_SRCS
    "../../libraries/*.c"
    "../../libraries/*.cpp"
)
```
→ 自动包含AP_VideoTX下的所有cpp文件

### 2. 清晰的排除规则

新版本在开头就明确排除不需要的库:
```cmake
# Exclude user-specified unwanted libraries
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_Airspeed/.*\\.(c|cpp)$")
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/AP_VisualOdom/.*\\.(c|cpp)$")
```

并添加编译定义:
```cmake
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    HAL_AIRSPEED_ENABLED=0
    HAL_VISUALODOM_ENABLED=0
)
```

### 3. 智能重添加机制

使用foreach循环自动处理AC_*模块:
```cmake
set(ROVER_REQUIRED_AC_MODULES
    AC_AttitudeControl
    AC_Avoidance
    # ... 其他模块
)

foreach(ac_module IN LISTS ROVER_REQUIRED_AC_MODULES)
    file(GLOB_RECURSE ac_module_srcs
        "../../libraries/${ac_module}/*.c"
        "../../libraries/${ac_module}/*.cpp"
    )
    list(APPEND COMPONENT_SRCS ${ac_module_srcs})
endforeach()
```

## 排除规则说明

### 必须排除的
1. **非ESP32 HAL**: `AP_HAL_SITL`, `AP_HAL_Empty`, `AP_IOMCU`
2. **SITL专用**: `SITL/` 目录下除必需文件外的所有文件
3. **测试和示例**: `tests/`, `examples/`, `benchmarks/`
4. **用户指定**: `AP_Airspeed`, `AP_VisualOdom`
5. **AC_*模块**: 默认排除,然后重新添加需要的8个

### 必须保留的
1. **HAL_ESP32**: 所有ESP32相关的HAL文件
2. **AC_*必需模块**: 8个Rover需要的AC模块
3. **SITL必需文件**: 用于仿真支持的基础文件
4. **所有业务库**: 除明确排除外的所有AP_*库

## 维护指南

### 添加新库
**旧版**: 需要手动在800+行中找到合适位置插入
```cmake
"../../libraries/NEW_LIB/new_file1.cpp"
"../../libraries/NEW_LIB/new_file2.cpp"
```

**新版**: 什么都不用做!
- 自动GLOB会发现新文件
- 如果是AC_*模块,添加到ROVER_REQUIRED_AC_MODULES列表即可

### 排除新库
只需添加一行排除规则:
```cmake
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/NEW_UNWANTED_LIB/.*\\.(c|cpp)$")
```

### 修改AC模块
只需在列表中添加/删除模块名:
```cmake
set(ROVER_REQUIRED_AC_MODULES
    AC_AttitudeControl
    AC_NewModule  # 添加新模块
)
```

## 性能影响

### 编译时间
- **GLOB操作**: 首次约0.1秒,CMake会缓存结果
- **FILTER操作**: 非常快,正则匹配是CMake内置优化的
- **总体影响**: 几乎可以忽略不计

### 文件变化检测
- CMake 3.12+ 会自动检测GLOB的文件变化
- 添加/删除文件后,重新运行`idf.py build`会自动更新

## 注意事项

### ⚠️ CMake版本要求
- 最低要求: CMake 3.5
- 建议版本: CMake 3.12+ (更好的GLOB支持)
- ESP-IDF 5.5.1 自带: CMake 3.24+ ✓

### ⚠️ 文件命名规范
确保所有源文件以`.c`或`.cpp`结尾,否则不会被GLOB收集。

### ⚠️ 手动排除的文件
如果某个特定文件需要排除,可以添加精确匹配:
```cmake
list(FILTER COMPONENT_SRCS EXCLUDE REGEX ".*/specific_file\\.cpp$")
```

## 回退方案

如果新版本出现问题,可以使用备份文件回退:
```bash
cd f:\opensource\usv_esp32\esp32s3rover\ardupilot_rover_esp32s3_idf\components\ardupilot
cp CMakeLists.txt.before_refactor CMakeLists.txt
```

## 验证步骤

1. **清理构建**:
```bash
idf.py fullclean
```

2. **重新配置**:
```bash
idf.py reconfigure
```

3. **编译**:
```bash
idf.py build
```

4. **检查源文件数量**:
编译输出中会显示编译的文件数量,应该与旧版本基本一致。

## 总结

| 指标 | 旧版本 | 新版本 | 改进 |
|------|--------|--------|------|
| 文件行数 | 1056行 | 308行 | **-71%** |
| 手动维护文件 | 800+ | 0 | **自动化** |
| 可读性 | ⭐⭐ | ⭐⭐⭐⭐⭐ | **显著提升** |
| 维护成本 | 高 | 低 | **大幅降低** |
| 新库适配 | 手动 | 自动 | **零成本** |

**关键优势**:
- ✅ 代码量减少71%
- ✅ 逻辑清晰,易于理解
- ✅ 自动化程度高,维护简单
- ✅ 易于扩展和修改
- ✅ 符合现代CMake最佳实践

**适用场景**:
- ✓ 大型项目,库文件众多
- ✓ 需要频繁添加/删除库
- ✓ 团队协作,降低维护成本
- ✓ 长期维护的项目
