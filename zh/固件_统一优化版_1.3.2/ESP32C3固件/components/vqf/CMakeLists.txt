idf_component_register(SRCS "vqf.cpp" "basicvqf.cpp" "vqf_wrapper.cpp"
                    INCLUDE_DIRS "include")

# ============================================================================
# VQF 特性开关 (注释/取消注释以启用/禁用)
# ============================================================================
# 使用单精度浮点 (ESP32-C3 仅有单精度 FPU, 关闭后使用软件 double)
target_compile_definitions(${COMPONENT_LIB} PUBLIC VQF_SINGLE_PRECISION)

# 默认启用运动偏置估计；如实测VQF耗时或丢帧明显增加，可用
# `idf.py -DVQF_ENABLE_MOTION_BIAS_ESTIMATION=OFF build` 恢复原行为。
option(VQF_ENABLE_MOTION_BIAS_ESTIMATION
       "Enable VQF motion bias estimation" ON)
if(NOT VQF_ENABLE_MOTION_BIAS_ESTIMATION)
    target_compile_definitions(
        ${COMPONENT_LIB} PUBLIC VQF_NO_MOTION_BIAS_ESTIMATION)
endif()
