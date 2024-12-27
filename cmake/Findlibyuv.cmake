# 在项目的 cmake 模块目录中创建 FindLibyuv.cmake 文件

# 尝试找到 libyuv 库

# 清除先前查找的库和头文件变量
unset(LIBYUV_INCLUDE_DIR CACHE)
unset(LIBYUV_LIBRARY CACHE)

# 查找 libyuv 的头文件
find_path(LIBYUV_INCLUDE_DIR NAMES libyuv.h PATHS /usr/include /usr/local/include)

# 查找 libyuv 的共享库文件(.so) 和 静态库文件(.a)
# 根据您的系统架构和库的实际存在情况，可能需要调整库文件的搜索路径和名称
find_library(LIBYUV_LIBRARY 
             NAMES libyuv libyuv.so libyuv.so.1 libyuv.so.1.0.0 
             PATHS /usr/lib/aarch64-linux-gnu /usr/lib /usr/local/lib)

# 使用 find_package_handle_standard_args 确保找到库和头文件
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(libyuv DEFAULT_MSG LIBYUV_LIBRARY LIBYUV_INCLUDE_DIR)

# 将 LIBYUV_INCLUDE_DIR 和 LIBYUV_LIBRARY 变量输出给外部使用
mark_as_advanced(LIBYUV_INCLUDE_DIR LIBYUV_LIBRARY)