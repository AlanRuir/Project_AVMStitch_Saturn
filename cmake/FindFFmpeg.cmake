set(FFmpeg_ROOT_DIR "/usr")

set(FFmpeg_INCLUDE_DIR "${FFmpeg_ROOT_DIR}/include")
set(FFmpeg_LIBRARY_DIR "${FFmpeg_ROOT_DIR}/lib/x86_64-linux-gnu")     # 若为X86架构，使用此代码
# set(FFmpeg_LIBRARY_DIR "${FFmpeg_ROOT_DIR}/lib/aarch64-linux-gnu")      # 若为ARM架构，使用此代码

set(FFmpeg_EXECUTABLE_DIR "${FFmpeg_ROOT_DIR}/bin")

set(FFmpeg_LIBRARIES
    "${FFmpeg_LIBRARY_DIR}/libavcodec.so"
    "${FFmpeg_LIBRARY_DIR}/libavformat.so"
    "${FFmpeg_LIBRARY_DIR}/libavfilter.so"
    "${FFmpeg_LIBRARY_DIR}/libavutil.so"
)

set(FFmpeg_COMPILE_FLAGS "-DENABLE_FEATURE_X")                             # 若为X86架构，使用此代码
# set(FFmpeg_COMPILE_FLAGS "-DENABLE_FEATURE_X -march=armv8-a+simd")           # 若为ARM架构，使用此代码

set(FFmpeg_FOUND TRUE)