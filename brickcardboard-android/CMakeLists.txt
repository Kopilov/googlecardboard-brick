cmake_minimum_required(VERSION 3.10.0)

# Standard Android dependencies
find_library(android-lib android)
find_library(EGL-lib EGL)
find_library(GLESv2-lib GLESv2)
find_library(log-lib log)

add_subdirectory(src/main/jni/assimp)

set(libs_dir ${CMAKE_CURRENT_SOURCE_DIR}/libraries)

# === Cardboard Sample ===
# Sources
file(GLOB native_srcs "src/main/jni/*.cpp")
# Output binary
add_library(cardboard_jni SHARED ${native_srcs})
# Includes
target_include_directories(cardboard_jni PRIVATE ${libs_dir} src/main/jni/assimp/include)
# Build
target_link_libraries(cardboard_jni
    ${android-lib}
    ${EGL-lib}
    ${GLESv2-lib}
    ${log-lib}
    assimp
    ${libs_dir}/jni/${ANDROID_ABI}/libcardboard_api.so)
