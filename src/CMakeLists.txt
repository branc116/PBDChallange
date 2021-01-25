cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

FIND_PACKAGE(Python3)

find_package(glm CONFIG REQUIRED)

set(PYTHON ${Python3_EXECUTABLE} CACHE STRING "Python path")
set(EMSCRIPTEN_ROOT $ENV{EMSDK}/upstream/emscripten CACHE STRING "Emscripten path")
set(ADITIONAL ${ADITIONAL_INCLUDE} CACHE STRING "ADITIONAL path")
set(CMAKE_TOOLCHAIN_FILE ${EMSCRIPTEN_ROOT}/cmake/Modules/Platform/Emscripten.cmake)
set(WEBIDL_BINDER_SCRIPT ${EMSCRIPTEN_ROOT}/tools/webidl_binder.py)
# set(PBD_FRONT_MATTER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/front-matter.js)
set(PBD_HEADER_FILE ${CMAKE_CURRENT_SOURCE_DIR}/hello.hpp)
set(PBD_IDL_FILE ${CMAKE_CURRENT_SOURCE_DIR}/hello.idl)
set(PBD_ONLOAD_FILE ${CMAKE_CURRENT_SOURCE_DIR}/onload.js)
set(BULLET_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bullet/src/)
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/Pose.cpp 
    ${CMAKE_CURRENT_SOURCE_DIR}/Body.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Help.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Joint.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Simulator.cpp)

# Disable bullet graphical benchmarks
set(USE_GRAPHICAL_BENCHMARK OFF)

# Disable bullet GLUT
set(USE_GLUT OFF)

# Build Release by default
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build Type")

# Default is 64*1024*1024 = 64MB
set(TOTAL_MEMORY 67108864 CACHE STRING "Total Memory")

# Enable for resizable heap, with some amount of slowness
set(ALLOW_MEMORY_GROWTH 0 CACHE STRING "Allow Memory Growth")

set(EMCC_ARGS
  --llvm-lto 1
  --post-js glue.js
  --post-js ${PBD_ONLOAD_FILE}
  -O3
  -s ALLOW_MEMORY_GROWTH=${ALLOW_MEMORY_GROWTH}
  -s ALLOW_TABLE_GROWTH=1
  -s EXPORTED_FUNCTIONS=["_malloc"]
  -s ERROR_ON_UNDEFINED_SYMBOLS=0
  -s EXPORTED_RUNTIME_METHODS=["UTF8ToString"]
  -s EXTRA_EXPORTED_RUNTIME_METHODS=["addFunction"]
  -s EXPORT_NAME="PBD"
  -s MODULARIZE=1
  -s NO_EXIT_RUNTIME=1
  -s NO_FILESYSTEM=1
  -s TOTAL_MEMORY=${TOTAL_MEMORY})

if(${CLOSURE})
  # Ignore closure errors about the bullet Node class
  # (Node is a DOM thing too)
  LIST(APPEND EMCC_ARGS
    --closure 1
    -s IGNORE_CLOSURE_COMPILER_ERRORS=1)
else()
  LIST(APPEND EMCC_ARGS
    -s NO_DYNAMIC_EXECUTION=1)
endif()

set(EMCC_JS_ARGS ${EMCC_ARGS}
  -I${ADITIONAL}
  -s AGGRESSIVE_VARIABLE_ELIMINATION=1
  -s ELIMINATE_DUPLICATE_FUNCTIONS=1
  -s LEGACY_VM_SUPPORT=1
  -s SINGLE_FILE=1
  -s WASM=0)

set(EMCC_WASM_ARGS ${EMCC_ARGS}
  -I${ADITIONAL}
  -s BINARYEN_IGNORE_IMPLICIT_TRAPS=1
  -s WASM=1)

set(EMCC_GLUE_ARGS
  -c
  -I${BULLET_SRC_DIR}
  -I${ADITIONAL}
  -include${PBD_HEADER_FILE})


#######################################
project("hello")

#######################################
add_custom_command(
  OUTPUT glue.cpp glue.js
  BYPRODUCTS parser.out WebIDLGrammar.pkl
  COMMAND ${PYTHON} ${WEBIDL_BINDER_SCRIPT} ${PBD_IDL_FILE} glue
  DEPENDS ${PBD_IDL_FILE}
  COMMENT "Generating PBD bindings"
  VERBATIM)
add_custom_command(
  OUTPUT glue.o
  COMMAND emcc glue.cpp ${EMCC_GLUE_ARGS} -o glue.o
  DEPENDS glue.cpp ${PBD_HEADER_FILE}
  COMMENT "Building PBD bindings"
  VERBATIM)
add_custom_target(PBD-bindings ALL DEPENDS glue.js glue.o)



#######################################
add_custom_command(
  OUTPUT PBD.wasm.js PBD.wasm.wasm
  COMMAND emcc glue.o ${SRC} ${EMCC_WASM_ARGS} -o PBD.wasm.js
  DEPENDS PBD-bindings ${PBD_FRONT_MATTER_FILE} ${PBD_ONLOAD_FILE} ${BULLET_TARGETS}
  COMMENT "Building PBD webassembly"
  VERBATIM)
add_custom_target(PBD-wasm ALL DEPENDS PBD.wasm.js PBD.wasm.wasm)