cmake_minimum_required(VERSION 3.16)

project(a2_deploy VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)
include(CTest)

find_package(Torch REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED COMPONENTS Crypto)

if(TARGET yaml-cpp::yaml-cpp)
    set(A2_YAML_CPP_TARGET yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
    set(A2_YAML_CPP_TARGET yaml-cpp)
else()
    message(FATAL_ERROR "yaml-cpp was found, but it did not provide a CMake target")
endif()

# Prefer a source checkout next to this repository. Set this cache entry to an
# empty string to require a system-installed unitree_sdk2 package instead.
set(
    UNITREE_SDK2_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2"
    CACHE PATH
    "Path to a unitree_sdk2 source checkout (empty: use find_package)"
)

if(UNITREE_SDK2_ROOT AND EXISTS "${UNITREE_SDK2_ROOT}/CMakeLists.txt")
    # unitree_sdk2 uses BUILD_EXAMPLES rather than BUILD_TESTING.
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build unitree_sdk2 examples" FORCE)
    add_subdirectory(
        "${UNITREE_SDK2_ROOT}"
        "${CMAKE_CURRENT_BINARY_DIR}/_deps/unitree_sdk2"
        EXCLUDE_FROM_ALL
    )
elseif(UNITREE_SDK2_ROOT AND
       NOT UNITREE_SDK2_ROOT STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}/../unitree_sdk2")
    message(FATAL_ERROR
        "UNITREE_SDK2_ROOT does not contain CMakeLists.txt: ${UNITREE_SDK2_ROOT}")
else()
    find_package(unitree_sdk2 CONFIG REQUIRED)
endif()

if(NOT TARGET unitree_sdk2)
    message(FATAL_ERROR "unitree_sdk2 did not provide the unitree_sdk2 target")
endif()

add_library(a2_core STATIC
    src/config.cpp
    src/high_command_receiver.cpp
    src/navigation.cpp
    src/safety.cpp
    src/policy.cpp
)

target_include_directories(a2_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
        ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(a2_core
    PUBLIC
        ${A2_YAML_CPP_TARGET}
        OpenSSL::Crypto
        ${TORCH_LIBRARIES}
)

# Torch uses TORCH_CXX_FLAGS to communicate ABI options on some distributions.
if(TORCH_CXX_FLAGS)
    separate_arguments(A2_TORCH_CXX_FLAGS NATIVE_COMMAND "${TORCH_CXX_FLAGS}")
    target_compile_options(a2_core PUBLIC ${A2_TORCH_CXX_FLAGS})
endif()

add_library(a2_runtime STATIC
    src/controller.cpp
)

target_include_directories(a2_runtime
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(a2_runtime
    PUBLIC
        a2_core
        unitree_sdk2
)

add_executable(a2_deploy src/main.cpp)
target_link_libraries(a2_deploy PRIVATE a2_runtime)
set_target_properties(a2_deploy PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# Keep the project-root-relative layout so paths such as models/... in the YAML
# configuration work identically from the source and build trees.
set(A2_BUILD_SHARE_DIR "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}")

function(a2_stage_resource relative_path)
    set(source_path "${CMAKE_CURRENT_SOURCE_DIR}/${relative_path}")
    set(destination_path "${A2_BUILD_SHARE_DIR}/${relative_path}")

    if(EXISTS "${source_path}")
        get_filename_component(destination_dir "${destination_path}" DIRECTORY)
        file(MAKE_DIRECTORY "${destination_dir}")
        configure_file("${source_path}" "${destination_path}" COPYONLY)
    else()
        message(WARNING "A2 resource is not present: ${source_path}")
    endif()
endfunction()

a2_stage_resource(params/a2.yaml)
a2_stage_resource(models/a2_45d_policy.jit)
a2_stage_resource(sim/a2_mujoco.py)
a2_stage_resource(sim/a2_highcmd.py)
a2_stage_resource(sim/a2_prepare_low_policy.py)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/a2")
    file(MAKE_DIRECTORY "${A2_BUILD_SHARE_DIR}/assets")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/assets/a2"
         DESTINATION "${A2_BUILD_SHARE_DIR}/assets")
else()
    message(WARNING "A2 simulation assets are not present")
endif()

if(BUILD_TESTING)
    foreach(test_name IN ITEMS test_contract test_safety test_policy test_command test_navigation test_high_command_receiver)
        set(test_source "${CMAKE_CURRENT_SOURCE_DIR}/tests/${test_name}.cpp")
        if(EXISTS "${test_source}")
            add_executable(${test_name} "${test_source}")
            if(test_name STREQUAL "test_command")
                target_link_libraries(${test_name} PRIVATE a2_runtime)
            else()
                target_link_libraries(${test_name} PRIVATE a2_core)
            endif()
            add_test(NAME ${test_name} COMMAND ${test_name})
        else()
            message(STATUS "Skipping ${test_name}: ${test_source} does not exist")
        endif()
    endforeach()
endif()

install(TARGETS a2_core a2_runtime a2_deploy
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY include/a2/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/a2
    OPTIONAL
)

install(FILES params/a2.yaml
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/params
    OPTIONAL
)

install(FILES models/a2_45d_policy.jit models/README.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/models
    OPTIONAL
)

install(PROGRAMS sim/a2_mujoco.py sim/a2_highcmd.py sim/a2_prepare_low_policy.py
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/sim
    OPTIONAL
)

install(DIRECTORY assets/a2
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/assets
    OPTIONAL
)

install(FILES LICENSE THIRD_PARTY_NOTICES.md
    DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/${PROJECT_NAME}
    OPTIONAL
)
