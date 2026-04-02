if(NOT DEFINED NSX_BOARD_FLAGS_TARGET)
    message(FATAL_ERROR "Board must define NSX_BOARD_FLAGS_TARGET before nsx-audio is added.")
endif()
if(NOT DEFINED NSX_SOC_FAMILY)
    message(FATAL_ERROR "NSX_SOC_FAMILY must be defined before nsx-audio is added.")
endif()

# Map SoC family to the correct arch-specific source directory.
set(NSX_AUDIO_ARCH_DIR "")
if(NSX_SOC_FAMILY STREQUAL "apollo5b" OR NSX_SOC_FAMILY STREQUAL "apollo510" OR NSX_SOC_FAMILY STREQUAL "apollo510b")
    set(NSX_AUDIO_ARCH_DIR "apollo5")
elseif(NSX_SOC_FAMILY STREQUAL "apollo4p" OR NSX_SOC_FAMILY STREQUAL "apollo4l" OR NSX_SOC_FAMILY STREQUAL "apollo4b")
    set(NSX_AUDIO_ARCH_DIR "apollo4")
else()
    message(FATAL_ERROR "Unsupported NSX_SOC_FAMILY for nsx-audio: ${NSX_SOC_FAMILY}. Supported: apollo5b/apollo510/apollo510b/apollo4p/apollo4l/apollo4b.")
endif()

file(GLOB nsx_audio_arch_sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_LIST_DIR}/src/${NSX_AUDIO_ARCH_DIR}/*.c
)

add_library(nsx_audio STATIC
    ${nsx_audio_arch_sources}
)
set_target_properties(nsx_audio PROPERTIES EXPORT_NAME audio)
add_library(nsx::audio ALIAS nsx_audio)

target_link_libraries(nsx_audio PUBLIC
    ${NSX_BOARD_FLAGS_TARGET}
    nsx_soc_hal
    nsx_core
    nsx_harness
)

target_include_directories(nsx_audio PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/includes-api>
    $<BUILD_INTERFACE:${NSX_ROOT}/modules/nsx-core/includes-api>
    $<BUILD_INTERFACE:${NSX_ROOT}/modules/nsx-harness/includes-api>
    $<INSTALL_INTERFACE:include>
)

install(DIRECTORY includes-api/ DESTINATION include)
install(TARGETS nsx_audio EXPORT nsxTargets)
