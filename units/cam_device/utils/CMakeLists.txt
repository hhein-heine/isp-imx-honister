add_subdirectory(base64)

if((${ISP_VERSION} STREQUAL ISP8000NANO_V1802) AND (${PLATFORM} STREQUAL ARM64) AND (NOT ANDROID))

else ((${ISP_VERSION} STREQUAL ISP8000NANO_V1802) AND (${PLATFORM} STREQUAL ARM64) AND (NOT ANDROID))
add_subdirectory(tinyxml2)
endif ((${ISP_VERSION} STREQUAL ISP8000NANO_V1802) AND (${PLATFORM} STREQUAL ARM64) AND (NOT ANDROID))

