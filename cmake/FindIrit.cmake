# Find Irit
#
# Irit_INCLUDE_DIRS
# Irit_LIBRARIES
# Irit_COMPILE_DEFINITIONS
# Irit_FOUND

if(WIN32)
	set(IRIT_ROOT "C:\\irit\\irit\\ntbin")
	set(IRIT64_ROOT "C:\\irit\\irit\\ntbin64")
else()
	set(IRIT_ROOT "/usr/local/irit/bin")
	set(IRIT64_ROOT "/usr/local/irit/bin")
endif()

if(NOT $ENV{IRIT_ROOT} STREQUAL "")
	set(IRIT_ROOT $ENV{IRIT_ROOT})
endif()

if(NOT $ENV{IRIT64_ROOT} STREQUAL "")
	set(IRIT64_ROOT $ENV{IRIT64_ROOT})
endif()

set(Irit_COMPILE_DEFINITIONS "")

if(WIN32)
	find_path(Irit_INCLUDE_DIRS NAMES inc_irit/irit_sm.h HINTS
		"${IRIT_ROOT}/.."
		"${IRIT_ROOT}/../.."
		"${IRIT64_ROOT}/.."
		"${IRIT64_ROOT}/../.."
	)

	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		if(CMAKE_BUILD_TYPE MATCHES Debug)
			find_library(Irit_LIBRARIES NAMES IritD64 HINTS
				"${IRIT64_ROOT}/../lib64"
				"${IRIT_ROOT}/../lib64")
			find_file(irit_dynamic_lib_path NAMES IritD64.dll HINTS
				"${IRIT64_ROOT}"
				"${IRIT_ROOT}")
		else()
			find_library(Irit_LIBRARIES NAMES Irit64 HINTS
				"${IRIT64_ROOT}/../lib64"
				"${IRIT_ROOT}/../lib64")
			find_file(irit_dynamic_lib_path NAMES Irit64.dll HINTS
				"${IRIT64_ROOT}"
				"${IRIT_ROOT}")
		endif()

		if(MSVC)
			set(Irit_COMPILE_DEFINITIONS "WIN64" "__WINNT__")
		endif()
	else()
		if(CMAKE_BUILD_TYPE MATCHES Debug)
			find_library(Irit_LIBRARIES NAMES IritD HINTS
				"${IRIT64_ROOT}/../lib64"
				"${IRIT_ROOT}/../lib64")
			find_file(irit_dynamic_lib_path NAMES IritD.dll HINTS
				"${IRIT64_ROOT}"
				"${IRIT_ROOT}")
		else()
			find_library(Irit_LIBRARIES NAMES Irit HINTS
				"${IRIT64_ROOT}/../lib64"
				"${IRIT_ROOT}/../lib64")
			find_file(irit_dynamic_lib_path NAMES Irit.dll HINTS
				"${IRIT64_ROOT}"
				"${IRIT_ROOT}")
		endif()

		if(MSVC)
			set(Irit_COMPILE_DEFINITIONS "WIN32" "__WINNT__")
		endif()
	endif()

else()
	find_path(Irit_INCLUDE_DIRS NAMES inc_irit/irit_sm.h HINTS
		"${IRIT_ROOT}/..")

	if(CMAKE_BUILD_TYPE MATCHES Debug)
		find_library(Irit_LIBRARIES NAMES IritD HINTS
			"${IRIT_ROOT}/../lib"
			"${IRIT_ROOT}")
		find_file(irit_dynamic_lib_path NAMES IritD.so HINTS
			"${IRIT_ROOT}"
			"${IRIT_ROOT}/../lib")
	else()
		find_library(Irit_LIBRARIES NAMES Irit HINTS
			"${IRIT_ROOT}/../lib"
			"${IRIT_ROOT}")
		find_file(irit_dynamic_lib_path NAMES Irit.so HINTS
			"${IRIT_ROOT}"
			"${IRIT_ROOT}/../lib")
	endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Irit DEFAULT_MSG Irit_LIBRARIES Irit_INCLUDE_DIRS)
mark_as_advanced(Irit_INCLUDE_DIRS Irit_LIBRARIES)
