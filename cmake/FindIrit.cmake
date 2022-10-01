# Find Irit
#
# Irit_INCLUDE_DIRS
# Irit_LIBRARIES
# Irit_FOUND
if(WIN32)
	find_path(Irit_INCLUDE_DIRS NAMES inc_irit/irit_sm.h HINTS
		"$ENV{IRIT_PATH}/.."
		"$ENV{IRIT_PATH}/../.."
		"$ENV{IRIT_PATH64}/.."
		"$ENV{IRIT_PATH64}/../.."
	)

	if(CMAKE_SIZEOF_VOID_P EQUAL 8)
		if(CMAKE_BUILD_TYPE MATCHES Debug)
			find_library(Irit_LIBRARIES NAMES IritD64 HINTS
				"$ENV{IRIT_PATH64}/../lib64"
				"$ENV{IRIT_PATH}/../lib64")
			find_file(irit_dynamic_lib_path NAMES IritD64.dll HINTS
				"$ENV{IRIT_PATH64}"
				"$ENV{IRIT_PATH}")
		else()
			find_library(Irit_LIBRARIES NAMES Irit64 HINTS
				"$ENV{IRIT_PATH64}/../lib64"
				"$ENV{IRIT_PATH}/../lib64")
			find_file(irit_dynamic_lib_path NAMES Irit64.dll HINTS
				"$ENV{IRIT_PATH64}"
				"$ENV{IRIT_PATH}")
		endif()

		if(MSVC)
			add_compile_definitions("WIN64")
			add_compile_definitions("__WINNT__")
		endif()
	else()
		if(CMAKE_BUILD_TYPE MATCHES Debug)
			find_library(Irit_LIBRARIES NAMES IritD HINTS
				"$ENV{IRIT_PATH64}/../lib64"
				"$ENV{IRIT_PATH}/../lib64")
			find_file(irit_dynamic_lib_path NAMES IritD.dll HINTS
				"$ENV{IRIT_PATH64}"
				"$ENV{IRIT_PATH}")
		else()
			find_library(Irit_LIBRARIES NAMES Irit HINTS
				"$ENV{IRIT_PATH64}/../lib64"
				"$ENV{IRIT_PATH}/../lib64")
			find_file(irit_dynamic_lib_path NAMES Irit.dll HINTS
				"$ENV{IRIT_PATH64}"
				"$ENV{IRIT_PATH}")
		endif()

		if(MSVC)
			add_compile_definitions("WIN32")
			add_compile_definitions("__WINNT__")
		endif()
	endif()

else()
	find_path(Irit_INCLUDE_DIRS NAMES inc_irit/irit_sm.h HINTS
		"$ENV{IRIT_PATH}/..")

	if(CMAKE_BUILD_TYPE MATCHES Debug)
		find_library(Irit_LIBRARIES NAMES IritD HINTS
			"$ENV{IRIT_PATH}/../lib"
			"$ENV{IRIT_PATH}")
		find_file(irit_dynamic_lib_path NAMES IritD.so HINTS
			"$ENV{IRIT_PATH}"
			"$ENV{IRIT_PATH}/../lib")
	else()
		find_library(Irit_LIBRARIES NAMES Irit HINTS
			"$ENV{IRIT_PATH}/../lib"
			"$ENV{IRIT_PATH}")
		find_file(irit_dynamic_lib_path NAMES Irit.so HINTS
			"$ENV{IRIT_PATH}"
			"$ENV{IRIT_PATH}/../lib")
	endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Irit DEFAULT_MSG Irit_LIBRARIES Irit_INCLUDE_DIRS)
mark_as_advanced(Irit_INCLUDE_DIRS Irit_LIBRARIES)
