include(FetchContent)

macro(collect_include_dirs result curdir)
	file(GLOB children ${curdir}/*)
	set(dirlist "")

	foreach(child ${children})
		if(IS_DIRECTORY ${child})
			list(APPEND dirlist ${child}/include)
		endif()
	endforeach()

	set(${result} ${dirlist})
endmacro()

macro(FetchContent_MakeAvailableExclude dep)
	FetchContent_GetProperties(${dep})

	if(NOT ${dep}_POPULATED)
		FetchContent_Populate(${dep})
		add_subdirectory(${${dep}_SOURCE_DIR} ${${dep}_BINARY_DIR} EXCLUDE_FROM_ALL)
	endif()
endmacro()
