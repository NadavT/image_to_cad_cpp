include(cmake/utils.cmake)

FetchContent_Declare(
	argparse
	GIT_REPOSITORY https://github.com/p-ranav/argparse.git
	GIT_TAG v2.7
	GIT_PROGRESS TRUE
	GIT_SHALLOW TRUE
)
FetchContent_MakeAvailableExclude(argparse)
