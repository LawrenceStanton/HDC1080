cmake_minimum_required(VERSION 3.24)

project(HDC1080)

set(LIBRARY ${PROJECT_NAME})

enable_language(CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Requires std::optional in C++17, default comparisons in C++20.

add_library(${LIBRARY} STATIC
	Src/HDC1080.cpp
)

target_include_directories(${LIBRARY} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

add_library(${LIBRARY}::${LIBRARY} ALIAS ${LIBRARY})

if(NOT CMAKE_CROSSCOMPILING)
	option(HDC1080_CODE_COVERAGE "Enable gcovr code coverage for HDC1080" OFF)
	option(HDC1080_AUTOFETCH_GTEST "Automatically fetch GTest if not included by parent." ON)

	if(HDC1080_AUTOFETCH_GTEST)
		if(NOT TARGET GTest::gtest OR NOT TARGET GTest::gmock)
			message(STATUS "GTest not found. Fetching...")

			include(FetchContent)
			FetchContent_Declare(
				googletest
				GIT_REPOSITORY https://github.com/google/googletest
				GIT_TAG origin/main
			)
			set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
			FetchContent_MakeAvailable(googletest)

			enable_testing()
		endif()
	endif()

	# Add addittional configuration to library target.
	target_link_libraries(${LIBRARY} PRIVATE
		GTest::gtest
	)

	target_compile_options(${LIBRARY} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	target_link_options(${LIBRARY} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	# GTest Test Executable
	set(TEST_EXECUTABLE ${LIBRARY}_Test)
	
	add_executable(${TEST_EXECUTABLE}
		Test/HDC1080.test.cpp
	)

	target_compile_options(${TEST_EXECUTABLE} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	target_include_directories(${TEST_EXECUTABLE} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/Inc
	)

	target_link_libraries(${TEST_EXECUTABLE} PRIVATE
		${PROJECT_NAME}::${LIBRARY}
		GTest::gtest_main
		GTest::gmock
	)

	target_link_options(${TEST_EXECUTABLE} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)
	
	include(GoogleTest)
	gtest_discover_tests(${TEST_EXECUTABLE})

	if(HDC1080_CODE_COVERAGE)
		set(GCOVR_COMMAND gcovr --root ${CMAKE_SOURCE_DIR} --gcov-executable gcov-13 --filter '.*HDC1080/.*' --exclude '.*\.test\..*' ${CMAKE_CURRENT_BINARY_DIR})
		set(SILENT_RUN_COMMAND ./${TEST_EXECUTABLE} > /dev/null)

		add_custom_command(TARGET ${TEST_EXECUTABLE} POST_BUILD
			COMMENT "Running gcovr Code Coverage for HDC1080 Tests..."
			COMMAND ${SILENT_RUN_COMMAND} && ${GCOVR_COMMAND} || echo "Code coverage failed. All tests must pass for code coverage to run."
		)
	endif()
endif()