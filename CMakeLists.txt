project(HDC1080)

set(EXECUTABLE ${PROJECT_NAME})

set(EXTERNAL_LIBRARY_TARGETS
	etl::etl
)

foreach(EXTERNAL_LIBRARY_TARGET ${EXTERNAL_LIBRARY_TARGETS})
	if(NOT TARGET ${EXTERNAL_LIBRARY_TARGET})
		message(FATAL_ERROR "External library target ${EXTERNAL_LIBRARY_TARGET} not found. Please add it to the project.")
	endif()
endforeach()

enable_language(CXX)

add_library(${EXECUTABLE} STATIC
	Src/HDC1080.cpp
)

target_include_directories(${EXECUTABLE} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(${EXECUTABLE} PUBLIC
	${EXTERNAL_LIBRARY_TARGETS}
)

add_library(${PROJECT_NAME}::${EXECUTABLE} ALIAS ${EXECUTABLE})

if(NOT CMAKE_CROSSCOMPILING)
	option(HDC1080_CODE_COVERAGE "Enable code coverage for HDC1080" ON)

	set(TEST_EXECUTABLE ${EXECUTABLE}_Test)

	# Add addittional configuration to library target.
	target_compile_definitions(${EXECUTABLE} INTERFACE
		HDC1080_GTEST_TESTING
	)

	target_link_libraries(${EXECUTABLE} PRIVATE
		GTest::gtest
	)

	target_compile_options(${EXECUTABLE} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	target_link_options(${EXECUTABLE} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	# GTest Test Executable
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
		${PROJECT_NAME}::${EXECUTABLE}
		GTest::gtest_main
		GTest::gmock
	)

	target_link_options(${TEST_EXECUTABLE} PRIVATE
		$<$<BOOL:${HDC1080_CODE_COVERAGE}>:--coverage>
	)

	include(CTest)
	
	include(GoogleTest)
	gtest_discover_tests(${TEST_EXECUTABLE})

	if(HDC1080_CODE_COVERAGE)
		option(HDC1080_CODE_COVERAGE_EXCLUDE "Enable code coverage for HDC1080" ON)

		add_custom_command(TARGET ${TEST_EXECUTABLE} POST_BUILD
			COMMENT "Running gcovr Code Coverage for HDC1080 Tests"
			COMMENT "Please endure the test target is executed before running this command."
			COMMAND ./${TEST_EXECUTABLE} > /dev/null # Silent Run of Executable
			COMMAND	gcovr --root ${CMAKE_SOURCE_DIR} --exclude .clang-format --gcov-executable gcov-13 --filter '.*HDC1080/*'
		)
	endif()
endif()