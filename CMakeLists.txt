project(HDC1080)

set(LIBRARY ${PROJECT_NAME})

enable_language(CXX)

add_library(${LIBRARY} STATIC
	Src/HDC1080.cpp
)

target_include_directories(${LIBRARY} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

target_link_libraries(${LIBRARY} PUBLIC
	${EXTERNAL_LIBRARY_TARGETS}
)

add_library(${PROJECT_NAME}::${LIBRARY} ALIAS ${LIBRARY})

if(NOT CMAKE_CROSSCOMPILING)
	option(HDC1080_CODE_COVERAGE "Enable code coverage for HDC1080" OFF)

	set(TEST_EXECUTABLE ${LIBRARY}_Test)

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

		set(GCOVR_COMMAND gcovr --root ${CMAKE_SOURCE_DIR} --exclude .clang-format --gcov-executable gcov-13 --filter '.*HDC1080/*')
		set(SILENT_RUN_COMMAND ./${TEST_EXECUTABLE} > /dev/null)

		add_custom_command(TARGET ${TEST_EXECUTABLE} POST_BUILD
			COMMENT "Running gcovr Code Coverage for HDC1080 Tests"
			COMMAND ${SILENT_RUN_COMMAND} && ${GCOVR_COMMAND} || echo "Code coverage failed."
		)
	endif()
endif()