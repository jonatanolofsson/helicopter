option(TEST_ENABLE "Enable tests" Off)

if(TEST_ENABLE)
    enable_testing()

    add_custom_target(run_tests ctest)

    OPTION( ENABLE_CODECOVERAGE "Enable code coverage testing support" )

    if ( ENABLE_CODECOVERAGE )

        if ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
            message( WARNING "Code coverage results with an optimised (non-Debug) build may be misleading" )
        endif ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )

        if ( NOT DEFINED CODECOV_OUTPUTFILE )
            set( CODECOV_OUTPUTFILE coverage.info )
        endif ( NOT DEFINED CODECOV_OUTPUTFILE )

        if ( NOT DEFINED CODECOV_HTMLOUTPUTDIR )
            set( CODECOV_HTMLOUTPUTDIR coverage )
        endif ( NOT DEFINED CODECOV_HTMLOUTPUTDIR )


        if (CMAKE_COMPILER_IS_GNUCXX)
            find_program( CODECOV_GCOV gcov )
            find_program( CODECOV_LCOV lcov )
            find_program( CODECOV_GENHTML genhtml )

            add_definitions(-fprofile-arcs -ftest-coverage -lgcov)
            set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs --coverage")

            add_custom_target(coverage
                COMMAND ${CODECOV_LCOV} --base-directory "${CMAKE_SOURCE_DIR}"  --directory ${CMAKE_BINARY_DIR} --output-file ${CODECOV_OUTPUTFILE} --capture --initial
                COMMAND ctest
                COMMAND ${CODECOV_LCOV} --base-directory "${CMAKE_SOURCE_DIR}"  --directory ${CMAKE_BINARY_DIR} --output-file ${CODECOV_OUTPUTFILE} --capture
                COMMAND ${CODECOV_LCOV} --output-file ${CODECOV_OUTPUTFILE} --extract ${CODECOV_OUTPUTFILE} "'${CMAKE_SOURCE_DIR}*'"
                COMMAND genhtml -o ${CODECOV_HTMLOUTPUTDIR} ${CODECOV_OUTPUTFILE}
            )
            add_dependencies(coverage coverage_init)
        endif ( CMAKE_COMPILER_IS_GNUCXX )

    endif (ENABLE_CODECOVERAGE )
endif()

macro(test_project TESTNAME)
    project(${TESTNAME} CXX)
    if(NOT TEST_ENABLE OR NOT TEST_${TESTNAME})
        return()
    endif()
endmacro(test_project)

function(make_test TNAME)
    if(TEST_ENABLE)
        add_executable("${TNAME}" "${TNAME}.cpp" ${ARGN})
        target_link_libraries("${TNAME}" gtest_main gtest pthread)
        add_test("${TNAME}" "${TNAME}")
        add_dependencies(run_tests "${TNAME}")
    endif()
endfunction(make_test)

macro(add_testdirectory DNAME)
    if(TEST_ENABLE)
        add_subdirectory(${DNAME})
    endif()
endmacro()
