# Offline tests. Run all:  ctest --test-dir build --output-on-failure
set(ROBOT_DIR "${CMAKE_SOURCE_DIR}/robot kol stl parçaları")

function(pegasus_test name)
  add_executable(${name} ${name}.cpp ${PEGASUS_WIN_MANIFEST})
  target_link_libraries(${name} PRIVATE pegasus_core)
  target_compile_definitions(${name} PRIVATE
    PEGASUS_ROBOT_DIR="${ROBOT_DIR}"
    PEGASUS_TEST_TMP="${CMAKE_CURRENT_BINARY_DIR}/tmp")
endfunction()

foreach(t geom_test expr_test motion_test ik_test scan_test link_test)
  pegasus_test(${t})
  add_test(NAME ${t} COMMAND ${t})
endforeach()
set_tests_properties(link_test ik_test PROPERTIES TIMEOUT 300)
