add_executable(realsense_simpleslam run_realsense.cpp)
target_link_libraries(realsense_simpleslam simpleslam ${THIRD_PARTY_LIBS})

add_executable(reconstruction run_reconstruction.cpp)
target_link_libraries(reconstruction simpleslam ${THIRD_PARTY_LIBS})
