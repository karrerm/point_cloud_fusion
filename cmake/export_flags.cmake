set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++11")
if (NOT ANDROID)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mssse3")  
endif()
