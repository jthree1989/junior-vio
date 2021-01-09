file(GLOB_RECURSE SRC *.cpp *.cc)

add_library(junior-vio ${SRC})

list(APPEND deps )

if(OPTION_BUILD_WITH_OPENCV)
    list(APPEND deps CONAN_PKG::opencv)
    # https://stackoverflow.com/questions/9017573/define-preprocessor-macro-through-cmake
    target_compile_definitions(junior-vio PUBLIC WITH_OPENCV=1)
endif()

target_link_libraries(junior-vio PUBLIC ${deps})