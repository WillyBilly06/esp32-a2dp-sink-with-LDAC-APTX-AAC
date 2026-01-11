cmake_minimum_required(VERSION 3.15)

idf_component_register(SRCS src/ldacBT.c
                            src/ldaclib.c
                       INCLUDE_DIRS inc
                       PRIV_INCLUDE_DIRS inc src)

target_compile_options(${COMPONENT_LIB} PRIVATE -Werror=implicit-function-declaration)

