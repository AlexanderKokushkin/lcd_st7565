#set(COMPONENT_ADD_INCLUDEDIRS "C:/dev/esp/libraries/")
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS ".")

set_source_files_properties(main.cpp
    PROPERTIES COMPILE_FLAGS
    "-fpermissive -Wno-unused-variable -std=gnu++17"
)

#  component_compile_options(-std=c++17).
