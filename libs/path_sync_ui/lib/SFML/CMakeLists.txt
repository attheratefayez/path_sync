file(GLOB lib_files "*.so")

foreach(lib_file ${lib_files})

    get_filename_component(lib_filename ${lib_file} NAME_WE)
    string(REPLACE "lib" "" lib_filename ${lib_filename})

    add_library(${lib_filename} SHARED IMPORTED)
    set_target_properties(${lib_filename} PROPERTIES
        IMPORTED_LOCATION ${lib_file}
    )
    message("[ADDED LIBRARY] ${lib_filename}")

endforeach()

message("[LOADED] SFML")
