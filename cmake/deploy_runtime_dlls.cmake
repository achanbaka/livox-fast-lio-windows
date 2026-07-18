if(NOT IS_DIRECTORY "${SOURCE_DIR}")
    message(FATAL_ERROR "Runtime DLL source directory does not exist: ${SOURCE_DIR}")
endif()

file(MAKE_DIRECTORY "${DESTINATION_DIR}")
file(GLOB runtime_dlls LIST_DIRECTORIES false "${SOURCE_DIR}/*.dll")
foreach(runtime_dll IN LISTS runtime_dlls)
    # Avoid rewriting DLLs that are already identical.  Besides reducing build
    # noise, this lets test targets rebuild while the main executable has the
    # same deployed PCL DLLs mapped into its running process.
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${runtime_dll}" "${DESTINATION_DIR}"
        RESULT_VARIABLE copy_result)
    if(NOT copy_result EQUAL 0)
        message(FATAL_ERROR "Could not deploy runtime DLL: ${runtime_dll}")
    endif()
endforeach()
