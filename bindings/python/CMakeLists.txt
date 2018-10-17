
IF(BUILD_PYTHON_INTERFACE)
  SET(${PROJECT_NAME}_PYTHON_HEADERS
      fwd.hpp
      timeopt/expose-timeopt.hpp
      timeopt/phase.hpp
      timeopt/problem.hpp
      timeopt/enums.hpp

      container/array.hpp
      container/visitor.hpp
      container/reference-wrapper.hpp

      serialization/archive.hpp
    )

  SET(${PROJECT_NAME}_PYTHON_SOURCES
      module.cpp
      timeopt/phase.cpp
      timeopt/problem.cpp
      timeopt/enums.cpp
    )  

  LIST(APPEND HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS})
  LIST(REMOVE_DUPLICATES HEADERS)

  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/timeopt")
  MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/container")

  FOREACH(header ${${PROJECT_NAME}_PYTHON_HEADERS})
    GET_FILENAME_COMPONENT(headerName ${header} NAME)
    GET_FILENAME_COMPONENT(headerPath ${header} PATH)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${header}
      ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/bindings/python/${header})
    INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${header}
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/bindings/python
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
  ENDFOREACH(header)

  # --- PyWrap library --- #
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  ADD_LIBRARY(${PYWRAP} SHARED ${${PROJECT_NAME}_PYTHON_SOURCES} ${${PROJECT_NAME}_PYTHON_HEADERS})
  ADD_HEADER_GROUP(${PROJECT_NAME}_PYTHON_HEADERS)
  ADD_SOURCE_GROUP(${PROJECT_NAME}_PYTHON_SOURCES)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)
  PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} pinocchio)
    
  TARGET_LINK_LIBRARIES(${PYWRAP} ${PROJECT_NAME})
  TARGET_LINK_BOOST_PYTHON(${PYWRAP})

  IF(APPLE)
    # We need to change the extension for python bindings
    SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
  ENDIF(APPLE)

  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}")

  INSTALL(FILES
    "${CMAKE_BINARY_DIR}/lib/python/${PROJECT_NAME}/lib${PYWRAP}.so"
    DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

   # --- INSTALL SCRIPTS 
  SET(PYTHON_FILES
    __init__.py
    )

  FOREACH(python ${PYTHON_FILES})
    GET_FILENAME_COMPONENT(pythonFile ${python} NAME)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${python}
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})

    # Tag pyc file as generated.
    SET_SOURCE_FILES_PROPERTIES(
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      PROPERTIES GENERATED TRUE)

    EXECUTE_PROCESS(COMMAND
      ${PYTHON_EXECUTABLE} -m py_compile
      ${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile})

    # Clean generated files.
    SET_PROPERTY(
      DIRECTORY APPEND PROPERTY
      ADDITIONAL_MAKE_CLEAN_FILES
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c")

    INSTALL(FILES
      "${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${python}"
      "${${PROJECT_NAME}_BINARY_DIR}/lib/python/${PROJECT_NAME}/${pythonFile}c"
      DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})
  ENDFOREACH(python)

ENDIF(BUILD_PYTHON_INTERFACE)

