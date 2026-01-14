cmake_minimum_required(VERSION 3.5.0)
if (NOT DEFINED GUI_BASE_DIR)
	if (DEFINED ENV{GUI_BASE_DIR})
		set(GUI_BASE_DIR $ENV{GUI_BASE_DIR})
	else()
		set(GUI_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../plugin-GUI)
	endif()
endif()

get_filename_component(PROJECT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
get_filename_component(PLUGIN_NAME ${PROJECT_FOLDER} NAME)

set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "Build architecture for Mac OS X" FORCE)

project(OE_PLUGIN_${PLUGIN_NAME})
set(CMAKE_SHARED_LIBRARY_PREFIX "")
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	set(LINUX 1)
	if(NOT CMAKE_BUILD_TYPE)
		set(CMAKE_BUILD_TYPE Debug)
	endif()
endif()

set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS
	OEPLUGIN
	"$<$<PLATFORM_ID:Windows>:JUCE_API=__declspec(dllimport)>"
	$<$<PLATFORM_ID:Windows>:_CRT_SECURE_NO_WARNINGS>
	$<$<PLATFORM_ID:Linux>:JUCE_DISABLE_NATIVE_FILECHOOSERS=1>
	$<$<CONFIG:Debug>:DEBUG=1>
	$<$<CONFIG:Debug>:_DEBUG=1>
	$<$<CONFIG:Release>:NDEBUG=1>
	)

set(SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Source)
file(GLOB_RECURSE SRC_FILES LIST_DIRECTORIES false "${SOURCE_PATH}/*.cpp" "${SOURCE_PATH}/*.h")
set(GUI_COMMONLIB_DIR ${GUI_BASE_DIR}/installed_libs)

set(CMAKE_CXX_STANDARD 17)

set(CONFIGURATION_FOLDER $<$<CONFIG:Debug>:Debug>$<$<NOT:$<CONFIG:Debug>>:Release>)

list(APPEND CMAKE_PREFIX_PATH ${GUI_COMMONLIB_DIR} ${GUI_COMMONLIB_DIR}/${CONFIGURATION_FOLDER})

if (APPLE)
	add_library(${PLUGIN_NAME} MODULE ${SRC_FILES})
else()
	add_library(${PLUGIN_NAME} SHARED ${SRC_FILES})
endif()

target_compile_features(${PLUGIN_NAME} PUBLIC cxx_auto_type cxx_generalized_initializers)
target_include_directories(${PLUGIN_NAME} PUBLIC ${GUI_BASE_DIR}/JuceLibraryCode ${GUI_BASE_DIR}/JuceLibraryCode/modules ${GUI_BASE_DIR}/Plugins/Headers ${GUI_COMMONLIB_DIR}/include)

set(GUI_BIN_DIR ${GUI_BASE_DIR}/Build/${CONFIGURATION_FOLDER})

if (NOT CMAKE_LIBRARY_ARCHITECTURE)
	if (CMAKE_SIZEOF_VOID_P EQUAL 8)
		set(CMAKE_LIBRARY_ARCHITECTURE "x64")
	else()
		set(CMAKE_LIBRARY_ARCHITECTURE "x86")
	endif()
endif()

#Libraries and compiler options
if(MSVC)
	target_link_libraries(${PLUGIN_NAME} ${GUI_BIN_DIR}/open-ephys.lib)
	target_compile_options(${PLUGIN_NAME} PRIVATE /sdl- /W0 /MP)
	
	install(TARGETS ${PLUGIN_NAME} RUNTIME DESTINATION ${GUI_BIN_DIR}/plugins  CONFIGURATIONS ${CMAKE_CONFIGURATION_TYPES})
	install(FILES $<TARGET_PDB_FILE:${PLUGIN_NAME}> DESTINATION ${GUI_BIN_DIR}/plugins OPTIONAL)

elseif(LINUX)
	target_link_libraries(${PLUGIN_NAME} GL X11 Xext Xinerama asound dl freetype pthread rt)
	set_property(TARGET ${PLUGIN_NAME} APPEND_STRING PROPERTY LINK_FLAGS
		"-fvisibility=hidden -fPIC -rdynamic -Wl,-rpath='$ORIGIN/../shared' -Wl,-rpath='$ORIGIN/../shared-api10'")
	target_compile_options(${PLUGIN_NAME} PRIVATE -fPIC -rdynamic)
	target_compile_options(${PLUGIN_NAME} PRIVATE -O3) #enable optimization for linux debug
	
	install(TARGETS ${PLUGIN_NAME} LIBRARY DESTINATION ${GUI_BIN_DIR}/plugins)
elseif(APPLE)
	set_target_properties(${PLUGIN_NAME} PROPERTIES BUNDLE TRUE)
	set_property(TARGET ${PLUGIN_NAME} APPEND_STRING PROPERTY LINK_FLAGS
	"-undefined dynamic_lookup -rpath @loader_path/../../../../shared-api10")

	set_target_properties(${PLUGIN_NAME} PROPERTIES
		XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY ""
		XCODE_ATTRIBUTE_CODE_SIGNING_REQUIRED NO)

	add_custom_command(TARGET ${PLUGIN_NAME} POST_BUILD COMMAND
	install_name_tool -change "libonidriver_ft600.dylib"
	"@rpath/libonidriver_ft600.dylib" $<TARGET_FILE:${PLUGIN_NAME}>)

	install(TARGETS ${PLUGIN_NAME} DESTINATION $ENV{HOME}/Library/Application\ Support/open-ephys/plugins-api10)
endif()

#create filters for vs and xcode

foreach( src_file IN ITEMS ${SRC_FILES})
	get_filename_component(src_path "${src_file}" PATH)
	file(RELATIVE_PATH src_path_rel "${SOURCE_PATH}" "${src_path}")
	string(REPLACE "/" "\\" group_name "${src_path_rel}")
	source_group("${group_name}" FILES "${src_file}")
endforeach()

#additional libraries
if (MSVC)
	target_link_libraries(${PLUGIN_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/Resources/okFrontPanel.lib")
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Resources/okFrontPanel.dll DESTINATION ${GUI_BIN_DIR}/shared CONFIGURATIONS ${CMAKE_CONFIGURATION_TYPES})
elseif(LINUX)
	target_link_libraries(${PLUGIN_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/Resources/libokFrontPanel.so")
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Resources/libokFrontPanel.so DESTINATION ${GUI_BIN_DIR}/shared)
elseif(APPLE)
	set(MAC_SHARED_FOLDER $ENV{HOME}/Library/Application\ Support/open-ephys/shared-api10)

	target_link_libraries(${PLUGIN_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/Resources/libokFrontPanel.dylib")
	install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/Resources/libokFrontPanel.1.dylib DESTINATION "${MAC_SHARED_FOLDER}")
endif()

function(vs_set_all_configurations target_name target_config_name)
    foreach(config ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${config} config_upper)
        set_property(
            TARGET ${target_name}
            APPEND PROPERTY MAP_IMPORTED_CONFIG_${config_upper} "${target_config_name}"
        )
    endforeach()
endfunction()

include(ExternalProject)

if (MSVC)
  # add liboni project

  set(LIBONI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/liboni/api/liboni")
  set(LIBONI_NAME liboni)
  set(LIBONI_PNAME "submodule_${LIBONI_NAME}")

  include_external_msproject (
      ${LIBONI_PNAME}
      "${LIBONI_DIR}/liboni.vcxproj"
      PLATFORM  ${CMAKE_LIBRARY_ARCHITECTURE}
  )

  vs_set_all_configurations(${LIBONI_PNAME} "ReleaseStatic")

  add_library("${LIBONI_NAME}" STATIC IMPORTED)
  set_target_properties("${LIBONI_NAME}" PROPERTIES
    IMPORTED_LOCATION "${LIBONI_DIR}/${CMAKE_LIBRARY_ARCHITECTURE}/ReleaseStatic/liboni.lib"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBONI_DIR}")

  add_dependencies(${LIBONI_NAME} ${LIBONI_PNAME})
  target_link_libraries(${PLUGIN_NAME} "${LIBONI_NAME}")

  # Add onidriver_ft600 project

  set(ONIDRIVER_FT600_DIR "${LIBONI_DIR}/drivers/ft600")
  set(ONIDRIVER_FT600_NAME libonidriver_ft600)
  set(ONIDRIVER_FT600_PNAME "submodule_${ONIDRIVER_FT600_NAME}")

  include_external_msproject(
    ${ONIDRIVER_FT600_PNAME}
    "${ONIDRIVER_FT600_DIR}/onidriver_ft600.vcxproj"
    PLATFORM ${CMAKE_LIBRARY_ARCHITECTURE}
  )

  vs_set_all_configurations(${ONIDRIVER_FT600_PNAME} "Release")

  add_dependencies(${ONIDRIVER_FT600_PNAME} ${LIBONI_PNAME})

  # Ensure needed DLLs are installed to the shared folder
  install(FILES "${CMAKE_BINARY_DIR}/${CMAKE_LIBRARY_ARCHITECTURE}/Release/${ONIDRIVER_FT600_NAME}.dll" DESTINATION ${GUI_BIN_DIR}/shared)
elseif(LINUX OR APPLE)
  # Add liboni project

  set(LIBONI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/liboni/api/liboni")
	set(LIBONI_NAME liboni)
	set(LIBONI_PNAME "submodule_${LIBONI_NAME}")

	ExternalProject_Add(
		${LIBONI_PNAME}
		SOURCE_DIR "${LIBONI_DIR}"
		CONFIGURE_COMMAND ""
		BUILD_COMMAND $(MAKE)
		BUILD_IN_SOURCE 1
		INSTALL_COMMAND ""
	)

	add_library("${LIBONI_NAME}" STATIC IMPORTED GLOBAL)
	set_target_properties("${LIBONI_NAME}" PROPERTIES
		IMPORTED_LOCATION "${LIBONI_DIR}/liboni.a"
		INTERFACE_INCLUDE_DIRECTORIES "${LIBONI_DIR}")

	add_dependencies(${LIBONI_NAME} ${LIBONI_PNAME})
	target_link_libraries(${PLUGIN_NAME} "${LIBONI_NAME}")

	# Add onidriver_ft600 project

	set(ONIDRIVER_FT600_DIR "${LIBONI_DIR}/drivers/ft600")
	set(ONIDRIVER_FT600_NAME libonidriver_ft600)
	set(ONIDRIVER_FT600_PNAME "submodule_${ONIDRIVER_FT600_NAME}")

	ExternalProject_Add(
		${ONIDRIVER_FT600_PNAME}
		SOURCE_DIR "${ONIDRIVER_FT600_DIR}"
		CONFIGURE_COMMAND ""
		BUILD_COMMAND $(MAKE)
		BUILD_IN_SOURCE 1
		INSTALL_COMMAND ""
		DEPENDS ${LIBONI_PNAME}
	)

	if(LINUX)
		set(ONIDRIVER_FT600_FILENAME "${ONIDRIVER_FT600_NAME}.so")

		# Ensure needed shared libraries are installed to the shared folder
		install(FILES "${ONIDRIVER_FT600_DIR}/${ONIDRIVER_FT600_FILENAME}.1" 
				DESTINATION ${GUI_BIN_DIR}/shared
				RENAME "${ONIDRIVER_FT600_FILENAME}")
	elseif(APPLE)
		set(ONIDRIVER_FT600_FILENAME "${ONIDRIVER_FT600_NAME}.dylib")

		# Ensure needed shared libraries are installed to the shared folder
		install(FILES "${ONIDRIVER_FT600_DIR}/${ONIDRIVER_FT600_FILENAME}"
            DESTINATION "${MAC_SHARED_FOLDER}")
	endif()
endif()
