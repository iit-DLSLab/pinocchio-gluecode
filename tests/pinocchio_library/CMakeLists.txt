enable_testing()

add_compile_options("-std=c++20")

find_package(pinocchio REQUIRED)

include(${CMAKE_CURRENT_SOURCE_DIR}/utils_pinocchio.cmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# PINOCCHIO CONTROLLER
#-- Build target
set(PINOCCHIO_CONTROLLER pinocchio_controller)
set(PINOCCHIO_MODEL_DIR "~/pinocchio_ws/pinocchio/models")
set(DEBUG_MSG_PATH "/usr/include/dls2/controllers/${PINOCCHIO_CONTROLLER}/debug_msg.xml")

add_library(${PINOCCHIO_CONTROLLER} SHARED
    src/${PINOCCHIO_CONTROLLER}.cpp
)
target_include_directories(${PINOCCHIO_CONTROLLER}
	PUBLIC
        include
        ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${PINOCCHIO_CONTROLLER}
    PUBLIC
    ${pinocchio_LIBRARIES}
	pinocchio_parsers
    periodic_app_plugin
	dls_messages 
	dls_topics
    robotlib
)
target_link_directories(${PINOCCHIO_CONTROLLER}
	PRIVATE
        /usr/lib/dls2
)

target_compile_definitions(${PINOCCHIO_CONTROLLER} 
	PRIVATE
		PINOCCHIO_MODEL_DIR="${PINOCCHIO_MODEL_DIR}"
		DEBUG_MSG_PATH="${DEBUG_MSG_PATH}"
	)

include(dls_message)		# message generation function
# Add idls here
dls_add_message(debug_pinocchio ${PINOCCHIO_CONTROLLER}) 	# generate message
	
	
#-- Install target
install(
	DIRECTORY   include/
	DESTINATION /usr/include/dls2
	COMPONENT   ${PINOCCHIO_CONTROLLER}_dev
)

install(
	FILES		config/scheduler.yaml
	DESTINATION /usr/include/dls2/schedulers/${PINOCCHIO_CONTROLLER}
	COMPONENT   ${PINOCCHIO_CONTROLLER}
)
install(
	TARGETS             ${PINOCCHIO_CONTROLLER}
	LIBRARY DESTINATION /usr/lib/dls2/controllers
	COMPONENT           ${PINOCCHIO_CONTROLLER}
)
