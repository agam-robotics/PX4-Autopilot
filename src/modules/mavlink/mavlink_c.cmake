############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

# MAVLink C library generation, shared by the mavlink module and any other
# module that packs MAVLink messages (include() this file and depend on mavlink_c).

if(NOT CONFIG_MAVLINK_DIALECT)
	set(CONFIG_MAVLINK_DIALECT "common")
endif()

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
file(RELATIVE_PATH MAVLINK_GIT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_GIT_DIR})
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")

if(NOT TARGET mavlink_c)
	px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")

	# always generate uAvionix dialect
	add_custom_command(
		OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
		COMMAND
			${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
				--lang C --wire-protocol 2.0
				--output ${MAVLINK_LIBRARY_DIR}
				${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${MAVLINK_DIALECT_UAVIONIX}.log
		DEPENDS
			git_mavlink_v2
			${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml

		COMMENT "Generating Mavlink ${MAVLINK_DIALECT_UAVIONIX}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml"
	)
	add_custom_target(mavlink_c_generate_uavionix DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h)
	set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h PROPERTIES GENERATED true)

	# generate the primary dialect
	add_custom_command(
		OUTPUT ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
		COMMAND
			${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
				--lang C --wire-protocol 2.0
				--output ${MAVLINK_LIBRARY_DIR}
				${MAVLINK_GIT_DIR}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${CONFIG_MAVLINK_DIALECT}.log
		DEPENDS
			git_mavlink_v2
			${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml
			${BOARD_DEFCONFIG}

			# generate after uAvionix
			mavlink_c_generate_uavionix
			${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h

		COMMENT "Generating Mavlink ${CONFIG_MAVLINK_DIALECT}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml"
	)
	add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h)
	set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h PROPERTIES GENERATED true)

	# mavlink header only library
	add_library(mavlink_c INTERFACE)
	target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
	target_sources(mavlink_c
		INTERFACE
			${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
			${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
	)
	target_include_directories(mavlink_c
		INTERFACE
			${MAVLINK_LIBRARY_DIR}
			${MAVLINK_LIBRARY_DIR}/../
			${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
			${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
	)
endif()
