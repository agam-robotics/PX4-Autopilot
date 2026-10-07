############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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

include(${CMAKE_CURRENT_LIST_DIR}/mavlink_c.cmake)

px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_events.cpp
		mavlink_ftp.cpp
		mavlink_ftp_param.cpp
		mavlink_log_handler.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_parameters.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_timesync.cpp
		mavlink_ulog.cpp
		mavlink_sign_control.cpp
		MavlinkStatustextHandler.cpp
		open_drone_id_translations.cpp
		tune_publisher.cpp
	MODULE_CONFIG
		module.yaml
		mavlink_params.yaml
	DEPENDS
		airspeed
		component_general_json # for checksums.h
		dataman_client
		drivers_accelerometer
		drivers_gyroscope
		drivers_magnetometer
		conversion
		gnss
		sensor_calibration
		geo
		mavlink_c
		timesync
		tunes
		variable_length_ringbuffer
		version
	UNITY_BUILD
	)

if(CONFIG_DRIVERS_SERIALPASSTHROUGH)
	target_link_libraries(modules__mavlink PRIVATE drivers__serialpassthrough)
endif()

target_link_libraries(modules__mavlink PRIVATE failure_injection)

px4_add_unit_gtest(SRC MavlinkStatustextHandlerTest.cpp
	INCLUDES
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
	COMPILE_FLAGS
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
		-Wno-cast-align # TODO: fix
	LINKLIBS
		modules__mavlink
	)

px4_add_unit_gtest(SRC MavlinkFtpParamTest.cpp EXTRA_SRCS mavlink_ftp_param.cpp)

if(CONFIG_NET AND "${PX4_PLATFORM}" MATCHES "nuttx")
	target_link_libraries(modules__mavlink PRIVATE nuttx_apps) # netlib_get_ipv4netmask
endif()
