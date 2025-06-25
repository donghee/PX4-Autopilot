############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

add_subdirectory(px4_daemon)

set(EXTRA_DEPENDS)

# Placeholder
set(VXWORKS_LAYER_SRCS
	px4_vxworks_impl.cpp
	tasks.cpp
	px4_sem.cpp
	px4_init.cpp
	# lib_crc32.c
	drv_hrt.cpp
	cpuload.cpp
	print_load.cpp
	${PX4_SOURCE_DIR}/platforms/common/Serial.cpp
	SerialImpl.cpp
)

add_library(px4_layer
	${PX4_SOURCE_DIR}/platforms/vxworks/unresolved_symbols.c
	${VXWORKS_LAYER_SRCS}
)

target_compile_definitions(px4_layer PRIVATE MODULE_NAME="px4")

target_link_libraries(px4_layer PRIVATE work_queue)
# target_link_libraries(px4_layer PRIVATE drivers_board)
target_link_libraries(px4_layer PRIVATE work_queue px4_work_queue)
target_link_libraries(px4_layer PRIVATE px4_daemon drivers_board)