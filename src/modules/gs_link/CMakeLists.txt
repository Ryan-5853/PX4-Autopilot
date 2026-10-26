############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__gs_link
	MAIN gs_link
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		GsLink.cpp
		GsLink.hpp
		GsLinkProtocol.hpp
	DEPENDS
		px4_work_queue
	)
