############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE modules__nn_control
	MAIN nn_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		NNControl.cpp
		NeuralNetworkInterface.cpp
		NNVerificationProtocol.hpp
	DEPENDS
		px4_work_queue
	)
