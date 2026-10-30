############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

if(CONFIG_NN_CONTROL_BACKEND_NEURAL)
	set(controller_backend_sources
		NeuralControllerBackend.cpp
		NeuralNetworkInterface.cpp
	)
elseif(CONFIG_NN_CONTROL_BACKEND_LQR)
	set(controller_backend_sources LqrControllerBackend.cpp)
elseif(CONFIG_NN_CONTROL_BACKEND_HYBRID)
	set(controller_backend_sources HybridControllerBackend.cpp)
else()
	message(FATAL_ERROR "nn_control requires exactly one controller backend")
endif()

px4_add_module(
	MODULE modules__nn_control
	MAIN nn_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		NNControl.cpp
		ControllerBackend.hpp
		ControllerTypes.hpp
		FinalMotorOutputMap.hpp
		GroundStationGate.hpp
		LqiCoaxialCollectiveCore.hpp
		LqiCoaxialCollectiveModel.hpp
		LqiCoaxialCollectivePayload.hpp
		NNVerificationProtocol.hpp
		${controller_backend_sources}
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
	)
