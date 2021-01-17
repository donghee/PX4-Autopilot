px4_add_module(
	MODULE drivers__crypto__dim
	MAIN dim
	COMPILE_FLAGS
    STACK_MAIN 4096
	SRCS
		DIM.cpp
		DIM.hpp

		dim_main.cpp
	DEPENDS
		px4_work_queue
	)
