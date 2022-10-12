add_executable(${TARGET_B})
target_sources(${TARGET_B}
		PRIVATE controladorB.c
			    displayB.c
)

# set required compiler and linker flags
target_compile_options(${TARGET_B} PRIVATE ${compiler_flags})
target_link_options(${TARGET_B} PRIVATE ${linker_flags})