add_executable(${TARGET_A})
target_sources(${TARGET_A}
		PRIVATE controladorA.c
			    displayA.c
)

# set required compiler and linker flags
target_compile_options(${TARGET_A} PRIVATE ${compiler_flags})
target_link_options(${TARGET_A} PRIVATE ${linker_flags})