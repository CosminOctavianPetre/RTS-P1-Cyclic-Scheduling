add_executable(${TARGET_C})
target_sources(${TARGET_C}
		PRIVATE controladorC.c
			    displayC.c
)

# set required compiler and linker flags
target_compile_options(${TARGET_C} PRIVATE ${compiler_flags})
target_link_options(${TARGET_C} PRIVATE ${linker_flags})