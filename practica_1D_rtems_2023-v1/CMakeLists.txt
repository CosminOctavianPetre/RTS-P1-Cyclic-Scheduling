add_executable(${TARGET_D})
target_sources(${TARGET_D}
		PRIVATE controladorD.c
			    displayD.c
)

# set required compiler and linker flags
target_compile_options(${TARGET_D} PRIVATE ${compiler_flags})
target_link_options(${TARGET_D} PRIVATE ${linker_flags})