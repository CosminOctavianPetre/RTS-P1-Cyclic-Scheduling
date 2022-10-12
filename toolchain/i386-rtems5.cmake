set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR i386)

set(target_prefix "i386-rtems5-")
# required compiler and linker flags
set(compiler_flags 	-c
					-fmessage-length=0
					-B${RTEMS_INSTALL_PATH}/rtems-lib/i386/5/i386-rtems5/pc386/lib
					-specs bsp_specs
)
set(linker_flags 	-B${RTEMS_INSTALL_PATH}/rtems-lib/i386/5/i386-rtems5/pc386/lib
					-specs bsp_specs
					-qrtems
					-Wl,-Ttext,0x00100000
)

# set the C and C++ compilers to be RTEMS5 GCC and G++ for x86 architecture
set(CMAKE_C_COMPILER "${RTEMS_INSTALL_PATH}/compiler/i386/5/bin/${target_prefix}gcc")
set(CMAKE_CXX_COMPILER "${RTEMS_INSTALL_PATH}/compiler/i386/5/bin/${target_prefix}g++")
