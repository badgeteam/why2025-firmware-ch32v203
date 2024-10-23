file(GLOB APPLICATION_SOURCES
	"*.c"
)

add_executable(${PROJECT_NAME}.elf ${APPLICATION_SOURCES})

target_include_directories(${PROJECT_NAME}.elf
	PUBLIC
		.
)

target_link_libraries(${PROJECT_NAME}.elf
	PUBLIC
		platform
		hal
		freertos_kernel
		freertos_config
)

add_custom_command(
    TARGET
        ${PROJECT_NAME}.elf
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
	COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)
