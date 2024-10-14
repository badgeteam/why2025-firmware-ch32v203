file(GLOB PLATFORM_SOURCES
	"*.c"
)

add_library(platform
	OBJECT
		${PLATFORM_SOURCES}
)

target_include_directories(platform
	PUBLIC
		.
)

target_compile_definitions(platform
    PUBLIC
        TARGET_MCU="CH32V203"
        MCU_PACKAGE=1
        TARGET_MCU_LD=2
        TARGET_MCU_MEMORY_SPLIT=
        TARGET_MCU_PACKAGE="CH32V203C8T6"
        CH32V20x=1

)
