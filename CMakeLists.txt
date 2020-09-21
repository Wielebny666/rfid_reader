set(SOURCES rfid_reader.c )

idf_component_register(SRCS "${SOURCES}"
					INCLUDE_DIRS .
					REQUIRES 	as3933
								enum_factory
					)
