# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" "idf_update.cpp" "led/led_strip_encoder.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()