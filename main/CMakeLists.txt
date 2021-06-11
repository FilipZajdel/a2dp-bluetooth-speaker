set(COMPONENT_SRCS "bt_app_av.c"
                   "bt_app_core.c"
                   "main.c")
set(COMPONENT_ADD_INCLUDEDIRS ".")

if (CONFIG_USE_MEDIA_BUTTONS)
    set(COMPONENT_SRCS ${COMPONENT_SRCS}
                        "media_control.c")
endif()

register_component()
