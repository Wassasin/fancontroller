idf_component_register(
    SRCS
        "app_main.c"
        "adc.c"
        "data.c"
        "events.c"
        "fans.c"
        "http_server.c"
        "i2c_bus.c"
        "led.c"
        "mqtt.c"
        "performance.c"
        "periodic.c"
        "power.c"
        "tacho.c"
        "temperature.c"
        "wifi.c"

        "driver/shtc3.c"

        "driver/fusb302b-wrapper.cpp"

        "../contrib/usb-pd/src/fusb302b.cpp"
        "../contrib/usb-pd/src/policy_engine.cpp"
        "../contrib/usb-pd/src/policy_engine_states.cpp"
    INCLUDE_DIRS "."
    PRIV_INCLUDE_DIRS "../contrib/usb-pd/include")
