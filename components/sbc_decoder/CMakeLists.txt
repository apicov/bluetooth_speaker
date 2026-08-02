# The OI ("Open Interface") SBC decoder, vendored from
# components/bt/host/bluedroid/external/sbc/decoder in ESP-IDF v6.0.1.
#
# Vendored rather than reused in place because satellites decode SBC without
# running Bluetooth at all -- depending on the `bt` component would drag the
# whole Bluedroid stack (~950 kB and a large slice of DRAM) onto a board that
# never opens a Bluetooth connection.
file(GLOB OI_SRCS "oi/srce/*.c")
idf_component_register(SRCS "sbc_decoder.c" ${OI_SRCS}
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS "oi" "oi/srce"
                       PRIV_REQUIRES log)
