Header header

# contorller gpio module state
int16 state

# controller gpio module error code
int16 code

# input_digitals[0]: digital input functional gpio state
# input_digitals[1]: digital input configuring gpio state
#    CI0: (input_digitals[1] >> 0) & 0x0001
#    CI1: (input_digitals[1] >> 1) & 0x0001
#    CI7: (input_digitals[1] >> 7) & 0x0001
#    DI0: (input_digitals[1] >> 8) & 0x0001
#    DI1: (input_digitals[1] >> 9) & 0x0001
#    DI7: (input_digitals[1] >> 15) & 0x0001
uint16[2] input_digitals

# output_digitals[0]: digital output functional gpio state
# output_digitals[1]: digital output configuring gpio state
#    CO0: (output_digitals[1] >> 0) & 0x0001
#    CO1: (output_digitals[1] >> 1) & 0x0001
#    CO7: (output_digitals[1] >> 7) & 0x0001
#    DO0: (output_digitals[1] >> 8) & 0x0001
#    DO1: (output_digitals[1] >> 9) & 0x0001
#    DO7: (output_digitals[1] >> 15) & 0x0001
uint16[2] output_digitals

# input_analogs[0]: the value of AI0
# input_analogs[1]: the value of AI1
float32[2] input_analogs

# output_analogs[0]: the value of AO0
# output_analogs[1]: the value of AO1
float32[2] output_analogs

# digital input functional info
uint8[16] input_conf

# digital output functional info
uint8[16] output_conf