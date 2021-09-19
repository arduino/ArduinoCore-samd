to add new board
change ss pin to desired pin for new board
export compiled binary
run (cat "SDU.bin" | xxd -i > nano_33_iot.h) file names are example
move new hex file to src/boot, and modify SDU.cpp with a new if else statement
should compile now and notnget unsupported board issue
