# Required include directories
#BOARDINC = local/${BOARD}

# List of all the board related files.
#BOARDSRC = ${BOARDINC}/board.c




# List of all the board related files.
BOARDSRC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F767ZI/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/os/hal/boards/ST_NUCLEO144_F767ZI

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)

