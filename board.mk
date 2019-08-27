# List of all the board related files.
BOARDSRC = $(CHIBIOS)/testhal/STM32/STM32F0xx/SB/board.c

# Required include directories
BOARDINC = $(CHIBIOS)/testhal/STM32/STM32F0xx/SB

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
