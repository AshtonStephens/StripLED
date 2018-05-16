# Makefile
#
# Purpose: to compile autonomous flight program
# By: Ashton Stephens
# Date : 08/10/17
#

IDIRS	  = ./
SERIAL_RX = FRSKY_TELEM
SERIAL_TX = SEND7 

# # # # # # # # # # # # # # # # 
# This is where you keep your .h files

CC = clang++
IFLAGS  = $(patsubst %,-I%,$(IDIRS))
CFLAGS  = -Wall -Wextra -Werror -Wfatal-errors $(IFLAGS)
LDFLAGS = 
DBFLAGS = -g -DDEBUG
LDLIBS  = -lrt -lpthread -ldl
RM_OBJS = && rm *.o

PROTOCALLS = $(SERIAL_RX) $(SERIAL_TX)
DEF_FLAGS = $(patsubst %,-D%,$(PROTOCALLS))

%.o: %.cpp
	$(CC) $(DEF_FLAGS) $(CFLAGS) -c -o $@ $<

%.o.test: %.cpp
	$(CC) $(DEF_FLAGS) $(DBFLAGS) $(CFLAGS) -c -o $*.o $<

%.o.shared: %.cpp
	$(CC) $(DEF_FLAGS) $(CFLAGS) -c -fPIC -o $@ $<
	
%.so: %.o.shared
	$(CC) $< -shared -o $@ -ldl

# # # # # # # # # # # # # # # # 
# stuff you might have to change 
# 	|	|	|
# 	V	V	V

all: main test 

EXECUTABLES = main test  

MAIN_INCLUDES  = main.o fake_arduino.o led_pixel.o debug_macros.o
TEST_INCLUDES  = test.o binterm.o debug_macros.o

main: $(MAIN_INCLUDES)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS) $(RM_OBJS) 
main_test: $(patsubst %,%.test,$(MAIN_INCLUDES))
	$(CC) $(LDFLAGS) -o $@ $(MAIN_INCLUDES) $(LDLIBS) $(RM_OBJS)

test: $(TEST_INCLUDES)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS) $(RM_OBJS) 
test_test: $(patsubst %,%.test,$(TEST_INCLUDES))
	$(CC) $(LDFLAGS) -o $@ $(TEST_INCLUDES) $(LDLIBS) $(RM_OBJS)

clean: 
	rm *~ *.swp *.o vgcore* $(EXECUTABLES) 2> /dev/null; true
