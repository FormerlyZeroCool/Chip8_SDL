#
# Makefile for conway/SDL2
#

CC  = g++
CXX = g++

INCLUDE = -I/usr/local/include/SDL 
CFLAGS   = 
CXXFLAGS = -O3 -std=c++2a $(INCLUDE) 

LDFLAGS = -L/usr/local/lib -lSDLmain -lSDL2 -Wl,-framework,Cocoa -fsanitize=address,undefined
default: chip8 
$@: $@.cpp
	g++ -o $@ $@.cpp -O3 $(LDFLAGS) $(CXXFLAGS)
