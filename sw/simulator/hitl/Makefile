# HITL temporary makefile
#  
CXX		:= c++
CXXFLAGS 	:= -lpthread -lrt
INCLUDES	:= -I /usr/include/boost/ -I /usr/include/JSBSim/ -I ../../airborne/ -I ../../include -I ../nps
OBJECTS		:=  
LIBS      := -lboost_thread -lboost_system /usr/lib/libJSBSim.so
SRCS		  := ../../airborne/math/pprz_geodetic_double.c ../../airborne/math/pprz_geodetic_float.c \
			   ../../airborne/math/pprz_geodetic_wmm2015.c ../../airborne/math/pprz_algebra_double.c
# 
all: main.cpp
	$(CXX) main.cpp ${SRCS} -o hitl.bin -std=c++11 ${INCLUDES} ${CXXFLAGS} ${LIBS}

clean:
	rm hitl.bin
