# Colorful output
# https://stackoverflow.com/questions/5947742/how-to-change-the-output-color-of-echo-in-linux

# Black        0;30     Dark Gray     1;30
# Red          0;31     Light Red     1;31
# Green        0;32     Light Green   1;32
# Brown/Orange 0;33     Yellow        1;33
# Blue         0;34     Light Blue    1;34
# Purple       0;35     Light Purple  1;35
# Cyan         0;36     Light Cyan    1;36
# Light Gray   0;37     White         1;37

BLACK=\033[0;30m
RED=\033[0;31m
GREEN=\033[0;32m
ORANGE=\033[0;33m
BLUE=\033[0;34m
PURPLE=\033[0;35m
CYAN=\033[0;36m
LGRAY=\033[0;37m
DGRAY=\033[1;30m
LRED=\033[1;31m
LGREEN=\033[1;32m
YELLOW=\033[1;33m
LBLUE=\033[1;34m
LPURPLE=\033[1;35m
LCYAN=\033[1;36m
WHITE=\033[1;37m

# High Intensity
IBLACK=\033[0;90m
IRED=\033[0;91m
IGREEN=\033[0;92m
IYELLOW=\033[0;93m
IBLUE=\033[0;94m
IPURPLE=\033[0;95m
ICYAN=\033[0;96m
IWHITE=\033[0;97m

# Bold High Intensity
BIRED=\033[1;91m

NC=\033[0m # No Color

CPP			    = g++
RM			    = rm -f
CPPFLAGS		= -Wall -I. -O2 -std=c++17 -g
LDFLAGS			= -lpthread -lzmqpp -lzmq -Wl,--no-as-needed
SOURCES			= $(wildcard *.cpp)
TARGETS			= $(SOURCES:%.cpp=%)

all:	${TARGETS}

clean:
	${RM} *.o *~* ${TARGETS}.out ${TARGETS}.d

${TARGETS}:
	${CPP} ${CPPFLAGS} -o $@.out ${@:%=%.cpp} ${LDFLAGS}

up-cpp: up-message all
	./main.out

up-message:
	@echo "${PURPLE}============="
	@echo "|    UP     |"
	@echo "=============${NC}"

init: init_cpp
	# @echo "\n${YELLOW}Don't forget to do ${WHITE}source venv/bin/activate${NC}"
	# @echo "                   ~~~~~~~~~~~~~~~~~~~~~~~~"

init_cpp:
	@echo "${GREEN}==============="
	@echo "| ZMQ INSTALL |"
	@echo "===============${NC}"

	# Build, check, and install the latest version of ZeroMQ
	git clone https://github.com/zeromq/libzmq.git && \
	cd libzmq && \
	./autogen.sh  && \
	./configure && make && \
	sudo make install && \
	sudo ldconfig


	git clone https://github.com/zeromq/zmqpp.git && \
	cd zmqpp && \
	make && \
	sudo make install
