# This Makefile assumes that GLFW is installed via Homebrew.
# If your setup is different, you will need to set GLFWROOT manually.

# This Makefile also assumes that MuJoCo.app is present in /Applications.

GLFWROOT?=$(shell brew --prefix)
MUJOCOPATH?=/Applications/MuJoCo.app/Contents/Frameworks

CFLAGS=-O2 -F$(MUJOCOPATH) -I$(GLFWROOT)/include -pthread
CXXFLAGS=$(CFLAGS) -std=c++17 -stdlib=libc++
OSQPFLAGS= -losqp
ALLFLAGS=$(CXXFLAGS) -L$(GLFWROOT)/lib -Wl,-rpath,$(MUJOCOPATH) $(OSQPFLAGS) -rpath /usr/local/lib

.PHONY: all
all:
	clang++ $(ALLFLAGS)    testspeed.cc  -framework mujoco        -o testspeed
	clang++ $(ALLFLAGS)    compile.cc    -framework mujoco        -o compile
	clang++ $(ALLFLAGS)    basic.cc      -framework mujoco -lglfw -o basic
	clang++ $(ALLFLAGS)    basic_gravity.cc -framework mujoco -lglfw -o basic_gravity $(shell pkg-config --cflags --libs pinocchio)
	clang++ $(ALLFLAGS)    record.cc     -framework mujoco -lglfw -o record
	clang++ $(ALLFLAGS)    hello.cc     -framework mujoco -lglfw -o hello







	