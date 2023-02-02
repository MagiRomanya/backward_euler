
# This is a dummy makefile to call the real one in the build directory

PROJECT_DIR=~/Documents/Project/backward_euler
all:
	cd $(PROJECT_DIR)/build && make

run: all
	$(PROJECT_DIR)/build/BackwardEuler
