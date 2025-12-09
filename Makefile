# Project Name
TARGET = chordbob

# Sources
CPP_SOURCES = chordbob.cpp

# Library Locations
LIBDAISY_DIR = ../DaisyExamples/libDaisy/
DAISYSP_DIR = ../DaisyExamples/DaisySP/

USE_DAISYSP_LGPL = 1
CPP_STANDARD = -std=c++20 -Wno-volatile

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
