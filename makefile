CC = g++
FLAGS = -pthread -Werror -Wall
LINKER = -lgtest_main
TARGETS = tests

APPDIR = src
APPSOURCES = $(APPDIR)/command.cpp

TESTDIR = tests
TESTSOURCES = $(TESTDIR)/CommandTests.cpp

INCDIR = -I $(APPDIR) -I $(TESTDIR)

.PHONY: all

all: $(TARGETS)

tests : $(TESTSOURCES) $(APPSOURCES) $(LIBS)
	$(CC) $(FLAGS) $(INCDIR) -o $@ $(TESTSOURCES) $(APPSOURCES) $(LINKER)
