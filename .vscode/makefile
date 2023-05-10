#source file search paths
VPATH = ./src ../pathfinder/src \

	# compiler include search paths
INCS = -I../pathfinder/src \


LIBS = \
	-lgdiplus -lgdi32 -lcomdlg32 -lstdc++fs 

static_link_flags = -static-libstdc++ -static-libgcc -static -lComctl32

# folders
ODIR = ./.vscode/obj

OPT = -g
CCflags = -std=c++17 $(OPT) -D_mingw_

# sources
OBJ_starter = \
	$(ODIR)/main.o \
	$(ODIR)/cGraph.o \
	$(ODIR)/GraphTheory.o \
	$(ODIR)/readfile.o \

### COMPILE

$(ODIR)/%.o: %.cpp  
	g++  $(CCflags) \
	-c -o $@ $< $(INCS)

starter: $(OBJ_starter)
	g++ $(OPT) \
	-o bin/starter.exe $^  \
	$(static_link_flags) $(LIBS)

.PHONY: clean
clean:
	del /q .vscode\\obj\\*.o

