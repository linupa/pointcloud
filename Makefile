target = wbc
os = $(shell uname -s)
machine = $(shell uname -m)
build = $(os)_$(machine)
srcs = main.cpp xml.cpp mywindow.cpp wbcrrt.cpp kin.cpp model.cpp
objs = $(patsubst %.cpp,$(build)/%.o,$(srcs))
tinyxmlobjs = $(build)/tinyxml.o $(build)/tinystr.o 
#$(build)/tinyxmlerror.o $(build)/tinyxmlparser.o

all: $(build) $(target)


#cflags = -g -I /usr/local/include -I /usr/local/include/eigen3 -I ../uta-wbc-dreamer/wbc_core/stanford_wbc/jspace/ -I ../uta-wbc-dreamer/wbc_core/stanford_wbc/opspace/include/ $(pkg-config --cflags gl)  -I ../libs
cflags = -O3 -g -msse2 -I . -I /usr/local/include -I /usr/local/include/eigen3 -I /usr/include/eigen3 -I ../uta-wbc-dreamer/wbc_core/stanford_wbc/jspace/ -I ../uta-wbc-dreamer/wbc_core/stanford_wbc/opspace/include/ $(pkg-config --cflags gl) -I ../libs
ldflags = -O3 -g -msse2 -ltinyxml -L/usr/local/lib -Wl,-L /usr/lib -lfltk -lfltk_gl -lGLU -lGL -lglut $(pkg-config --libs gl)-L $(build) -lwbc_core -lpthread -lutils -L ../libs/$(build)
#ldflags = -O3 -g -msse2  -L/usr/local/lib -Wl,-L /usr/lib -lfltk -lfltk_gl -lGLU -lGL -lglut $(pkg-config --libs gl)-L $(build) -lpthread -lutils -L ../libs/$(build)

$(target): $(objs)
	g++ -o $@ $^ $(tinyxmlobjs) $(cflags) $(ldflags) 
#	g++ -o $@ $^ $(cflags) $(ldflags) 

$(objs): $(build)/%.o : %.cpp 
	g++ -c $< $(cflags) -o $@

$(build): 
	mkdir $(build)

clean:
	rm -f $(objs) $(target)

$(target) : rrt.hpp
