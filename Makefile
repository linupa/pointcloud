target = pointcloud
os = $(shell uname -s | sed s/$1-.*//g)
machine = $(shell uname -m)
build = $(os)_$(machine)
srcs = main.cpp mywindow.cpp Octree.cpp 
objs = $(patsubst %.cpp,$(build)/%.o,$(srcs))

all: $(build) $(target)


cflags = -O3 -g -D$(os) -I . -I /usr/local/include -I /usr/local/include/eigen3 -I /usr/include/eigen3 $(pkg-config --cflags gl) -I ../libs
ldflags = -O3 -g -msse2 -L/usr/local/lib -Wl,-L /usr/lib -lfltk -lfltk_gl -lGLU -lGL -lglut $(pkg-config --libs gl) -L $(build) -lpthread -lutils -L ../libs/$(build)

$(target): $(objs)
	g++ -o $@ $^ $(cflags) $(ldflags) 

$(objs): $(build)/%.o : %.cpp 
	g++ -c $< $(cflags) -o $@

$(build): 
	mkdir $(build)

clean:
	rm -f $(objs) $(target)
