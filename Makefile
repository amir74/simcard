all: main.c
	$(CXX) $(CXXFLAGS) $(LDFAGS) -std=c++11 -o simcard $^ $(LDLIBS) -lrt -luci -lgpiod

clean:
	$(RM) -rf simcard *.o

