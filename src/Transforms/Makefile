
CXX		  := g++
CXX_FLAGS := -fPIC -03 -g -Wall -std=c++17 -Werror


NAME := Euler_to_Ra2b

lib: lib$(NAME).so

EXECUTABLE	:= Euler_to_Ra2b

lib$(NAME).so: $(NAME).o
	$(CXX) -shared $^ -o $@

clean:	
	$(RM) *.o *.so*

