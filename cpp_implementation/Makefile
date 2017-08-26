all: example actnetopt

example: example.cpp
	g++ -I ~/PubInv/boost_1_65_0 example.cpp -o example

actnetopt: actnetopt.cpp
	g++ -std=c++11 -I ~/PubInv/boost_1_65_0 actnetopt.cpp -I ~/PubInv/CppNumericalSolvers/include/cppoptlib   -o actnetopt
