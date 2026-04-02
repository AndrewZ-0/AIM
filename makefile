all: pkf run

run: pkf.cpp
	g++ -o sim sim.cpp 

pkf: 
	g++ -o pkf pkf.cpp 

clean:
	rm -r pkf
	rm -r sim