OPT = -O3 -ffast-math

run: sim

sim: clean
	g++ -o sim pkff.cpp sim.cpp $(OPT)

sim2: clean
	g++ -o sim2 pkff.cpp sim2.cpp $(OPT)

visualise: clean
	g++ -o visualise visualise.cpp pkff.cpp \
	-I/opt/homebrew/opt/raylib/include -L/opt/homebrew/opt/raylib/lib -lraylib \
	$(OPT) \
	-framework Cocoa \
	-framework IOKit \
	-framework CoreAudio \
	-framework CoreVideo

pkff:
	g++ -o pkff pkff.cpp $(OPT) 

clean:
	rm -f pkff
	rm -f sim
	rm -f sim2
	rm -f visualise