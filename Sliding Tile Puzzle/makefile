CXX=g++ -g -Ofast
all: bae

#astar: astar.cpp astar.h optimalSearchDefinition.h stp.h $(CXX) astar.cpp optimal.cpp -o astar_
	
#meet: meet.cpp meet.h optimalSearchDefinition.h stp.h  $(CXX) meet.cpp optimal.cpp -o meet

#mm: mm.cpp mm.h optimalSearchDefinition.h stp.h  $(CXX) mm.cpp optimal.cpp -o mm

bae: bae.cpp bae.h optimalSearchDefinition.h stp.h  
	$(CXX) bae.cpp optimal.cpp -o bae

clean:
	rm -f *.o bae
