all: default

default: main.o serial.o serialmanager.o UDP_server.o A9.o
	g++ main.o serial.o serialmanager.o UDP_server.o A9.o -pthread -std=c++11 -g -o output

main.o: main.cpp
	g++ -c -g main.cpp -pthread -std=c++11

serial.o: serial.cpp
	g++ -c -g serial.cpp -pthread -std=c++11

serialmanager.o: serialmanager.cpp
	g++ -c -g serialmanager.cpp -pthread -std=c++11

UDP_server.o: UDP_server.cpp
	g++ -c -g UDP_server.cpp -pthread -std=c++11

A9.o: A9.cpp
		g++ -c -g A9.cpp -pthread -std=c++11

clean:
	rm *o output
