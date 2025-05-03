CC = g++
CFLAGS = -std=c++11 -O3
EXECUTABLE = Lab2
INPUT_FILE1 = input/input.hgr
INPUT_FILE2 = input/input2.txt.hgr
INPUT_FILE3 = input/input3.txt.hgr
OUTPUT_FILE1 = output/output1.txt
OUTPUT_FILE2 = output/output2.txt
OUTPUT_FILE3 = output/output3.txt

run: all
	./$(EXECUTABLE) $(INPUT_FILE1) $(OUTPUT_FILE1)
	./$(EXECUTABLE) $(INPUT_FILE2) $(OUTPUT_FILE2)
	./$(EXECUTABLE) $(INPUT_FILE3) $(OUTPUT_FILE3)


all: $(EXECUTABLE)

$(EXECUTABLE): main.cpp fm.o
	$(CC) $(CFLAGS) -o $(EXECUTABLE) main.cpp fm.o

fm.o: fm.cpp fm.h partition.hpp
	$(CC) $(CFLAGS) -o fm.o -c fm.cpp

clean:
	rm -rf *.o $(EXECUTABLE)
