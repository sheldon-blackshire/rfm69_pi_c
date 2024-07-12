build: main.o
        gcc -o rfm69 main.o -lwiringPi
run: rfm69
        ./rfm69
