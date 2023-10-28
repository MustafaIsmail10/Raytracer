all:
	g++ *.cpp -o raytracer -std=c++11 -O3

clean:
	rm -f raytracer

run: clean all
	./raytracer $$(cat scene.txt)
