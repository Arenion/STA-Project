all :
	gcc main.c -o teleop.e
	rm -rf /tmp/cam.sock
	./teleop.e
	python ../Python/line_finder.py

clean :
	rm -rf teleop.e