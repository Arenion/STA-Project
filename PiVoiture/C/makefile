all :
	gcc main.c -o teleop.e
	rm -rf /tmp/cam.sock
	./teleop.e /dev/ttyAMA0 9600
	

clean :
	rm -rf teleop.e