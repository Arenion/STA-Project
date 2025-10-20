all :
	gcc main.c Statemachine/statemachine.c Statemachine/navigation.c communicationarduino.c -o teleop.e -lm
	rm -rf /tmp/cam.sock

clean :
	rm -rf teleop.e