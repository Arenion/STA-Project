all :
	gcc main.c Statemachine/statemachine.c Statemachine/navigation.c marvelmind.c/src/marvelmind.c communicationarduino.c Statemachine/map.c -o teleop.e -lm
	rm -rf /tmp/cam.sock

clean :
	rm -rf teleop.e