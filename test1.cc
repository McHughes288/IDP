#include <iostream>
#include <robot_link.h>
#include <robot_instr.h>
#include <stopwatch.h>
#include <string>
#include <stdio.h>
#include <delay.h>

#define ROBOT_NUM 17 

using namespace std;


robot_link rlink;
stopwatch watch;

int main()
{
	int val;
	char port[8];
	int time;
	
	if(!rlink.initialise(ROBOT_NUM))
	{
		cout << "Cannot initialise link" <<endl;
		rlink.print_errs(" ");
		return -1;
	}
	watch.start();
	for(int i = 0; i < 100; i++)
	{
		val = rlink.request(READ_PORT_0);
	}
	time = watch.read() / 100;
	watch.stop();
	
	for(int i=7; i>=0; i--)
	{
   		cout<<((val>>i)&1);
	}
	cout << endl;

	sprintf(port, "%x", val);
	
	cout << "Using sprintf(hex): 0x" << port << endl;
	
	cout << "Time to read: " << time << endl;
	
	rlink.command(RAMP_TIME,1000);
	rlink.command(MOTOR_4_GO, 100);
	delay(1000);
	
	rlink.command(WRITE_PORT_0, 0xFF);
	delay(5000);
	return 1;
}
