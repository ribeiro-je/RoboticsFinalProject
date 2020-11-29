#include <stdio.h>
#include <SerialStream.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <opencv2/core/mat.hpp>

#include "viz.hh"
#include "grid.hh"

using cv::Mat;
using namespace LibSerial;
using namespace std;

static SerialStream port;

char dist[50];
bool o1 = false;
bool o2 = false;
bool o3 = false;
bool o4 = false;

Pose p;
Mat m;

void setup()
{
	port.Open("/dev/ttyUSB0");
	port.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
	port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
	port.SetParity(LibSerial::Parity::PARITY_NONE);
	port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);

	viz_run(0, NULL);
}

void process_data(char type, char *data)
{
	if (type == 'd')
	{
		strcpy(dist, data);
	}
	if (type == '1')
	{
		data[0] == '1' ? o1 = true : o1 = false;
	}
	if (type == '2')
	{
		data[0] == '1' ? o2 = true : o2 = false;
	}
	if (type == '3')
	{
		data[0] == '1' ? o3 = true : o3 = false;
	}
	if (type == '4')
	{
		data[0] == '1' ? o4 = true : o4 = false;
	}
}

void read()
{
	char cur_value[1];
	char temp[50];
	int ii = 0;

	port.read(cur_value, 1);

	for (; ii < 96; ++ii)
	{
		port.read(temp + ii, 1);
		if (temp[ii] == '\n')
		{
			temp[ii + 1] = 0;
			process_data(cur_value[0], temp);
			break;
		}
	}
}

int main(int argc, char *argv[])
{

	setup();
	while (true)
	{
		read();
		// grid_apply_hit(dist, 0, p);
		// m = grid_view(p, NULL);
		// viz_show(m);
		cout << dist << endl;
		cout << o1 << endl;
		cout << o2 << endl;
		cout << o3 << endl;
		cout << o4 << endl;
	}
}