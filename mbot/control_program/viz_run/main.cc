#include <stdio.h>
#include <SerialStream.h>
#include <iostream>
#include <string.h>
#include <thread>
#include <chrono>
#include <csignal>
#include <opencv2/core/mat.hpp>
#include <unistd.h>

#include "viz.hh"
#include "grid.hh"

using cv::Mat;
using namespace LibSerial;
using namespace std;

static SerialStream port;

int i = 0;

Mat m;

Pose p = Pose();
float dist = 0.0;

bool o1 = false;
bool o2 = false;
bool o3 = false;
bool o4 = false;

void setup()
{
	port.Open("/dev/ttyUSB0");
	port.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
	port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
	port.SetParity(LibSerial::Parity::PARITY_NONE);
	port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
}

void process_data(char type, char *data)
{
	if (type == 'd')
	{
		dist = std::stof(data) / 100;
	}
	if (type == 'p')
	{
		p.t = std::stof(data);
	}
	if (type == 'x')
	{
		p.x = std::stof(data);
	}
	if (type == 'y')
	{
		p.y = std::stof(data);
	}
	if (type == '1')
	{
		if (!o1 && data[0] == '1' && dist < 1.0)
		{
			o1 = true;
			grid_apply_hit_color(dist, 0, p, "blue");
		}
	}
	if (type == '2')
	{
		if (!o2 && data[0] == '1' && dist < 1.0)
		{
			o2 = true;
			grid_apply_hit_color(dist, 0, p, "red");
		}
	}
	if (type == '3')
	{
		if (!o3 && data[0] == '1'&&  dist < 1.0)
		{
			o3 = true;
			grid_apply_hit_color(dist, 0, p, "brown");
		}
	}
	if (type == '4')
	{
		if (!o4 && data[0] == '1' && dist < 1.0)
		{
			o4 = true;
			grid_apply_hit_color(dist, 0, p, "green");
		}
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

void apply_data_on_grid()
{
	if (dist < 1.5)
	{
		grid_apply_hit(dist, 0.0, p);
	}

	m = grid_view(p, {});
}

void read_data()
{
	int i = 0;
	while (!o1 && !o2 && !o3 && !o4)
	{
		read();

		cout << "~~~~~~~~~~~~~~" << endl;
		cout << "Distance to hit " << dist << endl;
		cout << "Robot theta " << p.t;
		cout << ", ";
		cout << "x pos " << p.x;
		cout << ", ";
		cout << "y pos " << p.y << endl;
		cout << "Objct presence:" << endl;
		cout << "Object 1: " << o1 << endl;
		cout << "Object 2: " << o2 << endl;
		cout << "Object 3: " << o3 << endl;
		cout << "Object 4: " << o4 << endl;

		apply_data_on_grid();
		i++;
	}

	cout << i << endl;
}

int main(int argc, char *argv[])
{
	setup();
	read_data();
	viz_show(m);

	return viz_run(0, NULL);
}
