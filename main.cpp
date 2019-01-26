#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <FL/gl.h>
#include <FL/Fl_Gl_Window.H>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

#include "Octree.h"
#include "mywindow.h"
#include "fileio.h"
#include "timestamp.h"
#include "default.h"

#define CHECK_LIMIT
#define TICK_SEC (1000)

using namespace std;


int win_width(800);
int win_height(600);
static char win_title[100];

#ifdef Linux
double dt = 0.001;
#else
double dt = 0.03;
#endif
double color_offset = 0;
double color_scale = 1.;
double grid_height = 0.;
int color_axis = 0;

#define FILE_NAME_SIZE (100)
char pcd_filename[FILE_NAME_SIZE];

Octree *octree = NULL;

int main(int argc, char *argv[])
{
	int c;
	TiXmlDocument doc;
	int loaded = 0;

	RegisterValue(win_width);
	RegisterValue(win_height);
	RegisterValue(hor);
	RegisterValue(ver);
	RegisterValue(scale);
	RegisterValue(color_offset);
	RegisterValue(color_scale);
	RegisterValue(color_axis);
	RegisterValue(grid_height);

	Default::load(argv[0]);

	while ( (c = getopt(argc, argv, "p:h") ) != -1 )
	{
		switch (c)
		{
			case 'p':
				strncpy(pcd_filename, optarg,FILE_NAME_SIZE);
				break;
			case 'h':
				printf("Usage: %s -p <PCD file name>", argv[0]);
				exit(-1);
			default:
				break;

		}
	}

	octree = new Octree;
	octree->x = octree->y = octree->z = 0.;
	octree->len = 2.;

	data_stream ds;

	if ( !ds.open(pcd_filename) )
		return -1;

	int count = 0;
	while ( ds.read_line() > 0 )
	{
		vector<double> entry = ds.get_entry();

		if ( entry.size() != 3 )
			continue;
	
		OctreeEntry *cell = new OctreeEntry;

		cell->x = entry[0];
		cell->y = entry[1];
		cell->z = -entry[2];
		octree->addEntry(cell, 10);	
		count++;
	}

	cerr << "OpenGL Init " << count << " cells are registered" << endl;
	glutInit(&argc, argv);

	MyWindow win(win_width, win_height, "test");

	int ret = Fl::run();

	Default::save();
}

