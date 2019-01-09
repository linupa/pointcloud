#include <stdio.h>
#include <math.h>
#include <iostream>
#include <Eigen/Dense>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

#include "Octree.h"
#include "mywindow.h"

using Eigen::MatrixXd;

#define CLICK_THRESHOLD 3

// States
VectorXd disp_q1;

double target_pos[2];
int point_pressed = -1;
int drawing;
double hor = 0;
double ver = 0;
double scale;
double x_org;
double y_org;
int		octree_id = 0;
bool bPath = true;
double phi;
int		control = 1.;
double speed = 0.;

// Updated in draw every 30msec
// Reset when pushed
#define TICK_SEC (33)
int seq_ui = 0;
int node_id = 0;
int sample_ui = 0;

extern Octree *octree;
extern double dt;

extern double color_offset;
extern double color_scale;
extern double grid_height;
extern int color_axis;

//MySim::MySim(int xx, int yy, int width, int height) : Fl_Widget(xx, yy, width, height, "")
MySim::MySim(int xx, int yy, int width, int height) : Fl_Gl_Window(xx, yy, width, height, "")
{
	fprintf(stderr, "Gl Window created\n");
//	mode(FL_RGB | FL_ALPHA | FL_DEPTH | FL_DOUBLE);

	firstTime = true;
}

void MySim::initializeGl(void)
{
	glClearColor( 0., 0., 0., 0.);

	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

#if 0
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	GLfloat diff[] = {1., 1., 1., 1.};
	GLfloat ambi[] = {.9, .9, .9, 1.};
	GLfloat spec[] = {0., 0., 1., 1.};	

	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 2.);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diff);
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambi);
	glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
#endif

//	glLightfv(GL_LIGHT1, GL_AMBIENT, ambi);
}

void MySim::show()
{
//	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
	firstTime = true;

	Fl_Gl_Window::show();
}

void MySim::resize(int x, int y, int w, int h)
{
	int width, height;
	width = w, height = h;
	Fl_Gl_Window::resize(x, y, width, height);
//	Fl_Widget::resize(x, y, width, height);
//	if ( !firstTime )
		glViewport(0.,0., width, height);
}
MySim::~MySim()  {
}

int MySim::handle(int e)
{
	int ret = 0;
	int i;
	double posX, posY;
	static double x_pressed, y_pressed;
	static double org_hor, org_ver;

	posX = Fl::event_x();
	posY =  Fl::event_y();


	switch (e)
	{
	case FL_PUSH:
		x_pressed = posX;
		y_pressed = posY;
		org_hor = hor;
		org_ver = ver;
		target_pos[0] = (posX - x_org) / (scale+0.1);
		target_pos[1] = (y_org - posY) / (scale+0.1);
		point_pressed = -1;
		for ( i = 0 ; i < 4 ; i++ )
		{
			double diffx, diffy;
//				diffx = target_pos[0] - myModel.p(i,0);
//				diffy = target_pos[1] - myModel.p(i,1);
			if ( diffx*diffx + diffy*diffy < CLICK_THRESHOLD*CLICK_THRESHOLD/scale/scale )
			{
				point_pressed = i;
				break;
			}
		}
//		fprintf(stderr, "PUSH EVENT!!! (%d)(%f,%f)\n", 
//			point_pressed, target_pos[0], target_pos[1]);
		ret = 1;
		break;
	case FL_DRAG:
//		fprintf(stderr, "DRAG EVENT!!! (%f,%f) -> (%f,%f)\n", x_pressed, y_pressed, posX, posY);
//		if ( point_pressed >= 0 )
		{
//				myModel.p(point_pressed, 0) = (posX - x_org) / scale;
//				myModel.p(point_pressed, 1) = (y_org - posY) / scale;
			hor = org_hor - (posX - x_pressed) * 0.01;
			ver = org_ver + (posY - y_pressed) * 0.01;

			if ( ver > M_PI/2. )
				ver = M_PI/2.;
			else if ( ver < -M_PI/2. )
				ver = -M_PI/2.;
		}
		
		break;
	case FL_RELEASE:
//		fprintf(stderr, "RELEASE EVENT!!!\n");
		point_pressed = -1;
		break;
	case FL_ENTER:
		fprintf(stderr, "FOCUS EVENT!!!\n");
		ret = 1;
		break;
	case FL_MOUSEWHEEL:
		fprintf(stderr, "MOUSE WHEEL!!!\n");
		scale += Fl::event_dy()/10.;

		if ( scale < 0. )
			scale = 0.;
		ret = 1;
		break;
	}
	Fl_Gl_Window::handle(e);
//		Fl_Widget::handle(e);

	return ret;
}

// OpenGL Drawing
#define POLYGON_NUM (8)
void drawBlock(float width, float height)
{
	float h_w = width/2.;
	glBegin(GL_QUADS);

	glNormal3d(0,0,-1);
	glVertex3f(-h_w, -h_w, 0);
	glVertex3f(-h_w, h_w, 0);
	glVertex3f(h_w, h_w, 0);
	glVertex3f(h_w, -h_w, 0);

	glNormal3d(0,0,1);
	glVertex3f(-h_w, -h_w, height);
	glVertex3f(h_w, -h_w, height);
	glVertex3f(h_w, h_w, height);
	glVertex3f(-h_w, h_w, height);

	glNormal3d(0,1,0);
	glVertex3f(-h_w, h_w, 0);
	glVertex3f(-h_w, h_w, height);
	glVertex3f(h_w, h_w, height);
	glVertex3f(h_w, h_w, 0);

	glNormal3d(0,-1,0);
	glVertex3f(-h_w, -h_w, 0);
	glVertex3f(h_w, -h_w, 0);
	glVertex3f(h_w, -h_w, height);
	glVertex3f(-h_w, -h_w, height);

	glNormal3d(1,0,0);
	glVertex3f(h_w, -h_w, 0);
	glVertex3f(h_w, h_w, 0);
	glVertex3f(h_w, h_w, height);
	glVertex3f(h_w, -h_w, height);

	glNormal3d(-1,0,0);
	glVertex3f(-h_w, -h_w, 0);
	glVertex3f(-h_w, -h_w, height);
	glVertex3f(-h_w, h_w, height);
	glVertex3f(-h_w, h_w, 0);

	glEnd();
}

void drawCapsule(double *pos1, double *pos2, double radius)
{
	double diff[3];
	double axis[3];
	GLUquadricObj *quadratic;
	quadratic = gluNewQuadric();

	diff[0] = pos2[0] - pos1[0];
	diff[1] = pos2[1] - pos1[1];
	diff[2] = pos2[2] - pos1[2];

	axis[0] = - diff[1] * 1.;
	axis[1] = 1. * diff[0];
	axis[2] = 0.;

	double length = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
	double costh =  diff[2] / length;
	double th;
	
	if ( costh > -0.9999 )
		th = acos(costh);
	else
	{
		th = M_PI;
		axis[0] = 1.;
		axis[1] = 0.;
		axis[2] = 0.;
	}

	glPushMatrix();
	glTranslatef(pos1[0], pos1[1], pos1[2]);
	glRotatef(th * 180. / M_PI , axis[0], axis[1], axis[2] );
	gluCylinder(quadratic, radius, radius, length, POLYGON_NUM, POLYGON_NUM);
	glutSolidSphere(radius, POLYGON_NUM, POLYGON_NUM);
	glTranslatef(0., 0., length);
	double plane[] = {1., 0., 0., 1.};
	glClipPlane(GL_CLIP_PLANE0, plane);
	glutSolidSphere(radius, POLYGON_NUM, POLYGON_NUM);
	glPopMatrix();
}

void drawCapsule(const Vector3d &from, const Vector3d &to, double radius)
{
	double _from[3];
	double _to[3];

	for ( int i = 0 ; i < 3 ; i++ )
	{
		_from[i] = from[i];
		_to[i] = to[i];
	}

	drawCapsule( _from, _to, radius );
}


void drawCylinder(double *pos1, double *pos2, double radius)
{
	double diff[3];
	double axis[3];
	GLUquadricObj *quadratic;
	quadratic = gluNewQuadric();

	diff[0] = pos2[0] - pos1[0];
	diff[1] = pos2[1] - pos1[1];
	diff[2] = pos2[2] - pos1[2];

	axis[0] = - diff[1] * 1.;
	axis[1] = 1. * diff[0];
	axis[2] = 0.;

	double length = sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
	double costh =  diff[2] / length;
	double th;
	
	if ( costh > -0.9999 )
		th = acos(costh);
	else
	{
		th = M_PI;
		axis[0] = 1.;
		axis[1] = 0.;
		axis[2] = 0.;
	}

	glPushMatrix();
	glTranslatef(pos1[0], pos1[1], pos1[2]);
	glRotatef(th * 180. / M_PI , axis[0], axis[1], axis[2] );
	gluCylinder(quadratic, radius, radius, length, 32, 32);
	glPopMatrix();
}

void drawCylinder(const VectorXd &from, const VectorXd &to, double radius)
{
	double _from[3];
	double _to[3];

	for ( int i = 0 ; i < 3 ; i++ )
	{
		_from[i] = from[i];
		_to[i] = to[i];
	}

	drawCylinder( _from, _to, radius );
}


void drawSphere(const double *pos, double radius, int numPoly)
{
	glPushMatrix();
	glTranslatef(pos[0], pos[1], pos[2]);
	glutSolidSphere(radius, numPoly, numPoly);
	glPopMatrix();
}

void drawSphere(const Vector3d  &pos, double radius, int numPoly)
{
	double _pos[3];

	for ( int i = 0 ; i < 3 ; i++ )
	{
		_pos[i] = pos[i];
	}

	drawSphere(_pos, radius, numPoly);
}

void getColor(float value, float *color)
{
	float ref[6][3] = { 
		{1.,0., 0.},
		{1.,1., 0.},
		{0.,1., 0.},
		{0.,1., 1.},
		{0.,0., 1.},
		{1.,0., 1.},
	};

	if ( value <= 0 )
	{
		memcpy(color, ref[0], sizeof(float)*3);
		return;
	}
	if ( value >= 1. )
	{
		memcpy(color, ref[5], sizeof(float)*3);
		return;
	}

	int idx = (int)(value * 5.);
	float frac = (float)value * 5. - (float)idx;

	for ( int i = 0 ; i < 3 ; i++ )
		color[i] = ref[idx][i]*(1.-frac) + ref[idx+1][i]*frac;

	return;
}

void drawOccupancy(Octree *tree)
{
	OctreeEntry *entry = tree->entry;

	if ( entry == NULL )
		return;

	float color[3];
	float value;
	switch (color_axis)
	{
	case 0:
		value = tree->x;
		break;
	case 1:
		value = tree->y;
		break;
	default:
		value = tree->z;
		break;
	}
	getColor((value+color_offset)*color_scale, color);
	glColor3f(color[0], color[1], color[2]);
	glPushMatrix();
	glTranslatef(tree->x, tree->y, tree->z);
	drawBlock(tree->len*2, tree->len*2);
	glPopMatrix();
}




#define X(x) (x_org + (x)*scale)
#define Y(y) (y_org - (y)*scale)
  void MySim::draw()
  {
	int i, j;
	double xx, yy, zz;
//	xx = myModel.getX();
//	zz = myModel.getZ();
	MatrixXd p[4];
	double curv;
	GLfloat red[] = {1., 0., 0., 1.};
	GLfloat white[] = {1., 1., 1., 1.};

#if 0
    if (w() > h()) {
      scale = h() / 12.0;
    }
    else {
      scale = w() / 12.0;
    }
#endif

	double width, height;

	width = w();
	height = h();
    x_org = width / 4.0;
    y_org = height / 4.0;

	if (firstTime)
	{
		fprintf(stderr, "GL Initialized\n");
		initializeGl();
		firstTime = false;
	}// if

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      // clear the color and depth buffer

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(-1, 1, -height/width, height/width, 1, 10);
//	glFrustum(-2, 2, -2, 2, 0, 4);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	
	gluLookAt((scale+1)*cos(hor)*cos(ver), (scale+1)*sin(hor)*cos(ver), (scale+1)*sin(ver), 0, 0, 0, 0,0,1);
//	gluLookAt((scale+4), 0, 1., 0, 0, 0, 1./(scale+4), 0, 1);

	GLfloat lightPos[] = {1.0, 0., 1., 0.};
	GLfloat lightDir[] = {1.0, 0., 0.};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glLineWidth(1.0);
//	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, lightDir);

	// Draw a sphere on the center of the coordinate
#if 0
	glPushMatrix();
//	glRotatef(thdeg[0], 0., 0., 1.);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, white);
	glColor3f(1., 1., 1.);
//	drawBlock(0.05, 0.05);
	glutSolidSphere(0.025,20,20);
	glPopMatrix();
#endif



	double color[18][3] = { 
		{1.,0., 0.},
		{0.,1., 0.},
		{0.,0., 1.},
		{1.,1., 0.},
		{1.,0., 1.},
		{0.,1., 1.},
		{1.,0.5, 0.5},
		{0.5,1., 0.5},
		{0.5,1., 0.5},
		{0.5,1., 0.5},
		{0.5,1., 0.5},
		{0.5,1., 0.5},
		{0.5,1., 0.5},
		{1.,0., 0.},
		{0.,1., 0.},
		{0.,0., 1.},
		{1.,1., 0.},
		{1.,0., 1.},
	};

#if 1
	glLineWidth(1.0);
	glColor3f(color[0][0], color[0][1], color[0][2]);

	{
		glLineWidth(1.0);
		octree->callBack(drawOccupancy);
	}
#endif

	// Floor 
	glLineWidth(1.0);
	glPushMatrix();
	glRotatef(phi*180./M_PI, 0., 0., 1.);
	double h = grid_height;
	for ( i = -10 ; i <= 10 ; i++ )
	{
		glLineWidth(1.0);
		glColor3f(.3, .3, .3);

		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(i*0.1, -1., h);
		glVertex3f(i*0.1, 1., h);
		glEnd();
		glPopMatrix();

		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(-1., i*0.1, h);
		glVertex3f(1., i*0.1, h);
		glEnd();
		glPopMatrix();
	}
	glPopMatrix();

    glCullFace(GL_BACK);
  }


MyWindow:: 
MyWindow(int width, int height, const char * title) 
    : Fl_Double_Window(width, height, title) 
{ 
	fprintf(stderr, "Window created\n");
    Fl::visual(FL_DOUBLE|FL_INDEX); 
    begin(); 
    sim = new MySim(0, 0, width, height - 40); 
//	stat->hide();

    planButton = new Fl_Button(5, height - 35, 100, 30, "&Plan"); 
    planButton->callback(cb_plan, this); 
    randomButton = new Fl_Button(5, height - 35, 100, 30, "&Random"); 
    randomButton->callback(cb_random, this); 
    modeButton = new Fl_Button(5, height - 35, 100, 30, "&Reset"); 
    modeButton->callback(cb_reset, this); 
    pathButton = new Fl_Button(5, height - 35, 100, 30, "&Path"); 
    pathButton->callback(cb_path, this); 
    learnBaseButton = new Fl_Button(5, height - 35, 100, 30, "&Simul"); 
    learnBaseButton->callback(cb_simul, this); 
    sendButton = new Fl_Button(5, height - 35, 100, 30, "&Send"); 
    sendButton->callback(cb_send, this); 
    pushXButton = new Fl_Button(5, height - 35, 100, 30, "&X"); 
    pushXButton->callback(cb_pushx, this); 
    pushYButton = new Fl_Button(5, height - 35, 100, 30, "&Y"); 
    pushYButton->callback(cb_pushy, this); 
    pushZButton = new Fl_Button(5, height - 35, 100, 30, "&Z"); 
    pushZButton->callback(cb_pushz, this); 
    goalButton = new Fl_Button(5, height - 35, 100, 30, "&Goal"); 
    goalButton->callback(cb_goal, this); 
    interveneButton = new Fl_Button(5, height - 35, 100, 30, "&Intervene"); 
    interveneButton->callback(cb_intervene, this); 

	printf("WIN: %p\n", this);
    quitButton = new Fl_Button(width - 105, height - 35, 100, 30, "&Quit"); 
    quitButton->callback(cb_quit, this); 

	mGroupSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mGroupSlider->type(FL_HORIZONTAL);
	mGroupSlider->bounds(-1., (double)1.);
	mGroupSlider->callback(cb_group, this);
	mGroupSlider->step(0.01);
	mGroupSlider->value(color_offset);

	mSizeSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mSizeSlider->type(FL_HORIZONTAL);
	mSizeSlider->bounds(1., 400.);
	mSizeSlider->callback(cb_link2, this);
	mSizeSlider->step(1.);
	mSizeSlider->value(0.); //window_size);

	mSliceSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mSliceSlider->type(FL_HORIZONTAL);
	mSliceSlider->bounds(-1., 1.);
	mSliceSlider->callback(cb_control, this);
	mSliceSlider->step(.01);
	mSliceSlider->value(grid_height);

	mScaleSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mScaleSlider->type(FL_HORIZONTAL);
	mScaleSlider->bounds(-0.01, 0.01);
	mScaleSlider->step(0.001);
	mScaleSlider->callback(cb_speed, this);
	mScaleSlider->value(0);

	mNodeSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mNodeSlider->type(FL_HORIZONTAL);
	mNodeSlider->bounds(0., 100.);
	mNodeSlider->step(1.0);
	mNodeSlider->callback(cb_node, this);
	mNodeSlider->value(0);

	mSeqSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mSeqSlider->type(FL_HORIZONTAL);
	mSeqSlider->bounds(0.1, 10.);
	mSeqSlider->step(0.1);
	mSeqSlider->callback(cb_seq, this);
	mSeqSlider->value(color_scale);

	mHighlightSlider = new Fl_Value_Slider( 10, height - 35, width / 4 - 100, 30, "");
	mHighlightSlider->type(FL_VERTICAL);
	mHighlightSlider->bounds(0., (double)1080.-1.);
	mHighlightSlider->callback(cb_highlight, this);
	mHighlightSlider->step(1.);
	mHighlightSlider->value(0);

    end(); 
    resizable(this); 
	resize(0,0,width, height);
    show(); 

	Fl::add_timeout(dt, timer_cb, this);
}

void MyWindow::
resize(int x, int y, int w, int h)
{

	Fl_Double_Window::resize(x, y, w, h);
	sim->resize(0, 0, w-70, h-110);
	mSliceSlider->resize(	10, 				h-35,	(w-375)/2 - 5,25);
	mScaleSlider->resize(	10 + (w-375)/2 - 5,	h-35,	(w-375)/2 - 5,25);
	mSeqSlider->resize(		10, 				h-70,	(w-375)/2 - 5,25);
	mNodeSlider->resize(	10 + (w-375)/2 - 5,	h-70,	(w-375)/2 - 5,25);
	mGroupSlider->resize(	10, 				h-105,	(w-375)/2 - 5,25);
	mSizeSlider->resize(	10 + (w-375)/2 - 5,	h-105,	(w-375)/2 - 5,25);

	mHighlightSlider->resize(w-40, 10, 30, h-60);

	goalButton->resize(		w-375,	h - 105, 70, 25);
	interveneButton->resize(		w-375,	h - 70, 70, 25);
	pushXButton->resize(	w-300,	h - 105, 70, 25);
	pushYButton->resize(	w-300,	h - 70, 70, 25);
	pushZButton->resize(	w-300,	h - 35, 70, 25);
	randomButton->resize(	w-225,	h - 105, 70, 25);
	planButton->resize(		w-225,	h - 70, 70, 25);
	modeButton->resize(		w-225,	h - 35, 70, 25);
	sendButton->resize(		w-150,	h - 105, 70, 25);
	pathButton->resize(		w-150,	h - 70, 70, 25);
	learnBaseButton->resize(w-150,	h - 35, 70, 25);
	quitButton->resize(		w-75,	h - 35, 70, 25);

	win_width	= w;
	win_height	= h;
}

extern VectorXd Mins;
extern VectorXd Maxs;

void MyWindow::
timer_cb(void * param)
{
	const double scr_dt = 0.03;
	static double acc = 0.;

	if ( acc >= scr_dt )
	{

		reinterpret_cast<MyWindow*>(param)->sim->redraw();
		acc = 0.;
	}
	acc += dt;

	Fl::repeat_timeout(dt, // gets initialized within tickCount()
			   timer_cb,
			   param);

}

void MyWindow::
cb_simul(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_quit(Fl_Widget *widget, void *param)
{
	reinterpret_cast<MyWindow*>(param)->hide();
}

void MyWindow::
cb_control(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	grid_height = pSlider->value();
}

void MyWindow::
cb_speed(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	speed = pSlider->value();
}

void MyWindow::
cb_group(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	color_offset = pSlider->value();
}

void MyWindow::
cb_highlight(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	sample_ui = (int)pSlider->value();
}


void MyWindow::
cb_reset(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_plan(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_path(Fl_Widget *widget, void *param)
{

	bPath = !bPath;
}

void MyWindow::
cb_random(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_send(Fl_Widget *widget, void *param)
{
//	tickCount = 0;
}

int push_type = 0;

void MyWindow::
cb_pushx(Fl_Widget *widget, void *param)
{
	color_axis = 0;
}
void MyWindow::
cb_pushy(Fl_Widget *widget, void *param)
{
	color_axis = 1;
}
void MyWindow::
cb_pushz(Fl_Widget *widget, void *param)
{
	color_axis = 2;
}

void MyWindow::
cb_seq(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	
	color_scale = pSlider->value();
}

void MyWindow::
cb_link2(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_node(Fl_Widget *widget, void *param)
{
}

double goals[][3] =
{
{0.167017, -0.141333,  0.440087},
{0.253526, -0.0720188,   0.230207},
{0.242192, -0.243455,  0.448451},
{-0.0480875, -0.265047,  0.337994},
{-0.0969532, -0.231932,  0.419739},
{0.298594, -0.243733,  0.485369}
};

void MyWindow::
cb_goal(Fl_Widget *widget, void *param)
{
}

void MyWindow::
cb_intervene(Fl_Widget *widget, void *param)
{
}

MyWindow::~MyWindow(void)
{
	Fl::remove_timeout(timer_cb, this);
}
