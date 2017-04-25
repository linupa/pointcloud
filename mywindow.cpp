#include <stdio.h>
#include <math.h>
#include <iostream>

#include <jspace/Model.hpp>
#include <jspace/State.hpp>
#include <jspace/test/sai_util.hpp>
#include <jspace/pseudo_inverse.hpp>

#include <boost/scoped_ptr.hpp>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/fl_draw.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Value_Slider.H>

#include "mywindow.h"
//#include "xml.h"
#include "kin.h"
#include "prm.hpp"
#include "model.h"

using Eigen::MatrixXd;

#include "wbcrrt.h"
#define CLICK_THRESHOLD 3


extern Vector3d gGoal;
extern KinModel myModel;
double target_pos[2];
int point_pressed = -1;
int drawing;
double hor = 0;
double ver = 0;

extern int drawn;
static double dt = 0.001;
double x_org;
double y_org;
double scale;
extern int	group_threshold;
extern bool gTrack;
extern bool gShowBase;
bool bPath = true;
extern bool bRandom;
extern int goalIdx;
extern vector<VectorXd> elbow_log;
extern vector<VectorXd> elbow_des_log;
extern vector<double>   time_log;
extern int trackingGroupIndex;
extern int trackingGroupId;
extern double trackPos[3];
extern int slice_idx;
extern double gTorque[3];
double phi;
VectorXd q;
VectorXd q0;
vector<VectorXd> r0;

vector<VectorXd> qp1;
vector<VectorXd> qp2;
extern vector<double> eta;
int		control = 1.;
double speed = 0.;

extern bool bSimul;
extern bool bPlan;
extern bool bPlanning;
extern bool bSend;
//extern XmlNode baseNode;
extern void periodicTask(void);
extern pthread_mutex_t	mutex;
extern pthread_mutex_t	link_mutex;
extern bool pushing;

// Updated in draw every 30msec
// Reset when pushed
#define TICK_SEC (33)
int tickCount = 0;
double seq; 
int seq_ui = 0;
int node_id = 0;

//extern vector<Link>	myLocalLink;
//extern vector<Link>	myGlobalLink;
//extern WbcRRT *rrt;
extern PRM<9> *prm;
double *value;
double getPotential(int mode, const VectorXd &contact, const WbcNode &node, const vector<VectorXd> &r0);
VectorXd getQ(const VectorXd &uq);
VectorXd getQa(const VectorXd &q);
extern VectorXd elbow0;
extern VectorXd elbow;
extern VectorXd endeffector;
extern void push(VectorXd dir);
extern void intervention(void);

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
	gluCylinder(quadratic, radius, radius, length, 32, 32);
	glutSolidSphere(radius, 32, 32);
	glTranslatef(0., 0., length);
	double plane[] = {1., 0., 0., 1.};
	glClipPlane(GL_CLIP_PLANE0, plane);
	glutSolidSphere(radius, 32, 32);
	glPopMatrix();
}

void drawCapsule(const VectorXd &from, const VectorXd &to, double radius)
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

void drawSphere(const VectorXd  &pos, double radius, int numPoly)
{
	double _pos[3];

	for ( int i = 0 ; i < 3 ; i++ )
	{
		_pos[i] = pos[i];
	}

	drawSphere(_pos, radius, numPoly);
}




extern double _center;
extern double	avg[2];
extern double	min_pos[2], max_pos[2];

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



	double color[15][3] = { 
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
		{0.5,1., 0.5}
	};

	int index = 0;
//	XmlNode *pNode = &baseNode;
//	XmlNode *pNode = baseNode.childs;

	// Display Limbs with Cylinders of the Current State
#if 0
	glPushMatrix();
	do {
		glColor3f(color[index][0], color[index][1], color[index][2]);

		glLineWidth(3.0);

#if 0
		glBegin(GL_LINES);
		glVertex3f(0., 0., 0.);
		glVertex3f(pNode->pos(0), pNode->pos(1), 0.);
		glVertex3f(pNode->pos(0), pNode->pos(1), pNode->pos(2));
		glEnd();
#else
		GLUquadricObj *quadratic;

		quadratic = gluNewQuadric();
		glPushMatrix();
		glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
		gluCylinder(quadratic,0.05f,0.05f,pNode->pos(1),32,32);
		glPopMatrix();

		quadratic = gluNewQuadric();
		glPushMatrix();
		glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
		gluCylinder(quadratic,0.05f,0.05f,pNode->pos(0),32,32);
		glPopMatrix();

		quadratic = gluNewQuadric();
		glPushMatrix();
		glRotatef(-90.0f, 0.0f, 0.0f, 1.0f);
		gluCylinder(quadratic,0.05f,0.05f,pNode->pos(2),32,32);
		glPopMatrix();
#endif

		glLineWidth(1.0);
		glTranslatef(pNode->pos(0), pNode->pos(1), pNode->pos(2));
		glRotatef(pNode->rot(3)*180./M_PI, pNode->rot(0), pNode->rot(1), pNode->rot(2));
		glRotatef(q[index]*180./M_PI, 0., 0., 1.);
		drawBlock(0.025, 0.05);
#if 0
		glPushMatrix();
		glTranslatef(pNode->com(0), pNode->com(1), pNode->com(2));
		glutSolidSphere(0.025,20,20);
		glPopMatrix();
#endif
//		myModel.joints[index].theta = q[index];
		index ++;

//		Joint link1;
//		link1.setRot(pNode->rot.block(0,0,3,1), pNode->rot(3));
	}
	while (pNode = pNode->childs);
	glPopMatrix();
#endif

	VectorXd r0 = VectorXd::Zero(3);
	double mat[16] = {0.};

	// Draw Skeleton of the Current State
	if ( goalIdx <= 0 )
	{
		pthread_mutex_lock(&link_mutex);
		myModel.updateState(q);
		for ( i = 0 ; i < myModel.numLinks ; i++ )
		{
			Link	link = myModel.localLinks[i];

			VectorXd r0	= myModel.joints[link.index].getGlobalPos(link.from);
			VectorXd r	= myModel.joints[link.index].getGlobalPos(link.to);

			drawCapsule(r0, r, link.radius);
		}
		pthread_mutex_unlock(&link_mutex);
	}

	glColor3f(1.0, 0.0, 0.0);
	VectorXd unit = VectorXd::Zero(3);
	unit(0) = 1.;

	pthread_mutex_lock(&mutex);
	tickCount++;
	vector<VectorXd> *qp;

	if ( bPath )
	{
		qp = &qp1;
		glColor3f(1.0, 1.0, 0.8);
	}
	else
	{
		qp = &qp2;
		glColor3f(0.8, 0.8, 0.8);
	}

//	if ( (tickCount % 33) == 0)
//		cerr << "tickCount " << tickCount;

	if ( qp->size() > 0 && eta.size() > 0 )
	{
		double sec = (double)tickCount * 0.03;
		static int dest = 0;
//		seq	= (tickCount/2 ) % (3*qp->size()-1);
//		cerr << "sec: " << sec << endl;
		if ( sec >= 20. )
		{
			seq = 0.;
			pushing = false;
		}
		else if ( sec >= 10. )
		{
			seq	= (20. - sec ) / 10. * dest;
			
		}
		else if ( sec < 5. )
		{
			int idx = (int)(sec*100.);
			assert(idx < 500 );
			seq = eta[idx];
			dest = (int)seq;

			if ( elbow_log.size() > 0 )
			{
				if (elbow_log.size() != elbow_des_log.size())
				{
					cerr << elbow_log.size() << "!=" <<  elbow_des_log.size() << endl;
					assert(0);
				}
				for ( int i = 0  ; i < elbow_log.size() ; i++ )
				{
					Vector elbow = elbow_log[i];
					Vector elbow_des = elbow_des_log[i];

					double err = (elbow - elbow0).norm();
					cerr << time_log[i] << " " << seq << " " << elbow[0] << " " << elbow[1] << " " << elbow[2] << " " << err << "  ";

					err = (elbow_des - elbow0).norm();
					cerr << elbow_des[0] << " " << elbow_des[1] << " " << elbow_des[2] << " " << err << endl;
					time_log.clear();
					elbow_log.clear();
					elbow_des_log.clear();
				}
			}
		}

		if ( !( seq >= 0 && seq <= qp->size() - 1) )
		{
			cerr << seq << " " << qp->size() << endl;
			assert(0);
		}
//		assert( seq >= 0 && seq <= qp->size() - 1);
	}
	else
		pushing = false;
	if ( seq_ui > 0 )
	{
		seq = seq_ui-1;
	}

	VectorXd Q = q;


	if ( qp->size() )
	{
		int idx = (int)seq;
		double ratio = seq - idx;
		VectorXd q0, q1;

		if ( idx < qp->size()-1 )
		{
			q0 = (*qp)[idx];
			q1 = (*qp)[idx+1];

			Q = (1.-ratio) * q0 + ratio * q1;
		}
		else
			Q = (*qp)[idx];
	}
	if ( prm->numNodes > 0 && node_id > 0 )
		Q = getQ(prm->nodes[node_id-1]->q);

	pthread_mutex_unlock(&mutex);

//	for ( ; it != qp->end() ; it++ )
	if ( qp->size() > 0 )
	{
//		cerr << seq << " / " << qp->size() << endl;
#if 0
		index = 0;
		pNode = baseNode.childs;
		glPushMatrix();
		do {

			glLineWidth(3.0);
			glBegin(GL_LINES);
			glVertex3f(0., 0., 0.);
			glVertex3f(pNode->pos(0), pNode->pos(1), 0.);
			glEnd();

			glBegin(GL_LINES);
			glVertex3f(pNode->pos(0), pNode->pos(1), 0.);
			glVertex3f(pNode->pos(0), pNode->pos(1), pNode->pos(2));
			glEnd();

			glLineWidth(1.0);
			glTranslatef(pNode->pos(0), pNode->pos(1), pNode->pos(2));
			glRotatef(pNode->rot(3)*180./M_PI, pNode->rot(0), pNode->rot(1), pNode->rot(2));
			glRotatef(Q(index)*180./M_PI, 0., 0., 1.);
			drawBlock(0.025, 0.05);
			glPushMatrix();
			glTranslatef(pNode->com(0), pNode->com(1), pNode->com(2));
	//		glutSolidSphere(0.025,20,20);
			glPopMatrix();
			index ++;
		}
		while (pNode = pNode->childs);
		glPopMatrix();
#endif

		// Draw Skeleton of Plan
		pthread_mutex_lock(&link_mutex);
		myModel.updateState(Q);

		for ( i = 0 ; i < myModel.numLinks ; i++ )
		{
//			Link link = myLocalLink[i];
			Link link = myModel.localLinks[i];
			VectorXd r0	= myModel.joints[link.index].getGlobalPos(link.from);
			VectorXd r	= myModel.joints[link.index].getGlobalPos(link.to);
			if ( goalIdx <= 0 )
			{
				glBegin(GL_LINES);
				glVertex3f(r(0), r(1), r(2));
				glVertex3f(r0(0), r0(1), r0(2));
				glEnd();
			}
			else
			{
				drawCapsule(r0, r, link.radius);
			}
		}
		pthread_mutex_unlock(&link_mutex);


		// Draw Sphere on CoM of Each Limb
#if 1
		VectorXd r, r1;
		pthread_mutex_lock(&link_mutex);
		myModel.updateState(Q);
		for ( i = 0 ; i < DOF+1 ; i++ )
		{
			r = myModel.joints[i].getGlobalPos(myModel.joints[i].com);
			glColor3f(1.0, 1.0, 1.0);
#if 0
			glPushMatrix();
			glTranslatef(r(0), r(1), r(2));
			glutSolidSphere(0.020,20,20);
			glPopMatrix();
#else
			drawSphere(r, 0.02, 20);
#endif
		}

		r = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));
		VectorXd hand = VectorXd::Zero(3);
		hand(1) = -0.05;
		r1 = myModel.joints[9].getGlobalPos(hand);
		pthread_mutex_unlock(&link_mutex);

		// Elbow
		glColor3f(0.0, 0.0, 0.1);
		drawSphere(r, 0.04, 20);

		// End Effector
		glColor3f(0.0, 0.0, 1.0);
		drawSphere(r1, 0.04, 20);
	}

#if 0
	glPushMatrix();
	glColor3f(1.,0.,0.);
	glTranslatef(0.3, -0.1, 0.2);
	glutSolidSphere(0.025,20,20);
	glPopMatrix();
#endif
	
	glColor3f(0.0, 1.0, 0.0);
	drawSphere(elbow0, 0.020,20);

	glColor3f(0.0, 0.0, 1.0);
	drawSphere(elbow, 0.040,20);

	glColor3f(1.0, 1.0, 1.0);
	glLineWidth(3.0);
	glBegin(GL_LINES);
	glVertex3f(elbow0(0), elbow0(1), elbow0(2));
	glVertex3f(elbow(0), elbow(1), elbow(2));
	glEnd();

	glColor3f(0.,1.,0.);
	drawSphere(endeffector, 0.025, 20);

	glPushMatrix();
	glColor3f(1.,0.,0.);
	glTranslatef(.4, -.2, .2);
	glutSolidSphere(0.025,20,20);
	glPopMatrix();


//	fprintf(stderr, "%f %f %f %f %f %f\n", torque[0], torque[1], torque[2], force[0], force[1], force[2]); 
	// Draw Human Posture

#if 0
	for ( i = 0 ; i < 3 ; i++ )
	{
		double mag = force[i];
		double vec[3][3] = { 	{ 1., 0., 0. },
								{ -.5, sqrt(3.)/2., 0.},
								{ -.5, -sqrt(3.)/2., 0.}};

		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(0., 0., 0.);
		glVertex3f(vec[i][0]*mag, vec[i][1]*mag, vec[i][2]*mag);
		glEnd();
		glPopMatrix();
	}
#endif
	
	glColor3f(0.0, 1.0, 0.0);
	drawSphere(gGoal, 0.05, 100);

	glLineWidth(1.0);

	// Floor 
	glPushMatrix();
	glRotatef(phi*180./M_PI, 0., 0., 1.);
	double h = -0.5;
	for ( i = -10 ; i <= 10 ; i++ )
	{
		glLineWidth(1.0);
		glColor3f(1., 1., 1.);

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
#else // Unused parts
#endif

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
    baseButton = new Fl_Button(5, height - 35, 100, 30, "&Path"); 
    baseButton->callback(cb_path, this); 
    learnBaseButton = new Fl_Button(5, height - 35, 100, 30, "&Simul"); 
    learnBaseButton->callback(cb_simul, this); 
    sendButton = new Fl_Button(5, height - 35, 100, 30, "&Send"); 
    sendButton->callback(cb_send, this); 
    pushXButton = new Fl_Button(5, height - 35, 100, 30, "&PushX"); 
    pushXButton->callback(cb_pushx, this); 
    pushYButton = new Fl_Button(5, height - 35, 100, 30, "&PushY"); 
    pushYButton->callback(cb_pushy, this); 
    pushZButton = new Fl_Button(5, height - 35, 100, 30, "&PushZ"); 
    pushZButton->callback(cb_pushz, this); 
    goalButton = new Fl_Button(5, height - 35, 100, 30, "&Goal"); 
    goalButton->callback(cb_goal, this); 

	printf("WIN: %p\n", this);
    quitButton = new Fl_Button(width - 105, height - 35, 100, 30, "&Quit"); 
    quitButton->callback(cb_quit, this); 

	mGroupSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mGroupSlider->type(FL_HORIZONTAL);
	mGroupSlider->bounds(0., (double)1000.);
	mGroupSlider->callback(cb_quit, this);
	mGroupSlider->step(1.);
	mGroupSlider->value(0);

	mSizeSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mSizeSlider->type(FL_HORIZONTAL);
	mSizeSlider->bounds(1., 400.);
	mSizeSlider->callback(cb_quit, this);
	mSizeSlider->step(1.);
	mSizeSlider->value(0.); //window_size);

	mSliceSlider = new Fl_Value_Slider( 10, height - 30, width / 4 - 100, 30, "");
	mSliceSlider->type(FL_HORIZONTAL);
	mSliceSlider->bounds(0., 16.);
	mSliceSlider->callback(cb_control, this);
	mSliceSlider->step(1.);
	mSliceSlider->value(1.);

	mScaleSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mScaleSlider->type(FL_HORIZONTAL);
	mScaleSlider->bounds(-0.01, 0.01);
	mScaleSlider->step(0.001);
	mScaleSlider->callback(cb_speed, this);
	mScaleSlider->value(0);

	mNodeSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mNodeSlider->type(FL_HORIZONTAL);
	mNodeSlider->bounds(-3., 0.);
	mNodeSlider->step(1.0);
	mNodeSlider->callback(cb_node, this);
	mNodeSlider->value(0);

	mSeqSlider = new Fl_Value_Slider( width / 4 + 20, height - 30, width / 4 - 100, 30, "");
	mSeqSlider->type(FL_HORIZONTAL);
	mSeqSlider->bounds(0., 0.);
	mSeqSlider->step(1.);
	mSeqSlider->callback(cb_seq, this);
	mSeqSlider->value(0);

	mHighlightSlider = new Fl_Value_Slider( 10, height - 35, width / 4 - 100, 30, "");
	mHighlightSlider->type(FL_VERTICAL);
	mHighlightSlider->bounds(0., (double)1080.-1.);
	mHighlightSlider->callback(cb_quit, this);
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
	pushXButton->resize(	w-300,	h - 105, 70, 25);
	pushYButton->resize(	w-300,	h - 70, 70, 25);
	pushZButton->resize(	w-300,	h - 35, 70, 25);
	randomButton->resize(	w-225,	h - 105, 70, 25);
	planButton->resize(		w-225,	h - 70, 70, 25);
	modeButton->resize(		w-225,	h - 35, 70, 25);
	sendButton->resize(		w-150,	h - 105, 70, 25);
	baseButton->resize(		w-150,	h - 70, 70, 25);
	learnBaseButton->resize(w-150,	h - 35, 70, 25);
	quitButton->resize(		w-75,	h - 35, 70, 25);

}

extern Vector Mins;
extern Vector Maxs;

void MyWindow::
timer_cb(void * param)
{
	static double lastZ = 10.0;

	double x, z;
//	x = myModel.getX();
//	z = myModel.getZ();

//	if ( z <= 0.0 && lastZ <= 0.0 )
//		paused = true;
	
//	if ( !paused )

	if ( !bSimul )
	{
		q[control] += speed;
		int joint = control;

		if ( joint > 1)
			joint--;

		if ( q[control] > Maxs[joint] )
			q[control] = Maxs[joint];
		else if ( q[control] < Mins[joint] )
			q[control] = Mins[joint];

		q[2] = q[1];
	}

	periodicTask();

	static int count = 0;

	if ( (count % 30) == 0 )
	{
//		myModel.update();

		lastZ = z; 

//		traceX.push_back(x);
//		traceZ.push_back(z);

		reinterpret_cast<MyWindow*>(param)->sim->redraw();
	}
	count++;
/*
	if ( ! paused || ! paused_ready ) {
		reinterpret_cast<Simulator*>(param)->tickCount();
	}

	if ( paused )
		paused_ready = true;
	if ( ! paused )
		paused_ready = false;
*/

	vector<VectorXd> *qp;
	if ( bPath )
		qp = &qp1;
	else
		qp = &qp2;

	reinterpret_cast<MyWindow*>(param)->mSeqSlider->bounds(0., (double)(qp->size()));
	reinterpret_cast<MyWindow*>(param)->mNodeSlider->bounds(0., (double)(prm->numNodes));

	Fl::repeat_timeout(dt, // gets initialized within tickCount()
			   timer_cb,
			   param);

}

void MyWindow::
cb_simul(Fl_Widget *widget, void *param)
{
	bSimul = !bSimul;
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
	control = pSlider->value();
}

void MyWindow::
cb_speed(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	speed = pSlider->value();
}

void MyWindow::
cb_reset(Fl_Widget *widget, void *param)
{
	bSimul = false;
	for ( int i = 0 ; i < 10 ; i++ )
		 q[i] = 0.;
}

void MyWindow::
cb_plan(Fl_Widget *widget, void *param)
{
	bPlan = !bPlan;
}

void MyWindow::
cb_path(Fl_Widget *widget, void *param)
{

	bPath = !bPath;
}

void MyWindow::
cb_random(Fl_Widget *widget, void *param)
{
	bRandom = true;
	goalIdx = -1;
}

void MyWindow::
cb_send(Fl_Widget *widget, void *param)
{
	bSend = !bSend;
//	tickCount = 0;
}

int push_type = 0;

void MyWindow::
cb_pushx(Fl_Widget *widget, void *param)
{
	VectorXd dir = VectorXd::Zero(3);
	dir(0) = 1;
	push(dir);
}
void MyWindow::
cb_pushy(Fl_Widget *widget, void *param)
{
	VectorXd dir = VectorXd::Zero(3);
	dir(1) = -1;
	push(dir);
}
void MyWindow::
cb_pushz(Fl_Widget *widget, void *param)
{
	VectorXd dir = VectorXd::Zero(3);
	dir(2) = 1;
	push(dir);
}

extern bool getPotentialVerbose;
extern int pushType;
void MyWindow::
cb_seq(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	seq_ui = (int)pSlider->value();
	int seq_no = seq_ui - 1;

	if ( seq_no >= 0 )
	{
		VectorXd contact	= elbow0;

		pthread_mutex_lock(&mutex);
		vector<VectorXd> *qp;
		if ( bPath )
			qp = &qp1;
		else
			qp = &qp2;
		VectorXd Q = (*qp)[seq_no]; 
		pthread_mutex_unlock(&mutex);

#if 1
		WbcNode node;

		node.q = getQa(Q);
		node.getProjection();

		getPotentialVerbose = true;
		double val = getPotential(pushType, contact, node, r0);

		cerr << seq_no << ": " << val << ":" << Q.transpose() * 180. / M_PI << endl;
#endif
	}
	getPotentialVerbose = false;
}

void MyWindow::
cb_node(Fl_Widget *widget, void *param)
{
	Fl_Value_Slider *pSlider = (Fl_Value_Slider *)widget;
	node_id = (int)pSlider->value();
	if ( node_id > 0 )
		cerr <<  prm->nodes[node_id-1]->q.transpose()*180./M_PI << endl;
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
	int i;
	Vector3d goal(3);
#if 0
	double bound[3][2] = {
	{-0.3,0.3}, {-0.3,0.3}, {0,.5}
	};

	for ( i = 0 ; i < 3 ; i++ )
	{
		goal(i) = bound[i][0] + (double)rand() * (bound[i][1] - bound[i][0]) / RAND_MAX;
	}
#else
	static int idx = 0;

	goal(0) = goals[idx][0];
	goal(1) = goals[idx][1];
	goal(2) = goals[idx][2];
	idx = (idx+1) % 5;
#endif

	gGoal = goal;

 	intervention();
}

MyWindow::~MyWindow(void)
{
	Fl::remove_timeout(timer_cb, this);
}
