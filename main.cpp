#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "tinyxml.h"
#include <sys/time.h>

#include "config.h"
#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"
#include "Octree.h"

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

#include "mywindow.h"
#include "Comm.h"
#include "kin.h"
#include "timestamp.h"
#include "model.h"
#include "default.h"
#include "wbc.h"
#include "rvolume.h"

#define CHECK_LIMIT
#define TICK_SEC (1000)

using namespace std;


int win_width(800);
int win_height(600);
static char win_title[100];
#ifdef USE_WBC
static State body_state;
void simulate(State &body_state);
#endif

KinModel myModel;

double		gGoalTime = 1e10;
Vector3d	gGoal;
double		gObjTime;
Vector3d	gObj;
Vector3d	gObjVel;
vector<Vector3d> elbow_log;
vector<double>   time_log;

//void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0);
void *plan(void *);

int goalIdx = -1;
VectorXd q0;
extern int seq_ui;
extern vector<double> eta;
#ifdef Linux
double dt = 0.001;
#else
double dt = 0.03;
#endif
Vector3d desired_pos;
//vector<Link>	myLocalLink;
//vector<Link>	myGlobalLink;
WbcRRT *rrt;
PRM<DOF> *prm;
extern int sample_ui;
#define FILE_NAME_SIZE (200)
char joint_name[FILE_NAME_SIZE];
char link_name[FILE_NAME_SIZE];
extern void initSimulation(const char *robot_spec);

Vector3d elbow_des;
Vector3d elbow0;
Vector3d elbow;
Vector3d error;
double q_init[DOF+1] = {0.};

VectorXd interpolate(const vector<VectorXd> &list, double seq);
void intervention(void);
Vector3d intervention2(void);

VectorXd getQ(const VectorXd &uq)
{
	VectorXd ret = VectorXd::Zero(myModel.numJoints);

	assert(uq.size() == myModel.numJoints-1);

	ret.block(0,0,2,1) = uq.block(0,0,2,1);
	ret.block(2,0,myModel.numJoints-2,1) = uq.block(1,0,myModel.numJoints-2,1);

	return ret;
}
VectorXd getQa(const VectorXd &q)
{
	VectorXd ret = VectorXd::Zero(myModel.numJoints-1);

	assert(q.size() == myModel.numJoints);

	ret.block(0,0,2,1) = q.block(0,0,2,1);
	ret.block(2,0,myModel.numJoints-3,1) = q.block(3,0,myModel.numJoints-3,1);

	return ret;
}

pthread_mutex_t	mutex;
pthread_mutex_t	model_mutex;
pthread_mutex_t link_mutex;

Comm *pSttComm;
Comm *pCmdComm;
Comm *pObjComm;
Comm *pKinComm;


#define INIT_JOINT	(0x0001)
#define INIT_LINK	(0x0002)

void periodicTask(void);
int main(int argc, char *argv[])
{
	int c;
	TiXmlDocument doc;
	int loaded = 0;
	const char *robot_spec;
	pthread_t planning_thread;
	RVolume rv(LINKS);

	RegisterValue(win_width);
	RegisterValue(win_height);
	RegisterValue(hor);
	RegisterValue(ver);
	RegisterValue(scale);
	RegisterValue(link_name);
	RegisterValue(joint_name);
	RegisterValue(q_init);

	Default::load(argv[0]);

	while ( (c = getopt(argc, argv, "j:l:t:dh") ) != -1 )
	{
		switch (c)
		{
			case 'j':
				strncpy(joint_name, optarg,FILE_NAME_SIZE);
				break;
			case 'l':
				strncpy(link_name, optarg, FILE_NAME_SIZE);
				break;
			case 'h':
				fprintf(stderr, "-j <joint list>\n-l <link list>\n");
				break;
			default:
				break;

		}
	}

	loaded = 0;
	loaded |= (myModel.initJoint(joint_name))?INIT_JOINT:0;
	loaded |= (myModel.initLink(link_name))?INIT_LINK:0;
	robot_spec = joint_name;

	if ( !(loaded != (INIT_LINK|INIT_JOINT)) )
		exit(-1);

	q0 = VectorXd::Zero(DOF+1);
	{
		int i;
		for ( i = 0 ; i < DOF+1 ; i++ )
			q0[i] = q_init[i];
	}
	disp_q1 = q0;

	cerr << "Model Init Done... " << endl;
	myModel.updateState(disp_q1);
	cerr << disp_q1.transpose() << endl;

	cerr << "Model update... " << endl;

	int i, j;

#if 0
	int idxs[] = {1, 1, 2, 2, 2, 4, 6, 9};
	double radii[] = {0.07, 0.07, 0.07, 0.07, 0.06, 0.06, 0.06, 0.06};
	double fromJoint[][3]	= { 
		{0.,0.,0.06},    
		{0.,0.,-.06},    
		{0.,0.,0.06},   
		{0.,0.,-0.06},  
		{.2337,0.,-0.18465},
		{0.03175,-0.27857,0.},
		{0.,0.,0.},     
		{0.,-.1,0.}
	};
	double toJoint[][3]		= { 
		{0.1397,0.,0.06},
		{0.1397,0.,-.06},
		{.2337,0.,0.06},
		{.2337,0.,-.06},
		{.2337,0.,-.06},    
		{0.,0.,0.},           
		{0.,0.27747,0.},
		{0.,0.,0.}
	};

	int numLinks = sizeof(idxs) / sizeof(int);
	cerr << "Num Link : " << numLinks << " " << sizeof(fromJoint) << "/" << sizeof(double) << endl;
	assert( sizeof(fromJoint) / sizeof(double) / 3 == numLinks);
	assert( sizeof(toJoint) / sizeof(double) / 3 == numLinks);

	myLocalLink.clear();
	pos3D temp;
	for ( i = 0 ; i < numLinks ; i++ )
	{
		Link link;
		link.index	= idxs[i];
		link.radius	= radii[i];
		link.from	= fromJoint[i];
		link.to		= toJoint[i];
		temp.sub(link.from, link.to);
		link.length	= pos3D::norm(temp);
		myLocalLink.push_back(link);
	}
//	myGlobalLink = myLocalLink;
#endif
	
	cerr << "OpenGL Init" << endl;
	glutInit(&argc, argv);

	pthread_mutex_init( &mutex, NULL );
	pthread_mutex_init( &model_mutex, NULL );
	pthread_mutex_init( &link_mutex, NULL );

#ifdef USE_WBC
	initSimulation(robot_spec);
	cerr << __LINE__ << " " << myModel.numJoints << endl;
	body_state = State(myModel.numJoints-1, myModel.numJoints-1, 6);

	WbcNode::model->update(body_state);
#endif
//	WbcNode::callbackAdd = NULL;


	WbcNode::callbackAdd = (WbcNode::CALLBACK)RVolume::addToOctree;

	pthread_create( &planning_thread, NULL, plan, NULL);

	pSttComm = new Comm(STT_PORT, 20000);
	pSttComm->listen();

	pCmdComm = new Comm("192.168.50.33", CMD_PORT, 20000);

	pObjComm = new Comm(STT_PORT+2, 20000);
	pObjComm->listen();

	pKinComm = new Comm(STT_PORT+3, 20000);

	MyWindow win(win_width, win_height, "test");


	periodicTask();

	int ret = Fl::run();

	Default::save();
}

Vector3d	gIntervenePos;
Vector3d	gInterveneVel;
int qmap[] = {0,1,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
void periodicTask(void)
{
	double seq = -1.;
	double currentTime = Timestamp::getCurrentTime();

	if ( !(getRobotState() & STATE_SIMUL) )
	{
		int i;

		Message msg;

		if ( (msg = pSttComm->read()).size > 0 )
		{
			double *pData = (double *)msg.data;
//			fprintf(stderr, "%d data\n", msg.size);
			for ( i = 0 ; i < DOF ; i++ )
			{
				disp_q1[qmap[i]] = pData[i];
			}
			disp_q1[2] = disp_q1[1];
			free(msg.data);
//			cerr << disp_q1.transpose() << endl;
		}

		if ( (msg = pObjComm->read()).size > 0 )
		{
			double *pData = (double *)msg.data;
			gObjTime	= currentTime;
			gObj[0] = pData[0];
			gObj[1] = pData[1];
			gObj[2] = pData[2];
			gGoalTime = pData[3];
			gGoal[0] = pData[4];
			gGoal[1] = pData[5];
			gGoal[2] = pData[6];
			gObjVel[0] = pData[7];
			gObjVel[1] = pData[8];
			gObjVel[2] = pData[9];
			free(msg.data);
		}

		{
#if 0
			{
				static double prevTime = 0;
				if ( prevTime + 5 < currentTime )
				{
					if ( gGoalTime < 5 )
					{
						gIntervenePos = gObj;
						gInterveneVel = intervention2();

						prevTime = currentTime;
					}
					else
					{
//						gIntervenePos = Vector3d::Zero();
//						gInterveneVel = Vector3d::Zero();
					}
				}
			}
#else
			{
				static double prevTime = 0;
				if ( !(getRobotState() & (STATE_OPERATE|STATE_OPERATING)) )
				{
					if ( gGoalTime < 5 )
					{
						gIntervenePos = gObj;
						gInterveneVel = intervention2();

						prevTime = currentTime;
					}
					else
					{
//						gIntervenePos = Vector3d::Zero();
//						gInterveneVel = Vector3d::Zero();
					}
				}
				else
				{
					double inner = gObjVel.transpose() * gInterveneVel;

					if ( inner < 0 )
					{
						cerr << "===============================" << endl;
						cerr << "Finalize intervention" << endl;
						clearRobotState(STATE_OPERATE | STATE_OPERATING);
						setRobotState(STATE_RETURN | STATE_RETURNING);
						gInterveneVel = Vector3d::Zero();
					}
				}
			}
#endif
		}

		{
			static int count = 0;

			if ( (count%1000) == 0 )
			{
#if 0
				WbcNode node;
				VectorXd qa;
				VectorXd q_des;
				VectorXd q_err;

				qa = getQa(disp_q1);
				q_des = VectorXd::Zero(DOF);
				q_des(5) = 1.57;
				VectorXd  tau;

				q_err = qa - q_des;
				node.q = qa;

				tau = node.getProjection()*q_err;
				cerr << "=== Posture ===" << endl;
				cerr << q.transpose() << endl;
				cerr << q_err.transpose() << endl;
				cerr << tau.transpose() << endl;
#endif
			}

			count++;
		}
		{
			static int prevState = 0;
			int state = getRobotState();
			static int currentIdx = 0;

			if ( state & STATE_OPERATING )
			{
				if ( !(prevState & STATE_OPERATING) )
				{
					Timestamp::resetTime();
					currentIdx = 0;
				}
				double sec = Timestamp::getElapsedTime();
				int idx = (int)(sec*100.);
				seq = eta[idx];
				seq = sec*30.;
				if ( seq >= qp1.size()-1 )
				{
					seq = qp1.size()-1;
				}

			}
			if ( state & STATE_RETURNING )
			{
				static double startIdx = -1;
				if ( !(prevState & STATE_RETURNING) )
				{
					startIdx = seq;
					cerr << "Returning " << startIdx << endl;
					Timestamp::resetTime();
				}
				double sec = Timestamp::getElapsedTime();
				seq = startIdx - sec*0.1;
				if ( seq < 0 )
				{
					cerr << "Returned " << startIdx << " " << sec << endl;
					seq = 0;
					clearRobotState(STATE_RETURNING | STATE_RETURN);
				}
			}

			if ( qp1.size() > 0 && seq >= 0 )
			{
				disp_q1 = interpolate(qp1, seq);
			}

			prevState = state;
		}
		if ( seq < 0. )
			seq = 0.;
		
#if 0
		if ( qp1.size() > 0 && eta.size() > 0 )
		{
			static int dest = 0;
			double sec = Timestamp::getElapsedTime();
	//		cerr << "sec: " << sec << endl;
			if ( sec >= 20. )
			{
				seq = 0.;
				clearRobotState(STATE_OPERATING);
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
			}

			disp_q1 = interpolate(qp1, seq);
		}
#endif

//		if ( sample_ui > 0 && sample_ui <= rrt->numNodes )
//			disp_q2 = getQ(rrt->nodes[sample_ui - 1]->q);

		if ( !(getRobotState() & STATE_LEARNING) )
		{
#if 0
			WbcNode node;
			node.q = getQa(disp_q1);
			node.getProjection();

			desired_pos = node.actual;
#else
			if (disp_q1.rows() != LINKS)
			{
				cerr << disp_q1.rows() << endl;
				assert(0);
			}
			myModel.updateState(disp_q1);
			VectorXd p = VectorXd::Zero(3);
			p(1) = -0.05;
			desired_pos = myModel.joints[9].getGlobalPos(p);
#endif
		}

		int j;

		pthread_mutex_lock(&link_mutex);

		myModel.updateState(q0);
		elbow0 = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));

		myModel.updateState(disp_q1);
		elbow = myModel.joints[5].getGlobalPos(VectorXd::Zero(3));

		pthread_mutex_unlock(&link_mutex);

		error = elbow - elbow0;

//		if ( getRobotState() & STATE_OPERATING );
		{
//			cerr << "Elbow Logging" << endl;
			pthread_mutex_lock(&mutex);
			elbow_log.push_back(elbow);
			time_log.push_back(Timestamp::getElapsedTime());
			pthread_mutex_unlock(&mutex);
		}
	}
#ifdef USE_WBC
	else // Simulation
	{
		body_state.position_ = getQa(disp_q1);
		simulate(body_state);
		disp_q1 = getQ(body_state.position_); 
	}
#endif

#if 0
	{
		static double prevTime = 0;

		if ( prevTime + 1. < currentTime )
		{
			fprintf(stderr, "%f %08x\n", getRobotState());
			prevTime = currentTime;
		}

	}
#endif

	{
		Message msg;
		double dat[DOF+1];
		msg.timeStamp = 0;
		msg.data	= &dat;
		msg.size	= sizeof(dat);
		for ( int i = 0 ; i < DOF+1 ; i++ )
		{
			dat[i] = disp_q1[i];
		}
		pKinComm->send(&msg);
	}

	if ( (getRobotState() & STATE_SEND && qp1.size() > 2 ) )
	{
		Message msg;
		VectorXd qav;

		VectorXd q = interpolate(qp1, seq);
		seq_ui = seq + 1;
		qav = getQa(q);
		
		command cmd;
		cmd.command = CMD_SET_CONFIGURATION;
		cmd.data_len = 10;
		for ( int i = 0 ; i < 10 ; i++ )
		{
			cmd.buf[i] = qav[i];
		}

		msg.timeStamp = 0;
		msg.data	= &cmd;
		msg.size	= sizeof(cmd);
			
		pCmdComm->send(&msg);
//		cout << "JPos: " << endl << disp_q1 << endl;
//		cout << "JPos: " << qav.transpose()  << endl;
	}
}

static int robot_state;
void setRobotState(int st)
{
//	if ( (st & robot_state) == st )
//		return;

	robot_state |= st;
}
void clearRobotState(int st)
{
//	if ( (st & (~robot_state)) == st )
//		return;

	robot_state &= ~st;
}

int getRobotState(void)
{
	return robot_state;
}

