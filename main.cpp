#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <tinyxml.h>
#include <sys/time.h>

#include <jspace/Model.hpp>
#include <jspace/State.hpp>
#include <jspace/test/sai_util.hpp>
#include <jspace/pseudo_inverse.hpp>
#include <boost/scoped_ptr.hpp>

#include "wbcrrt.h"
#include "prm.hpp"
#include "xml.h"

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

#define CHECK_LIMIT

using namespace std;
using namespace boost;
using namespace jspace;

static int win_width(800);
static int win_height(600);
static char win_title[100];
static scoped_ptr<jspace::Model> model;
static scoped_ptr<jspace::Model> model_planning;
State body_state(9, 9, 6);

vector<VectorXd> elbow_log;
vector<VectorXd> elbow_des_log;
vector<double>   time_log;
//void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0);
void *plan(void *);

XmlNode baseNode;
bool bSimul = false;
bool bPlan = false;
bool bPlanning = false;
bool bRandom = false;
int goalIdx = -1;
bool bSend = false;
extern VectorXd q;
extern VectorXd q0;
extern vector<VectorXd> r0;
extern vector<VectorXd> qp1;
extern vector<VectorXd> qp2;
vector<double> dist;
vector<double> distp;
vector<double> d_t;
vector<double> eta;
static double dt = 0.001;
VectorXd desired_pos;
extern int tickCount;
extern double seq; 
vector<Joint>	myJoint(10);
vector<Link>	myLocalLink;
vector<Link>	myGlobalLink;
WbcRRT *rrt;
PRM<9> *prm;
extern double *value;
extern int seq_ui;

Vector sentQ;
Vector elbow_des;
Vector elbow0;
Vector elbow;
Vector error;

VectorXd endeffector;

void push(VectorXd dir);
double getPotential(int mode, const VectorXd &contact, const WbcNode &node, const vector<VectorXd> &r0);

VectorXd getQ(const VectorXd &uq)
{
	VectorXd ret = VectorXd::Zero(10);

	assert(uq.size() == 9);

	ret.block(0,0,2,1) = uq.block(0,0,2,1);
	ret.block(2,0,8,1) = uq.block(1,0,8,1);

	return ret;
}
VectorXd getQa(const VectorXd &q)
{
	VectorXd ret = VectorXd::Zero(9);

	assert(q.size() == 10);

	ret.block(0,0,2,1) = q.block(0,0,2,1);
	ret.block(2,0,7,1) = q.block(3,0,7,1);

	return ret;
}

pthread_mutex_t	mutex;
pthread_mutex_t	model_mutex;
pthread_mutex_t link_mutex;

Comm *pRecvComm;
Comm *pCmdComm;

void periodicTask(void);
int main(int argc, char *argv[])
{
	int c;
	TiXmlDocument doc;
	bool loaded = false;
	const char *robot_spec;
	pthread_t planning_thread;

	while ( (c = getopt(argc, argv, "f:c:t:dh") ) != -1 )
	{
		switch (c)
		{
			case 'f':
				try 
				{
					doc = TiXmlDocument( optarg );
					robot_spec = optarg;
					loaded = doc.LoadFile();

					cerr << "Load XML File " << loaded << endl;
				}
				catch (int e)
				{
					std::cerr << "ERROR!!!";
				}
				break;
			default:
				break;
		}

	}
	if ( !loaded )
		exit(-1);

	TiXmlNode* pChild;
	TiXmlText* pText;
	int t = doc.Type();

	baseNode.parseXML(&doc);

	XmlNode *node = baseNode.childs;

	q = VectorXd::Zero(10);
	endeffector = VectorXd::Zero(3);;
	q0 = q;

	int i = 0, j;
	while ( node )
	{
		if ( node->name )
			cerr << node->name << endl;

		Joint link;

		link.com = node->com;
		link.setRot(node->rot.block(0,0,3,1), node->rot(3));
		link.trans	= node->pos;
		link.axis	= 2;
		link.mass	= node->mass;
		if ( i > 0 )
			link.parent = &(myJoint[i-1]);
		else
			link.parent = NULL;
		myJoint[i] = link;

		node = node->childs;
		i++;
	}

	r0.clear();
	for ( j = 0 ; j < 10 ; j++ )
	{
		myJoint[j].setTheta(q0[j]);
	}
	for ( j = 0 ; j < 10 ; j++ )
	{
		r0.push_back(myJoint[j].getGlobalPos(myJoint[j].com));
	}

	int idxs[] = {1, 1, 2, 2, 2, 4, 6, 9};
	double fromJoint[][3]	= { {0.,0.,0.06},    {0.,0.,-.06},    {0.,0.,0.06},   {0.,0.,-0.06},  {.2337,0.,-.06},    {0.,0.,0.},           {0.,0.,0.},     {0.,0.,0.}};
	double toJoint[][3]		= { {0.1397,0.,0.06},{0.1397,0.,-.06},{.2337,0.,0.06},{.2337,0.,-.06},{.2337,0.,-0.18465},{0.03175,-0.27857,0.},{0.,0.27747,0.},{0.,-.1,0.}};

	int numLinks = sizeof(idxs) / sizeof(int);
	cerr << "Num Link : " << numLinks << " " << sizeof(fromJoint) << "/" << sizeof(double) << endl;
	assert( sizeof(fromJoint) / sizeof(double) / 3 == numLinks);
	assert( sizeof(toJoint) / sizeof(double) / 3 == numLinks);

	myLocalLink.clear();
	for ( i = 0 ; i < numLinks ; i++ )
	{
		Link link;
		for ( j = 0 ; j < 3 ; j++ )
		{
			link.from[j] = fromJoint[i][j];
			link.to[j] = toJoint[i][j];
		}
		myLocalLink.push_back(link);
	}
	myGlobalLink = myLocalLink;
	
	glutInit(&argc, argv);

	pthread_mutex_init( &mutex, NULL );
	pthread_mutex_init( &model_mutex, NULL );
	pthread_mutex_init( &link_mutex, NULL );

	model.reset(test::parse_sai_xml_file(robot_spec, false));
	model->setConstraint("Dreamer_Torso");

	WbcNode::model.reset(test::parse_sai_xml_file(robot_spec, false));
	WbcNode::model->setConstraint("Dreamer_Torso");

	pthread_create( &planning_thread, NULL, plan, NULL);

	pRecvComm = new Comm(STT_PORT, 20000);
	pRecvComm->listen();

	pCmdComm = new Comm("192.168.1.110", CMD_PORT, 20000);

	MyWindow win(win_width, win_height, "test");

	periodicTask();

	int ret = Fl::run();
}

MatrixXd getJacobian(const Model &model)
{
	MatrixXd Jfull, J;
	Vector actual_;
	taoDNode const *end_effector_node_ = model.getNode(9);
	jspace::Transform ee_transform;
	model.computeGlobalFrame(end_effector_node_,
		0.0, -0.05, 0.0, ee_transform);
	actual_ = ee_transform.translation();

	model.computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
	J = Jfull.block(0, 0, 3, Jfull.cols());

	return J;
}
bool pushing = false;

extern int push_type;
int qmap[] = {0,1,3,4,5,6,7,8,9};
Timestamp ts1("Period");
void periodicTask(void)
{
	Vector tau1, tau2, tau;
	Vector fullJpos_, fullJvel_;
	Vector actual_;
	Vector desired_posture = Vector::Zero(9);
	MatrixXd UNcBar;
	MatrixXd phi;
	MatrixXd phiinv; 
	MatrixXd J1star;
	MatrixXd Lambda1;
	MatrixXd ainv;
	Vector grav;
	MatrixXd Nc;
	MatrixXd UNc;
	MatrixXd U;
	MatrixXd Jfull, J;
	MatrixXd N1, Lambda2;
	MatrixXd J2star;
	Vector	ddq;


	if ( desired_pos.rows() != 3 )
		desired_pos = VectorXd::Zero(3);

	if ( !bSimul )
	{
		int i;

		Message msg;

		if ( (msg = pRecvComm->read()).size > 0 )
		{
			double *pData = (double *)msg.data;
//			fprintf(stderr, "%d data\n", msg.size);
			for ( i = 0 ; i < 9 ; i++ )
			{
				q[qmap[i]] = pData[i];
			}
		//	cerr << fullJvel_(1) << " " << fullJvel_(2) << endl;
			q[2] = q[1];
			free(msg.data);
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

				qa = getQa(q);
				q_des = VectorXd::Zero(9);
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
#if 0
		for ( i = 0 ; i < 9 ; i++ )
		{
			body_state.position_(i) = q[qmap[i]];
			body_state.velocity_(i) = 0.;
		}
#else
		body_state.position_	= getQa(q);
		body_state.velocity_	= VectorXd::Zero(9);
#endif

#if 1
		if ( !bPlanning )
		{
			WbcNode node;
			node.q = body_state.position_;
			node.getProjection();

			desired_pos = node.actual;
		}
#endif

		int j;

		pthread_mutex_lock(&link_mutex);
		for ( j = 0 ; j < 10 ; j++ )
		{
			myJoint[j].setTheta(q0[j]);
		}
		elbow0 = myJoint[5].getGlobalPos(VectorXd::Zero(3));

		for ( j = 0 ; j < 10 ; j++ )
		{
			myJoint[j].setTheta(q[j]);
		}
		elbow = myJoint[5].getGlobalPos(VectorXd::Zero(3));

		if ( sentQ.size() == 10 )
		{
//			cerr << "sentQ " << sentQ.transpose() << endl;
			for ( j = 0 ; j < 10 ; j++ )
			{
				myJoint[j].setTheta(sentQ[j]);
			}
			elbow_des = myJoint[5].getGlobalPos(VectorXd::Zero(3));
		}
		else
		{
			elbow_des = Vector::Zero(3);
//			cerr << "Size " << sentQ.size() << endl;
		}
		pthread_mutex_unlock(&link_mutex);

		error = elbow - elbow0;

//		if ( pushing )
		{
//			cerr << "Elbow Logging" << endl;
			pthread_mutex_lock(&mutex);
			elbow_log.push_back(elbow);
			elbow_des_log.push_back(elbow_des);
			time_log.push_back(tickCount);
			pthread_mutex_unlock(&mutex);
		}

		if ( seq_ui == 0 && rrt->numNodes > 0 && bPlanning == false && pushing == false)
		{
			if ( error(0) > 0.02 && bPlanning == false && pushing == false)
			{
				cerr << "====================================" << endl;
				cerr << " Push Forward " << error(0) << endl;
				cerr << "====================================" << endl;
				VectorXd dir = VectorXd::Zero(3);
				dir(0) = 1;
				push(dir);
				pushing = true;
				elbow_log.clear();
				elbow_des_log.clear();
				time_log.clear();
			}
			if ( error(1) < -0.02 && bPlanning == false && pushing == false)
			{
				cerr << "====================================" << endl;
				cerr << " Push Right " << endl;
				cerr << elbow.transpose() << " " << elbow0.transpose() << endl;
				cerr << "====================================" << endl;
				VectorXd dir = VectorXd::Zero(3);
				dir(1) = -1;
				push(dir);
				pushing = true;
				elbow_log.clear();
				elbow_des_log.clear();
				time_log.clear();
			}
			if ( error(2) > 0.02 && bPlanning == false && pushing == false)
			{
				cerr << "====================================" << endl;
				cerr << " Push Up " << error(2) << endl;
				cerr << elbow.transpose() << " " << elbow0.transpose() << endl;
				cerr << "====================================" << endl;
				VectorXd dir = VectorXd::Zero(3);
				dir(2) = 1;
				push(dir);
				pushing = true;
				elbow_log.clear();
				elbow_des_log.clear();
				time_log.clear();
			}
//		cerr << "PUSH " << bPlanning << " " << pushing << endl;
		}
	}
	else
	{

		ts1.setBaseline();

		model->update(body_state);

		ts1.checkElapsed(0);

		fullJpos_ = model->getFullState().position_;
		fullJvel_ = model->getFullState().velocity_;

		model->getInverseMassInertia(ainv);

		model->getGravity(grav);

		Constraint * constraint = model->getConstraint();
		
		constraint->updateJc(*model);
		constraint->getNc(ainv,Nc);
		constraint->getU(U);
		UNc = U*Nc;

		ts1.checkElapsed(1);

#if 1
		taoDNode const *end_effector_node_ = model->getNode(9);
		jspace::Transform ee_transform;
		model->computeGlobalFrame(end_effector_node_,
			0.0, -0.05, 0.0, ee_transform);
		actual_ = ee_transform.translation();

		ts1.checkElapsed(2);

		model->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
		J = Jfull.block(0, 0, 3, Jfull.cols());
#else
		MatrixXd J = getJacobian(*model);
#endif
		
		ts1.checkElapsed(3);

		double kp = 100., kd = 10.0;

		phi = UNc * ainv * UNc.transpose();
		//XXXX hardcoded sigma threshold
		pseudoInverse(phi,
			0.0001,
			phiinv, 0);
		UNcBar = ainv * UNc.transpose() * phiinv;
		J1star = J * UNcBar;

		pseudoInverse( J1star * phi * J1star.transpose(),
			0.0001,
			Lambda1, 0);

		ts1.checkElapsed(4);

		desired_pos(0) = 0.4;
		desired_pos(1) = -0.2;
		desired_pos(2) = 0.2;

		desired_posture(5) = M_PI/2.;

		N1 = MatrixXd::Identity(9,9) - phi*J1star.transpose()*Lambda1*J1star;

		J2star = U*UNcBar*N1;
		pseudoInverse(J2star*phi*J2star.transpose(),
			0.0001,
			Lambda2, 0);

		tau1 =  J1star.transpose() * Lambda1 * (kp * ( desired_pos - actual_ ) - kd * J * fullJvel_);
	//	cout << "Lambda2 " << Lambda2.rows() << "x" << Lambda2.cols() << endl;
	//	cout << "N1 " << N1.rows() << "x" << N1.cols() << endl;
	//	cout << "vel " << body_state.position_.rows() << "x" << body_state.position_.cols() << endl;
	//	cout << "UNcBar " << UNcBar.rows() << "x" << UNcBar.cols() << endl;
	//	cout << "U " << U.rows() << "x" << U.cols() << endl;
	//	tau2 =  UNcBar.transpose() * U.transpose() * Lambda2 * (kp * (-body_state.position_) - kd * body_state.velocity_);
		tau2 =  J2star.transpose() * Lambda2 * (kp * ( desired_posture - body_state.position_) - kd * body_state.velocity_ - tau1);
	//	cout << "ainv " << ainv.rows() << "x" << ainv.cols() << endl;
	//	tau		= tau1 + UNcBar.transpose() * grav;
		tau		= tau1 + tau2 + UNcBar.transpose() * grav;
	//	cout << "JAN1 " << endl << J * ainv * UNc.transpose() * N1.transpose() << endl;

	//	cout << "ainv " << ainv.rows() << "x" << ainv.cols() << endl;
	//	cout << "J1star " << J1star.rows() << "x" << J1star.cols() << endl;
	//	cout << "Nc " << NcT.rows() << "x" << NcT.cols() << endl;
	//	cout << "GRAV " << grav.rows() << "x" << grav.cols() << endl;
	//	Vector ddq =  -ainv * NcT * grav;
		ddq = ainv * ( UNc.transpose()*tau - Nc.transpose() * grav );

	//	cout << "ddt " << ddq.rows() << "x" << ddq.cols() << endl;
	//	cout << "jVel " << fullJvel_.rows() << "x" << fullJvel_.cols() << endl;

		fullJvel_ += ddq * dt;
		fullJpos_ += fullJvel_ * dt;

#if 0
		for ( int i = 0 ; i < 9 ; i++ )
		{
			body_state.position_(i) = fullJpos_[qmap[i]];
			body_state.velocity_(i) = fullJvel_[qmap[i]];
			q[qmap[i]] = body_state.position_(i);
		}
		q[3] = fullJpos_[2];
#else
		body_state.position_ = getQa(fullJpos_);
		body_state.velocity_ = getQa(fullJvel_);
		q						= fullJpos_;
		endeffector				= actual_;
#endif
	//	cerr << fullJvel_(1) << " " << fullJvel_(2) << endl;
		ts1.checkElapsed(5);
	}

	{
		static int count = 0;

		if ( (count % 1000) == 0 )
		{
			if ( bSimul )
			{
				cout << ts1;
				cout << "stat " << body_state.position_.transpose() << endl;
				cout << "q " << q[1] << " " << q[2] << endl;
				cout << "actual_ " << endl << actual_.transpose() << endl;
				cout << "full " << fullJpos_.transpose() << endl;
				cout << "full " << (fullJpos_*180./M_PI).transpose() << endl;
				cout << "tau1 " << tau1.transpose() << endl;
				cout << "tau2 " << tau2.transpose() << endl;
				Vector ddx = J * ddq; // ainv * UNc.transpose() * tau;
				cout << "ddx " << ddx.transpose() << endl;
				cout << "ddq " << ddq.transpose() << endl;
				cout << "J " << J << endl;
//				cout << "U " << U << endl;
			}
		}
		count++;
	}

	pthread_mutex_lock(&link_mutex);
	if ( qp1.size() > 2 )
	{
		int idx = (int)seq;
		double ratio = seq - idx;
		VectorXd q0, q1, Q;

		if ( idx < qp1.size()-1 )
		{
			q0 = qp1[idx];
			q1 = qp1[idx+1];

			Q = (1.-ratio) * q0 + ratio * q1;
		}
		else
			Q = qp1[idx];
		sentQ = Q;
	}
	pthread_mutex_unlock(&link_mutex);

	if ( bSend && qp1.size() > 2 )
	{
		Message msg;
		double qa[9];
		VectorXd qav;

		qav = getQa(sentQ);
		
		for ( int i = 0 ; i < 9 ; i++ )
		{
			qa[i] = qav(i);
//			cerr << q[i] << " ";
		}
//		cerr << endl;

		msg.timeStamp = 0;
		msg.data	= (void *)qa;
		msg.size	= sizeof(qa);

			
		pCmdComm->send(&msg);
//		cout << "JPos: " << endl << body_state.position_ << endl;
	}
}

#define STEP (M_PI*0.020) // Resolution 0.02PI = 3.6 deg
//double mins[] = {	-80.,	-10,	-70,	 -10,	-60,	  0,	-20,	-40,	-40};
//double maxs[] = {	 80.,	 40,	180,	130,	 60,	100,	200,	 40,	 40};
Vector Mins;
Vector Maxs;
// Nominal Joint Limits
double mins[] = {	-90.,	-12,	-80,	-25,	-85,	  0,	-48,	-60,	-60};
double maxs[] = {	 90.,	 43,	200,	150,	 85,	133,	230,	 60,	 60};
// Actual Joint Limits
//double mins[] = {	-90.,	-22,	-80,	-23,	-85,	  0,	-48,	-60,	-60};
//double maxs[] = {	 90.,	 49,	200,	150,	 85,	133,	230,	 60,	 60};
template <int T>
double project2( const Node<T> *p, Node<T> *np );

template <int T>
double project1( Node<T> &np );

void *plan(void *)
{
	fprintf(stderr, "Planning Thread Started...\n");
	int i;
	Mins = Maxs = VectorXd::Zero(9);
	for ( i = 0 ; i < 9 ; i++ )
	{
		Mins(i) = mins[i] * M_PI / 180.;
		Maxs(i) = maxs[i] * M_PI / 180.;
	}
	cerr << "PRM init" << endl;
	prm = new PRM<9>(Mins, Maxs, STEP);
	cerr << "PRM init done" << endl;

	while (1)
	{
		int next_count = 1000;

		rrt = new WbcRRT(Mins, Maxs, STEP);
		prm->init();
//		rrt = new WbcRRT<9>(-M_PI, M_PI, STEP);
#if 0
		Node<9> *newNode = new Node<9>;

		newNode->q = body_state.position_;
		newNode->getProjection();
		newNode->parent = NULL;
		rrt->nodes[rrt->numNodes++] = newNode;
#else
		rrt->reset();
		rrt->nodes[0]->q = getQa(q); // body_state.position_;
		for ( int i = 0 ; i < 9 ; i++ )
		{
			rrt->qs[0][i] = rrt->nodes[0]->q(i);
		}
		
		((WbcNode *)rrt->nodes[0])->getProjection();
#endif


		int count = 0;
		q0 = getQ(body_state.position_);
		r0.clear();
		pthread_mutex_lock(&link_mutex);
		for ( int j = 0 ; j < 10 ; j++ )
		{
			myJoint[j].setTheta(q[j]);
		}
		for ( int j = 0 ; j < 10 ; j++ )
		{
			r0.push_back(myJoint[j].getGlobalPos(myJoint[j].com));
		}
		pthread_mutex_unlock(&link_mutex);
		fprintf(stderr, "Start Planning... %d,%d\n", rrt->numNodes, rrt->numEdges);
		cerr << q0.transpose() << endl;
		while (bPlan)
		{
			bPlanning = true;

			rrt->iterate();
//			prm->addNode(project1);

	//		usleep(1000);
			{
				if ( rrt->numNodes > next_count )
				{
					fprintf(stderr, "%d nodes added\n", rrt->numNodes);	
					WbcNode *p = (WbcNode *)(rrt->nodes[rrt->numNodes-1]);

					cerr << p << endl;
					cerr << p->q.transpose() << endl;
					cerr << "Actual " << p->actual.transpose() << endl;
					cerr << *rrt << endl;

					pthread_mutex_lock(&mutex);
					qp1.clear();
					qp1.push_back(getQ(p->q));
					pthread_mutex_unlock(&mutex);
					next_count += 1000;
				}
				count++;

			}
		}
		bPlanning = false;
		pushing = false;

		cerr << "Total " << rrt->numNodes << " added" << endl;
		bRandom = false;

		while (!bPlan)
		{
			if ( bRandom && rrt->numNodes > 1 )
			{
				Timestamp ts1("Path Planning Time");
				ts1.setMode(Timestamp::MODE_SHOW_MSEC | Timestamp::MODE_OVERLAP);
				ts1.setBaseline();

				pthread_mutex_lock(&mutex);
				qp1.clear();
				qp2.clear();
				dist.clear();
				distp.clear();

#define dD (0.01)
				int from = 0;
	//			int from = (int)((double)rand() * (double)rrt->numNodes / RAND_MAX);
				int to;
				
				if ( goalIdx < 0 )
					to = (int)((double)rand() * (double)rrt->numNodes / RAND_MAX);
				else
					to = goalIdx;
//					from = rrt->numNodes-1;
//					to = 0;
				fprintf(stderr, "NODE %d->%d/%d\n", from, to, rrt->numNodes);
				WbcPath path(rrt->nodes[from], rrt->nodes[to]);
				path.step = 0.01*M_PI;
				path.optimize(100);
	//			path.optimize(NULL, 10);

				ts1.checkElapsed(0);
				Node<9> *p = rrt->nodes[rrt->numNodes-1];
				cerr << "Optimal path " << path.numNewNode << " nodes" << endl;
				for ( int i = 0 ; i < path.numNewNode ; i++ )
				{
					VectorXd q_plan= path.newNodes[i]->q;
					VectorXd Q;
#if 0
					Q = VectorXd::Zero(10);
					for ( int i = 0 ; i < 9 ; i++ )
					{
						Q(qmap[i]) = q_plan(i);
					}
					Q(2) = Q(1);
#else
					Q = getQ(q_plan);
#endif
					qp1.push_back(Q);
					((WbcNode *)path.newNodes[i])->getProjection();
//					cerr << getPotential(push_type, elbow0, (const WbcNode&)*(path.newNodes[i]), r0) << endl;

					// Derive elbow location to get distance
					pthread_mutex_lock(&link_mutex);
					for ( int j = 0 ; j < 10 ; j++ )
					{
						myJoint[j].setTheta(Q[j]);
					}
					VectorXd elbow = myJoint[5].getGlobalPos(VectorXd::Zero(3));
					VectorXd dv;
					double dd;
					dv = elbow - elbow0;
					dd = dv.norm();
					dist.push_back(dd);
					if ( distp.size() > 0 && dd < distp.back() + dD ) 
						distp.push_back(distp.back() + dD);
					else
						distp.push_back(dd);
					pthread_mutex_unlock(&link_mutex);
				}
				int i;

				for ( i = 0 ; i < dist.size() ; i++ )
					cerr << dist[i] << endl;
				cerr << endl << endl;
				for ( i = 0 ; i < distp.size() ; i++ )
					cerr << distp[i] << endl;
				ts1.checkElapsed(1);

				double d_max = dist.back();
				d_t.clear();
				d_t.resize(500);
				eta.clear();
				eta.resize(500);
				for ( i = 0 ; i < 500 ; i++ )
				{
					double t = i * 0.01;
					d_t[i] = d_max * ( 1. - exp(-1.5*t));
				}

				eta[0] = 0;
				int index = 1;
				for ( i = 1 ; i < 500 ; i++ )
				{
					double d = d_t[i];

					while ( index < 500 && d > distp[index] )
						index++;

					eta[i] = 0.;
					if ( index < 499 )
					{
						double difference = d - distp[index-1];
						double gap = distp[index] - distp[index-1];
						double ratio = difference / gap;

						if (gap <= 0. && ( ratio >= 1 || ratio < 0 ) )
						{
							assert(0);
						}

						eta[i] = index + ratio - 1;
					}
					else
						eta[i] = 500-1;
						
				}

#if 1
				cerr << "d_t" << endl;
				for ( i = 0 ; i < 500 ; i++ )
				{
					int idx = (int)eta[i];
					double ratio = eta[i] - idx;
					double value1, value2;

					if ( index < 499 )
					{
						value1 = (1.-ratio) * dist[idx] + ratio * dist[idx+1];
						value2 = (1.-ratio) * distp[idx] + ratio * distp[idx+1];
					}
					else
					{
						value1 = dist[idx];
						value2 = distp[idx];
					}

					VectorXd q0, q1, Q;

					if ( idx < qp1.size()-1 )
					{
						q0 = qp1[idx];
						q1 = qp1[idx+1];

						Q = (1.-ratio) * q0 + ratio * q1;
					}
					else
						Q = qp1[idx];

					pthread_mutex_lock(&link_mutex);
					for ( int j = 0 ; j < 10 ; j++ )
					{
						myJoint[j].setTheta(Q[j]);
					}
					VectorXd elbow = myJoint[5].getGlobalPos(VectorXd::Zero(3));
					pthread_mutex_unlock(&link_mutex);


					VectorXd dv;
					double dd;
					dv = elbow - elbow0;
					dd = dv.norm();

					cerr << eta[i] << "  " << value1 << " " << value2 << " " << dd << endl;
				}
#else
				cerr << "eta" << endl;
				for ( int i = 0 ; i < 500 ; i++ )
					cerr << eta[i] << endl;
#endif

				cerr << "Original path " << path.numNode << " nodes" << endl;
				for ( int i = 0 ; i < path.numNode ; i++ )
				{
					VectorXd q_plan= path.nodes[i]->q;
					VectorXd Q;
#if 0
					Q = VectorXd::Zero(10);
					for ( int i = 0 ; i < 9 ; i++ )
					{
						Q(qmap[i]) = q_plan(i);
					}
					Q(3) = Q(2);
#else
					Q = getQ(q_plan);
#endif
					qp2.push_back(Q);
#if 0
					cerr << value[(*it).index] << " > ";

					double val = getPotential(0, VectorXd::Zero(3), Q, r0);
					cerr << "(" << val << ")"; 
#endif
					((WbcNode *)path.nodes[i])->getProjection();
					cerr << getPotential(push_type, elbow0, (const WbcNode&)*(path.nodes[i]), r0) << endl;
				}
				pthread_mutex_unlock(&mutex);
				bRandom = false;
				ts1.checkElapsed(2);

				cerr << ts1;
			}
			usleep(10);
		}

		delete rrt;
		cerr << "Reset Plan" << endl;
	}
}


template <int T>
double project1( Node<T> *np )
{
	Vector dq0;
	Vector dq;
	Vector dst = np->q;

	assert(np->q.rows() == T);

	int trial;
	double err = 1.e10;

	bool violate = true;
	for ( trial = 100 ; trial >= 0 ; trial-- )
	{
		np->getProjection();


//	cerr << "New Node" << np->q.transpose()*180./M_PI << endl;


		Vector diff1 = desired_pos - np->actual;
		err = diff1.norm();

//	cerr << p.q.transpose() << endl;
//	cerr << err << endl;
#if 1
		if ( err <= 0.01 && !violate )
			break;

		cerr << err << "->";

		Vector modified;
//		cerr << "From  : " << p.q.transpose()<<endl;
//		cerr << "Before: " << np->q.transpose()<<endl;
//		modified = np->q + np->traction * diff1 + np->projection * diff2;
		modified = np->q + np->traction * diff1;
		np->q = modified;

		Vector diff2 = prm->center - np->q;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( diff2[i] > M_PI )
				diff2[i] -= 2.*M_PI;
			if ( diff2[i] < -M_PI )
				diff2[i] += 2.*M_PI;
		}
		np->q = prm->center - diff2;
		np->getProjection();
		MatrixXd weight = prm->weight;
		violate = false;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( (np->q(i) < Mins[i]) || (np->q(i) > Maxs[i]) )
			{
				violate = true;
				break;
			}
			else
			{
				weight(i,i) = 0.;
			}
		}
		modified = np->q + np->projection * weight * diff2;
		np->q = modified;
//		cerr << "After : " << np->q.transpose()<<endl;
//		np->getProjection();
#endif
	}
	if ( err > 0.01 || violate )
	{
		cerr << "Failed" << endl;
		return -1.;
	}

	cerr << "Found" << endl;

#if 0
	Vector diff = np->q - prm->center;
	for ( int i = 0 ; i < T ; i++ )
	{
		if ( diff[i] > M_PI )
			diff[i] -= 2.*M_PI;
		if ( diff[i] < -M_PI )
			diff[i] += 2.*M_PI;
	}
	np->q = prm->center + diff;

	violate = true;
	MatrixXd weight = prm->weight;
	for ( trial = 100 ; trial >= 0 && violate == true; trial-- )
	{
		Vector modified;
		Vector diff2;


		np->getProjection();
		diff2 = prm->center - np->q;

		modified = np->q + np->projection * weight * diff2;
		np->q = modified;

		violate = false;
		Vector q = np->q;
		weight = prm->weight;
		for ( int i = 0 ; i < T ; i++ )
		{
			if ( (q(i) < Mins[i]) || (q(i) > Maxs[i]) )
			{
				violate = true;
				break;
			}
			else
			{
				weight(i,i) = 0.;
			}
		}
	}
#if 1
	if ( violate )
	{
		cerr << "Mins  : " << Mins.transpose()*180./M_PI << endl;
		cerr << "EXCEED: " << np->q.transpose()*180./M_PI << endl;
		cerr << "Maxs  : " << Maxs.transpose()*180./M_PI << endl;
		err = -1.;
	}
#endif
#endif

	return err;
}
template <int T>
double project2( const Node<T> *_p, Node<T> *_np )
{
	WbcNode *p, *np;
	Vector dq0;
	Vector dq;
	Vector dst;

	p = (WbcNode *)_p;
	np = (WbcNode *)_np;

	dst = np->q;

	assert(np->q.rows() == T);
	assert(p->q.rows() == T);
	dq0 = np->q - p->q;
//	double mag0 = dq0.norm();

	dq = p->projection * dq0;

	double mag = dq.norm();

//	if ( mag < STEP && mag < mag0/2. )
//		return -1.;

	if ( dq0.transpose() * dq < 0)
	{
		cerr << "ERROR!!!: " << dq0.transpose() << ":" << dq.transpose() << endl;
		cerr << p->projection << endl;
		assert(0);
	}

	dq = dq * STEP / mag;

	np->q	= p->q + dq;

	VectorXd qdeg = np->q * 180. / M_PI;
#ifdef CHECK_LIMIT
	for ( int i = 0 ; i < T ; i++ )
	{
#if 0
		if ( qdeg(i) < mins[i] ) 
			np->q(i) = mins[i]*M_PI/180.;
		if ( qdeg(i) > maxs[i] )
			np->q(i) = maxs[i]*M_PI/180.;
#else
		if ( (qdeg(i) < mins[i]) || (qdeg(i) > maxs[i]) )
		{
//			cerr << "FRM:    " << p.q.transpose()*180./M_PI << endl;
//			cerr << "EXCEED: " << qdeg.transpose() << endl;
//			cerr << "DES:    " << dst.transpose()*180./M_PI << endl;
			return -1.;
		}

#endif
	}
#endif
//	cerr << "New Node" << np->q.transpose()*180./M_PI << endl;

	np->getProjection();

	Vector diff = desired_pos - np->actual;
	double err = diff.norm();


//	cerr << p.q.transpose() << endl;
//	cerr << err << endl;
	if ( err > 0.05 )
	{

		cerr << "New   : " << np->actual.transpose() << endl;
		cerr << "Error : " << diff.transpose() << endl;
		cerr << "DES   : " << desired_pos.transpose() << endl;

		return -2.;
	}
#if 1
	else if ( err > 0.01 )
	{
		Vector modified;
//		cerr << "From  : " << p.q.transpose()<<endl;
//		cerr << "Before: " << np.q.transpose()<<endl;
		modified = np->q + np->traction * diff;
		np->q = modified;
//		cerr << "After : " << np.q.transpose()<<endl;
//		np->getProjection();
	}
#endif

	return mag;
}

//VectorXd pushError;
int pushType;
Timestamp pushTimestamp;
void push(VectorXd dir)
{
	int i, j ;

	if ( bPlanning )
		return;

	pushTimestamp = Timestamp("Push Check");
	pushTimestamp.setBaseline();
	pushTimestamp.setMode(Timestamp::MODE_OVERLAP | Timestamp::MODE_SHOW_MSEC);

	if ( !rrt || rrt->numNodes == 0 )
		return;

	if ( dir(0) > 0.03 )
		push_type = 1;
	else if ( dir(1) < -0.03 )
		push_type = 2;
	else if ( dir(2) > 0.03 )
		push_type = 3;
	else
		return;
	pushType = push_type;

	VectorXd contact = elbow0;

	cerr << "Start Pushing... " << contact.transpose() << " : " << dir.transpose() << " " << push_type << endl;

	double	max = -100000.;
	int		max_idx = 0;

	cerr << contact.transpose() << endl;
	cerr << q0.transpose() << endl;
	value = (double *)malloc(sizeof(double)*rrt->numNodes);

	Timestamp ts0("Total Time"), ts1("Check Potential");
	ts0.setMode(Timestamp::MODE_SHOW_SEC);
	ts1.setMode(Timestamp::MODE_SHOW_USEC | Timestamp::MODE_SHOW_COUNT);
	ts0.setBaseline();
	pthread_mutex_lock(&mutex);
	ts0.checkElapsed(0);
	for ( i = rrt->numNodes-1 ; i >= 0 ; i-- )
//	for ( i = 0 ; i < rrt->numNodes ; i++ )
	{
		ts1.setBaseline();
		ts0.checkElapsed(1);
		WbcNode *pNode = (WbcNode *)rrt->nodes[i];
		VectorXd Q = getQ(pNode->q);

		double sum = getPotential(push_type, contact, *pNode, r0);
		value[i] = sum;
		
		if ( sum > max )
		{
			max		= sum;
			max_idx	= i;
//			cerr << "MAX " << max_idx << ": " << max <<endl;
//			cerr << Q.transpose() << endl;
//			cerr << q0.transpose() << endl;
		}
		ts1.checkElapsed(0);
	}
	ts0.checkElapsed(2);
	pthread_mutex_unlock(&mutex);
	cerr << "MAX " << max_idx << ": " << max << " " << i << endl;
	cerr << ts0;
	cerr << ts1;

	bRandom = true;;
	goalIdx = max_idx;
	tickCount = 0;
}

double sgn(double in)
{
	if ( in > 0. )
		return 1.;
	else if ( in < 0. )
		return -1.;
	return 0.;
}

bool getPotentialVerbose = false;
double getPotential(int mode, const VectorXd &contact, const WbcNode &node, const vector<VectorXd>& r0)
{
	int j;
	double sum = 0.;
	double weight[10] = {0.};
	int mask[10] = {0};
	double value[10];

	weight[5] = 12.;
	weight[6] = 12.;
	weight[7] = 12.;
	mask[5] = 1;
	mask[6] = 1;
	mask[7] = 1;
//	VectorXd r[10];


#if 0
	pthread_mutex_lock(&link_mutex);
	VectorXd q = getQ(node.q);
	for ( j = 0 ; j < 10 ; j++ )
	{
		myJoint[j].setTheta(q[j]);
	}
	for ( j = 0 ; j < 10 ; j++ )
	{
		r[j] = myJoint[j].getGlobalPos(myJoint[j].com);
	}
	pthread_mutex_unlock(&link_mutex);
#endif

	for ( j = 0 ; j < 10 ; j++ )
	{
		VectorXd d = node.coms[j] - contact;
//		VectorXd d = r[j] - contact;
		
		if ( mask[j] )
		{
			switch (mode)
			{
			case 1:
				value[j] = weight[j] * d(0) * myJoint[j].mass; // X-
				break;
			case 2:
				value[j] = - weight[j] * d(1) * myJoint[j].mass; // Y-
				break;
			case 3:
				value[j] = weight[j] * d(2) * myJoint[j].mass; // Z+
				break;
			}
			if ( getPotentialVerbose )
			{
				cerr << d.transpose() << ":" << myJoint[j].mass << "(" << value[j] << ")" << endl;
			}
		}
		else
		{
			value[j] = - sqrt(((r0[j] - node.coms[j]).transpose() * (r0[j] - node.coms[j]))(0)) * myJoint[j].mass;
//			value[j] = - sqrt(((r0[j] - r[j]).transpose() * (r0[j] - r[j]))(0)) * myJoint[j].mass;
			if ( getPotentialVerbose )
			{
				cerr << (r0[j]-node.coms[j]).transpose() << ":" << myJoint[j].mass << "(" << value[j] << ")" << endl;
//				cerr << (r0[j]-r[j]).transpose() << ":" << myJoint[j].mass << "(" << value[j] << ")" << endl;
			}
		}
	}

	sum = 0;
	for ( j = 0 ; j < 10 ; j++ )
	{
//		if ( getPotentialVerbose )
//			cerr << value[j] << ":" << myJoint[j].mass << " ";
		sum += value[j];
	}
//	if ( getPotentialVerbose )
//		cerr << endl;

	return sum;
}

