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

#include "rrt.hpp"
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

using namespace std;
using namespace boost;
using namespace jspace;

static int win_width(800);
static int win_height(600);
static char win_title[100];
static scoped_ptr<jspace::Model> model;
static scoped_ptr<jspace::Model> model_planning;
State body_state(9, 9, 6);

//void dump_to_stdout( TiXmlNode* pParent, unsigned int indent = 0);
void *plan(void *);

XmlNode baseNode;
bool bSimul = false;
bool bPlan = false;
bool bRandom = false;
int goalIdx = -1;
bool bSend = false;
extern double q[11];
extern vector<VectorXd> qp1;
extern vector<VectorXd> qp2;
static double dt = 0.001;
Vector desired_pos;
extern int tickCount;
extern int seq; 
vector<Link> myLink(10);
RRT<9> *rrt;
extern double *value;

template<int T>
MatrixXd getProjection(Node<T> &p);

pthread_mutex_t	mutex;

Comm *pRecvComm;
Comm *pCmdComm;

// Checking process time
int start_time_sec;
int start_time_usec;
int ts[10][2] = {{0}};
void mark_start_time(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	start_time_sec	= tv.tv_sec;
	start_time_usec	= tv.tv_usec;
}

int get_elapsed(void)
{
	struct timeval tv;

	int usec;
	int sec;

	gettimeofday(&tv, NULL);

	usec	= tv.tv_usec - start_time_usec;
	sec		= tv.tv_sec - start_time_sec;

	return (sec*1000000 + usec); 
}

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

	int i = 0;
	while ( node )
	{
		if ( node->name )
			cerr << node->name << endl;

		Link link;

		link.com = node->com;
		link.setRot(node->rot.block(0,0,3,1), node->rot(3));
		link.trans	= node->pos;
		link.axis	= 2;
		link.mass	= node->mass;
		if ( i > 0 )
			link.parent = &(myLink[i-1]);
		else
			link.parent = NULL;
		myLink[i] = link;

		node = node->childs;
		i++;
	}
	
	cerr << "Change CoM " << myLink[2].mass <<endl;
	myLink[2].mass = 0.;


	glutInit(&argc, argv);

	model.reset(test::parse_sai_xml_file(robot_spec, false));
	model->setConstraint("Dreamer_Torso");

	model_planning.reset(test::parse_sai_xml_file(robot_spec, false));
	model_planning->setConstraint("Dreamer_Torso");

	pthread_mutex_init( &mutex, NULL );
	pthread_create( &planning_thread, NULL, plan, NULL);

	pRecvComm = new Comm(STT_PORT, 20000);
	pRecvComm->listen();

	pCmdComm = new Comm("192.168.1.117", CMD_PORT, 20000);

	MyWindow win(win_width, win_height, "test");

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

int qmap[] = {1,2,4,5,6,7,8,9,10};
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

		msg = pRecvComm->read();
		if ( msg.size > 0 )
		{
			double *pData = (double *)msg.data;
//			fprintf(stderr, "%d data\n", msg.size);
			for ( i = 0 ; i < 9 ; i++ )
			{
				q[qmap[i]] = pData[i];
			}
		//	cerr << fullJvel_(1) << " " << fullJvel_(2) << endl;
			q[3] = q[2];
		}
		for ( i = 0 ; i < 9 ; i++ )
		{
			body_state.position_(i) = q[qmap[i]];
			body_state.velocity_(i) = 0.;
		}

#if 1
		if ( !bPlan )
		{
			Node<9> node;
			node.q = body_state.position_;
			getProjection(node);

			desired_pos = node.actual;
		}
#endif
	}
	else
	{

		mark_start_time();

		model->update(body_state);

		get_elapsed();
		ts[0][0] += get_elapsed();
		ts[0][1] ++;

		fullJpos_ = model->getFullState().position_;
		fullJvel_ = model->getFullState().velocity_;

		model->getInverseMassInertia(ainv);

		model->getGravity(grav);

		Constraint * constraint = model->getConstraint();
		
		constraint->updateJc(*model);
		constraint->getNc(ainv,Nc);
		constraint->getU(U);
		UNc = U*Nc;

		get_elapsed();
		ts[1][0] += get_elapsed();
		ts[1][1] ++;

#if 1
		taoDNode const *end_effector_node_ = model->getNode(9);
		jspace::Transform ee_transform;
		model->computeGlobalFrame(end_effector_node_,
			0.0, -0.05, 0.0, ee_transform);
		actual_ = ee_transform.translation();

		get_elapsed();
		ts[2][0] += get_elapsed();
		ts[2][1] ++;

		model->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
		J = Jfull.block(0, 0, 3, Jfull.cols());
#else
		MatrixXd J = getJacobian(*model);
#endif
		
		get_elapsed();
		ts[3][0] += get_elapsed();
		ts[3][1] ++;

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

		get_elapsed();
		ts[4][0] += get_elapsed();
		ts[4][1] ++;

		desired_pos(0) = 0.3;
		desired_pos(1) = -0.1;
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

		for ( int i = 0 ; i < 9 ; i++ )
		{
			body_state.position_(i) = fullJpos_[qmap[i]-1];
			body_state.velocity_(i) = fullJvel_[qmap[i]-1];
			q[qmap[i]] = body_state.position_(i);
		}
	//	cerr << fullJvel_(1) << " " << fullJvel_(2) << endl;
		q[3] = fullJpos_[2];
		get_elapsed();
		ts[5][0] += get_elapsed();
		ts[5][1] ++;
	}

	{
		static int count = 0;

		if ( (count % 1000) == 0 )
		{
			if ( bSimul )
			{
				fprintf(stderr, "Average Time %f %f %f %f %f %f\n", 
						(double)ts[0][0] / ts[0][1],
						(double)ts[1][0] / ts[1][1],
						(double)ts[2][0] / ts[2][1],
						(double)ts[3][0] / ts[3][1],
						(double)ts[4][0] / ts[4][1],
						(double)ts[5][0] / ts[5][1]);
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
				cout << "U " << U << endl;
			}
		}
		count++;
	}
	if ( bSend && qp1.size() > 2 )
	{
		Message msg;
		double q[9];
		
		for ( int i = 0 ; i < 9 ; i++ )
		{
			q[i] = qp1[seq][qmap[i]];
//			cerr << q[i] << " ";
		}
//		cerr << endl;

		msg.timeStamp = 0;
		msg.data	= (void *)q;
		msg.size	= sizeof(q);
			
		pCmdComm->send(&msg);
	}
//	cout << "JPos: " << endl << body_state.position_ << endl;
}

#define STEP (M_PI*0.020) // Resolution 0.02PI = 3.6 deg
double mins[] = {	-80.,	-10,	-70,	 -10,	-60,	  0,	-20,	-40,	-40};
double maxs[] = {	 80.,	 40,	180,	130,	 60,	100,	200,	 40,	 40};
// Original Joint Limits
//double mins[] = {	-90.,	-12,	-80,	-25,	-85,	  0,	-48,	-60,	-60};
//double maxs[] = {	 90.,	 53,	200,	150,	 85,	133,	230,	 60,	 60};
template <int T>
double project( const Node<T> &p, Node<T> &np );
void *plan(void *)
{
	fprintf(stderr, "Planning Thread Started...\n");
	VectorXd Mins, Maxs;
	int i;
	Mins = Maxs = VectorXd::Zero(9);
	for ( i = 0 ; i < 9 ; i++ )
	{
		Mins(i) = mins[i] * M_PI / 180.;
		Maxs(i) = maxs[i] * M_PI / 180.;
	}

	while (1)
	{
		rrt = new RRT<9>(Mins, Maxs, STEP);
#if 0
		Node<9> *newNode = new Node<9>;

		newNode->q = body_state.position_;
		getProjection(*newNode);
		newNode->parent = NULL;
		rrt->nodes[rrt->numNodes++] = newNode;
#else
		rrt->reset();
		rrt->nodes[0]->q = body_state.position_;
		getProjection(*(rrt->nodes[0]));
#endif

		fprintf(stderr, "Start Planning... %d,%d\n", rrt->numNodes, rrt->numEdges);

		int count = 0;
		while (bPlan)
		{

			rrt->iterate( project );

	//		usleep(1000);
			{

				if ( (count%100) == 0 )
				{
					fprintf(stderr, "%d nodes added\n", rrt->numNodes);	
					Node<9> *p = rrt->nodes[rrt->numNodes-1];

					cerr << p << endl;
					cerr << p->q.transpose() << endl;
					cerr << "Actual " << p->actual.transpose() << endl;
					cerr << *rrt << endl;

					pthread_mutex_lock(&mutex);
					qp1.clear();
	//				do
					{
//						fprintf(stderr, "%p\n", p);
//						cout << p->q << endl;
						VectorXd q_plan= p->q;
						VectorXd q;
						q = VectorXd::Zero(11);
						for ( int i = 0 ; i < 9 ; i++ )
						{
							q(qmap[i]) = q_plan(i);
						}
						q(3) = q(2);
						qp1.push_back(q);
					}
					pthread_mutex_unlock(&mutex);
	//				while ( p->parent && (p = p->parent) );
				}
				count++;

			}
		}

		cerr << "Total " << rrt->numNodes << " added" << endl;

		while (!bPlan)
		{
			if ( bRandom && rrt->numNodes > 1 )
			{
				pthread_mutex_lock(&mutex);
				qp1.clear();
				qp2.clear();

				Path<9> path;
				{
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
					path = Path<9>(rrt->nodes[from], rrt->nodes[to]);
					path.step = 0.01*M_PI;
					path.optimize(project, 100);
		//			path.optimize(NULL, 10);
				}
				Node<9> *p = rrt->nodes[rrt->numNodes-1];
				list<Node<9> >::iterator it = path.newNodes.begin();
				cerr << "Optimal path " << path.newNodes.size() << " nodes" << endl;
				for ( ; it != path.newNodes.end() ; it++ )
				{
					VectorXd q_plan= (*it).q;
					VectorXd q;
					q = VectorXd::Zero(11);
					for ( int i = 0 ; i < 9 ; i++ )
					{
						q(qmap[i]) = q_plan(i);
					}
					q(3) = q(2);
					qp1.push_back(q);
				}

				cerr << "Original path " << path.nodes.size() << " nodes" << endl;
				it = path.nodes.begin();
				for ( ; it != path.nodes.end() ; it++ )
				{
					VectorXd q_plan= (*it).q;
					VectorXd q;
					q = VectorXd::Zero(11);
					for ( int i = 0 ; i < 9 ; i++ )
					{
						q(qmap[i]) = q_plan(i);
					}
					q(3) = q(2);
					qp2.push_back(q);
//					cerr << value[(*it).index] << " > ";
				}
				cerr << endl;
				pthread_mutex_unlock(&mutex);
				bRandom = false;
			}
			usleep(100000);
		}

		delete rrt;
		cerr << "Reset Plan" << endl;
	}
}

template<int T>
MatrixXd getProjection(Node<T> &p)
{
	MatrixXd ainv;
	MatrixXd Nc;
	MatrixXd UNc;
	MatrixXd U;
	MatrixXd UNcBar;
	MatrixXd phi;
	MatrixXd phiinv; 
	MatrixXd J1star;
	MatrixXd J2star;
	MatrixXd Lambda1;
	MatrixXd Lambda2;
	MatrixXd Lambda2P;
	MatrixXd N1;
	MatrixXd J, Jfull;
	Vector actual_;

	Constraint *constraint;

	State state = body_state;
	state.position_ = p.q;
	state.velocity_ = VectorXd::Zero(state.velocity_.rows());

	model_planning->update(state);

	model_planning->getInverseMassInertia(ainv);

	constraint = model_planning->getConstraint();
	constraint->getU(U);
	constraint->updateJc(*model_planning);

	constraint->getNc(ainv,Nc);

	UNc = U*Nc;

#if 1
	taoDNode const *end_effector_node_ = model_planning->getNode(9);
	jspace::Transform ee_transform;
	model_planning->computeGlobalFrame(end_effector_node_,
		0.0, -0.05, 0.0, ee_transform);
	actual_ = ee_transform.translation();

	get_elapsed();
	ts[2][0] += get_elapsed();
	ts[2][1] ++;

	model_planning->computeJacobian(end_effector_node_, actual_[0], actual_[1], actual_[2], Jfull);
	J = Jfull.block(0, 0, 3, Jfull.cols());
#else
	MatrixXd J = getJacobian(*model_planning);
#endif

	phi = UNc * ainv * UNc.transpose();
	//XXXX hardcoded sigma threshold
	pseudoInverse(phi,
		0.0001,
		phiinv, 0);
	UNcBar = ainv * UNc.transpose() * phiinv;

	J1star = J*UNcBar;
	pseudoInverse( J1star * phi * J1star.transpose(),
		0.0001,
		Lambda1, 0);

	N1 = MatrixXd::Identity(9,9) - phi*J1star.transpose()*Lambda1*J1star;
	J2star = U*UNcBar*N1;
	Lambda2P = J2star*phi*J2star.transpose();
	pseudoInverse(Lambda2P,
		0.0001,
		Lambda2, 0);

	p.projection		= Lambda2P * Lambda2;
	p.actual			= actual_;

//	return Lambda2;
	return p.projection;
}

template <int T>
double project( const Node<T> &p, Node<T> &np )
{
	Vector dq0 = np.q - p.q;
	Vector dq;

	assert(np.q.rows() == T);
	assert(p.q.rows() == T);
	dq0 = np.q - p.q;
	double mag0 = dq0.norm();

	dq = p.projection * dq0;

	double mag = dq.norm();

//	if ( mag < STEP && mag < mag0/2. )
//		return -1.;

	if ( dq0.transpose() * dq < 0)
	{
		cerr << "ERROR!!!: " << dq0.transpose() << ":" << dq.transpose() << endl;
	}

	dq = dq * STEP / mag;

	np.q	= p.q + dq;
	np.projection	= getProjection(np);

	Vector desired = Vector::Zero(3);

	Vector diff = desired_pos - np.actual;
	double err = diff.norm();

	for ( int i = 0 ; i < T ; i++ )
	{
		if ( (np.q(i) < mins[i]) || (np.q(i) > maxs[i]) )
			return -1.;
	}

//	cerr << p.q.transpose() << endl;
//	cerr << err << endl;
	if ( err > 0.05 )
	{
		cerr << "Error : " << diff.transpose() << endl;
		cerr << "DES: " << desired_pos.transpose() << endl;
		return -1.;
	}

	return mag;
}

double getPotential(int mode, const VectorXd &contact, const VectorXd &q)
{
	int j;
	double sum = 0.;

	for ( j = 0 ; j < 9 ; j++ )
	{
		myLink[j].theta = q[j];
	}
	for ( j = 0 ; j < 9 ; j++ )
	{
		VectorXd r = myLink[j].getGlobal(myLink[j].com);
		VectorXd d = r - contact;
		
//			sum += d.transpose() * d;
#if 0
		sum += d(2) * myLink[j].mass; // X-
#else
		switch (mode)
		{
		case 0:
			sum -= d(0) * myLink[j].mass; // X-
			break;
		case 1:
			sum -= d(1) * myLink[j].mass; // Y-
			break;
		case 2:
			sum += d(2) * myLink[j].mass; // Z+
			break;
		}
#endif
	}

	return sum;
}

