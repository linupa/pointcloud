#ifndef __MY_SIM_H__
#define __MY_SIM_H__

#include <vector>

#include <FL/gl.h>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl.H>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <FL/fl_draw.H>

extern int win_width;
extern int win_height;
extern double hor;
extern double ver;
extern double scale;

using namespace std;

typedef struct {
	double x;
	double y;
	double z;
} Pos;

extern double	high_z;
extern double	farthest;
extern double	min_rad, max_rad;

class MySim : public Fl_Gl_Window
//class MySim : public Fl_Widget
{
public:
	MySim(int xx, int yy, int width, int height);
	void initializeGl(void);
	virtual ~MySim();
    bool firstTime;
	vector<Pos>trj;
	
	virtual void draw();
	virtual void resize(int x, int y, int w, int h);
	virtual void show();

	void tick();
	
#if 1
	int handle(int e);
#endif
};

class MyWindow : public Fl_Double_Window 
{
public:
	MyWindow(int width, int height, const char *title);

	virtual void resize(int x, int y, int w, int h);	

	Fl_Button *planButton;
	Fl_Button *randomButton;
	Fl_Button *modeButton;
	Fl_Button *pathButton;
	Fl_Button *learnBaseButton;
	Fl_Button *sendButton;
	Fl_Button *quitButton;
	Fl_Button *pushXButton;
	Fl_Button *pushYButton;
	Fl_Button *pushZButton;
	Fl_Button *goalButton;
	Fl_Button *interveneButton;
	MySim *sim;
	Fl_Value_Slider *mGroupSlider;
	Fl_Value_Slider *mSizeSlider;
	Fl_Value_Slider *mSliceSlider;
	Fl_Value_Slider *mScaleSlider;
	Fl_Value_Slider *mNodeSlider;
	Fl_Value_Slider *mSeqSlider;
	Fl_Value_Slider *mHighlightSlider;
	~MyWindow(void);


	static void cb_group(Fl_Widget *widget, void *param);
	static void cb_link2(Fl_Widget *widget, void *param);
	static void cb_floor(Fl_Widget *widget, void *param);
	static void cb_phi(Fl_Widget *widget, void *param);

	static void cb_track(Fl_Widget *widget, void *param);
	static void cb_showBase(Fl_Widget *widget, void *param);
	static void cb_learnBase(Fl_Widget *widget, void *param);
	static void cb_random(Fl_Widget *widget, void *param);
	static void cb_iterate(Fl_Widget *widget, void *param);

	static void cb_highlight(Fl_Widget *widget, void *param);

	static void cb_slice(Fl_Widget *widget, void *param);
	static void cb_scale(Fl_Widget *widget, void *param);

	static void cb_mode(Fl_Widget *widget, void *param);
	static void cb_control(Fl_Widget *widget, void *param);
	static void cb_speed(Fl_Widget *widget, void *param);

	static void cb_send(Fl_Widget *widget, void *param);
	static void cb_plan(Fl_Widget *widget, void *param);
	static void cb_path(Fl_Widget *widget, void *param);
	static void cb_simul(Fl_Widget *widget, void *param);
	static void cb_reset(Fl_Widget *widget, void *param);
	static void cb_quit(Fl_Widget *widget, void *param);
	static void cb_pushx(Fl_Widget *widget, void *param);
	static void cb_pushy(Fl_Widget *widget, void *param);
	static void cb_pushz(Fl_Widget *widget, void *param);
	static void cb_push(Fl_Widget *widget, void *param);

	static void cb_seq(Fl_Widget *widget, void *param);
	static void cb_node(Fl_Widget *widget, void *param);

	static void cb_goal(Fl_Widget *widget, void *param);
	static void cb_intervene(Fl_Widget *widget, void *param);

	static void timer_cb(void *param);
};


#endif // _MY_SIM_H_
