/*
 * Copyright (C) 1998 Janne Löf <jlof@mail.student.oulu.fi>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the Free
 * Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */


/* Zktor is a word that does not mean anything and is difficult to pronounce. */
/* I apologize for horrible coding. */


#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <glib.h>
#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <gdk/gdkkeysyms.h>
#include <gtkgl/gtkglarea.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

static GLfloat          yellow[4]       = { 0.90, 0.90, 0.00, 1.00 };

void on_IMU_MAG(IvyClientPtr app, void *user_data, int argc, char *argv[]);
void on_IMU_ACCEL(IvyClientPtr app, void *user_data, int argc, char *argv[]);

#define DegOfRad(r) (r * 180. / M_PI)

#define AXIS_X  0
#define AXIS_Y  1
#define AXIS_Z  2
#define AXIS_NB 3


#define REFRESH_RATE 166   /* ms */

float biases[3];
float quat[4];
float eulers[3];
float mag[3];
float accel[3];
float dcm00, dcm01, dcm02, dcm12, dcm22;

int width = 640;
int height = 480;


/* game state */

GTimer *gtimer = NULL;

double game_time;
double game_tick;
int draw_fast = 1;



GLuint fontbase = 0;


void game_init() {
  if (!gtimer)
    gtimer = g_timer_new();
  g_timer_reset(gtimer);

  game_time = g_timer_elapsed(gtimer, NULL);
  game_tick  = 1.0 / 60;
}

void game_play()
{
  int i;
  double time_now,tick_now;

  /* timing */
  time_now = g_timer_elapsed(gtimer, NULL);
  tick_now = time_now - game_time;
  if (tick_now < 0.001) tick_now = 0.001;
  if (tick_now > 0.2  ) tick_now = 0.2;
  game_tick = (tick_now + 4*game_tick)/5; /* average */
  game_time = time_now;


}

void quat_to_euler( gfloat* quat, gfloat* euler) {
  float q02 = quat[0] * quat[0];
  float q12 = quat[1] * quat[1];
  float q22 = quat[2] * quat[2];
  float q32 = quat[3] * quat[3];

  euler[0] = atan2( 2*(quat[2]*quat[3] + quat[0]*quat[1]),(q02 - q12 - q22 + q32));
  euler[1] = -asin(2*(quat[1]*quat[3] - quat[0]*quat[2]));
  euler[2] = atan2( 2*(quat[1]*quat[2] + quat[0]*quat[3]),(q02 + q12 - q22 - q32));
}


float ahrs_heading_of_mag( const float* mag ) {

  const float cphi  = cos( eulers[0] );
  const float sphi  = sin( eulers[0] );
  const float ctheta  = cos( eulers[1] );
  const float stheta  = sin( eulers[1] );

  const float mn =
    ctheta*      mag[0]+
    sphi*stheta* mag[1]+
    cphi*stheta* mag[2];
  const float me =
    0*     mag[0]+
    cphi*  mag[1]+
    -sphi* mag[2];

  const float heading = -atan2( me, mn );
  return heading;
}

float ahrs_roll_of_accel( const float* accel ) {
  return atan2(accel[AXIS_Y], accel[AXIS_Z]);
}

float ahrs_pitch_of_accel( float* accel) {
  float g2 =
    accel[AXIS_X]*accel[AXIS_X] +
    accel[AXIS_Y]*accel[AXIS_Y] +
    accel[AXIS_Z]*accel[AXIS_Z];
  return -asin( accel[AXIS_X] / sqrt( g2 ) );
}

float compute_euler_heading( void ) {
  return atan2( dcm01, dcm00 );
}

void compute_DCM( void ) {
  dcm00 = 1.0-2*(quat[2]*quat[2] + quat[3]*quat[3]);
  dcm01 =     2*(quat[1]*quat[2] + quat[0]*quat[3]);
  dcm02 =     2*(quat[1]*quat[3] - quat[0]*quat[2]);
  dcm12 =     2*(quat[2]*quat[3] + quat[0]*quat[1]);
  dcm22 = 1.0-2*(quat[1]*quat[1] + quat[2]*quat[2]);
}


void draw_heli ( void ) {
  /* draw heli */
  glColor3f(1.0, 1.0, 0.0);
  glPushMatrix();
  glRotatef(DegOfRad(eulers[AXIS_X]), 1., 0., 0.);
  glRotatef(DegOfRad(eulers[AXIS_Y]), 0., 1., 0.);
  glRotatef(DegOfRad(eulers[AXIS_Z]), 0., 0., 1.);
  glBegin( GL_LINES );
  glVertex3f(  0.00,  1.,    0. );
  glVertex3f(  10.00,  0.0,  0. );
  glVertex3f(  0.00,  -1.,    0. );
  glVertex3f(  10.00,  0.0,  0. );
  glVertex3f(  0.00,  1.,    0. );
  glVertex3f(  0.00,  -1.0,  0. );
  glVertex3f(  0.00,  0.,    0. );
  glVertex3f(  10.00,  0.,  0. );
  glVertex3f(  0.00,  0.,    1. );
  glVertex3f(  10.00,  0.,  0. );
  glVertex3f(  0.00,  0.,    0. );
  glVertex3f(  0.00,  0.,  1. );
  glEnd();
  glPopMatrix();


  /* untilt mag reading */
  float ahrs_mag_heading = ahrs_heading_of_mag(mag);
  float ahrs_filt_heading = eulers[2];

  /* display unfiltered mag heading */
  glColor3f(1., 0., 0.);
  glPushMatrix();
  glRotatef(DegOfRad(ahrs_mag_heading) , 0., 0., 1.);
  glBegin( GL_LINES );
  glVertex3f( 0., 0., 0. );
  glVertex3f( 5., 0 , 0);
  glEnd();
  glPopMatrix();


  /* display filtered heading */
  glColor3f(0., 1., 1.);
  glPushMatrix();
  glRotatef(DegOfRad(ahrs_filt_heading) , 0., 0., 1.);
  glBegin( GL_LINES );
  glVertex3f( 0., 0., 0. );
  glVertex3f( 5., 0 , 0);
  glEnd();
  glPopMatrix();

  /* draw mag vector */
  glColor3f(0., 1.0, 1.0);
  glPushMatrix();
  glRotatef(DegOfRad(eulers[0]), 1., 0., 0.);
  glRotatef(DegOfRad(eulers[1]), 0., 1., 0.);
  glRotatef(DegOfRad(eulers[2]), 0., 0., 1.);
  float xmag = mag[0]/30.;
  float ymag = mag[1]/30.;
  float zmag = mag[2]/30.;
  glBegin( GL_LINES );
  glVertex3f( 0., 0., 0. );
  glVertex3f( xmag, ymag, zmag );
  glEnd();
  glColor3f(0.5, 0.5, 0.5);
  glBegin( GL_LINES );
  glVertex3f( xmag, ymag, zmag );
  glVertex3f( xmag, ymag, 0 );
  glVertex3f( xmag, ymag, 0 );
  glVertex3f( 0., 0., 0. );
  glEnd();
  glPopMatrix();


  /* draw accel */
  float ahrs_accel_phi = DegOfRad(ahrs_roll_of_accel(accel));
  float ahrs_accel_theta = DegOfRad(ahrs_pitch_of_accel(accel));

  glColor3f(0.6, .6, 1.0);
  glPushMatrix();
  glRotatef(ahrs_accel_phi, 1., 0., 0.);
  glRotatef(ahrs_accel_theta, 0., 1., 0.);
  //  glRotatef(DegOfRad(eulers[2]), 0., 0., 1.);
  glBegin( GL_LINES );
  glVertex3f( 0., 0., 0. );
  glVertex3f( accel[0]/4., accel[1]/4., accel[2]/4. );
  glEnd();
  glPopMatrix();

  printf("mag heading %f  ac heading %f err %f\n\n", DegOfRad(ahrs_mag_heading), DegOfRad(eulers[2]),
	 DegOfRad(ahrs_mag_heading) - DegOfRad(eulers[2]));


}

#define CAM_POS 10.,10.,0.
//#define CAM_POS 0.,-10., -30.
#define HELI_POS 0., 0., 0.
//#define UP 0, 1, 0
#define UP -1, 0, 0

void game_render()
{
  int i;

  /* drawmode */
  if (draw_fast) {
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_POINT_SMOOTH);
    glDisable(GL_BLEND);
  } else {
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  }


  glClearColor(0., 0., 0.2, 0.);
  glClear(GL_COLOR_BUFFER_BIT);

  glViewport(0,0,width,height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // glOrtho(-10.0, 10.0, -10.0, 10.0, -50.0, 50.0);
  gluPerspective(45.0f,(GLfloat)width/(GLfloat)height,1.0f,1000.0f);
  glEnable(GL_DEPTH_TEST);
  glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);



  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  //  glTranslatef(HELI_POS);
  gluLookAt(CAM_POS, HELI_POS, UP);

#define NED
  /* switch to nort east down */
#ifdef NED
  glPushMatrix();
  glRotatef(90., 1., 0., 0.);
  glRotatef(180., 0., 0., 1.);
#endif

  /* draw frame */

  quat_to_euler(quat, eulers);

  glColor3f(1.0, 0.0, 0.0);
  glBegin( GL_LINES );
  glVertex3f(  0.,  0.,   0. );
  glVertex3f(  10., 0.,   0. );
  glEnd();
  glColor3f(.0, 1.0, 0.0);
  glBegin( GL_LINES );
  glVertex3f(  0.,  0.,   0. );
  glVertex3f(  0., 10.,   0. );
  glEnd();
  glColor3f(0.0, 0.0, 0.9f);
  glBegin( GL_LINES );
  glVertex3f(  0.,  0.,   0. );
  glVertex3f(  0.,  0.,  10. );
  glEnd();

  compute_DCM();

  draw_heli();


#ifdef NED
  glPopMatrix();
#endif

/*   glViewport(0,0, width, height); */
/*   glMatrixMode(GL_PROJECTION); */
/*   glLoadIdentity(); */
/*   gluOrtho2D(0,100, 100,0); */
/*   glMatrixMode(GL_MODELVIEW); */
/*   glLoadIdentity(); */

  /* textual info */
/*   if (fontbase) {  */
/*     char s[200]; */
/*     g_snprintf(s, sizeof(s), "magnetometer : mx -> %d, my -> %d, mz -> %d", mx, my, mz); */

/*     glColor3f(.8,.8,.8); */
/*     glRasterPos2f(-90, 90); */
/*     glListBase(fontbase); */
/*     glCallLists(strlen(s), GL_UNSIGNED_BYTE, s); */

/*   } */
}


gint init(GtkWidget *widget)
{
  /* OpenGL functions can be called only if makecurrent returns true */
  if (gtk_gl_area_make_current(GTK_GL_AREA(widget))) {
    /* generate font display lists */
    GdkFont *font;
    font = gdk_font_load("-adobe-helvetica-medium-r-normal--*-120-*-*-*-*-*-*");
    if (font) {
      fontbase = glGenLists( 128 );
      gdk_gl_use_gdk_font(font, 0, 128, fontbase);
      gdk_font_unref(font);
    }
  }
  return TRUE;
}


/* When widget is exposed it's contents are redrawn. */
gint draw(GtkWidget *widget, GdkEventExpose *event)
{
  /* Draw only last expose. */
  if (event->count > 0)
    return TRUE;

  if (gtk_gl_area_make_current(GTK_GL_AREA(widget)))
    game_render();

  /* Swap backbuffer to front */
  gtk_gl_area_swapbuffers(GTK_GL_AREA(widget));

  return TRUE;
}

/* When glarea widget size changes, viewport size is set to match the new size */
gint reshape(GtkWidget *widget, GdkEventConfigure *event)
{
  /* OpenGL functions can be called only if make_current returns true */
  if (gtk_gl_area_make_current(GTK_GL_AREA(widget)))
    {
      glViewport(0,0, widget->allocation.width, widget->allocation.height);
    }
  return TRUE;
}


gint animate(GtkWidget *glarea)
{
  game_play();
  gtk_widget_draw(GTK_WIDGET(glarea), NULL);
  return TRUE;
}

void on_IMU_MAG(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  int i;
  //  for (i=0; i< argc; i++)
  //    printf("[%s] ", argv[i]);
  //  printf("\n");

  float mx = atoi(argv[0]);
  float my = atoi(argv[1]);
  float mz = atoi(argv[2]);

  mag[0] = mx;
  mag[1] = my;
  mag[2] = mz;

  //  printf("mag %d %d %d\n", mx, my, mz);
}
void on_IMU_ACCEL(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  int i;
  float ax = atoi(argv[0]);
  float ay = atoi(argv[1]);
  float az = atoi(argv[2]);

  accel[0] = ax;
  accel[1] = ay;
  accel[2] = az;
}

void on_AHRS_STATE(IvyClientPtr app, void *user_data, int argc, char *argv[]){
  float q0 = atof(argv[0]);
  float q1 = atof(argv[1]);
  float q2 = atof(argv[2]);
  float q3 = atof(argv[3]);
  float bx = atof(argv[4]);
  float by = atof(argv[5]);
  float bz = atof(argv[6]);
  biases[0] = bx;
  biases[1] = by;
  biases[2] = bz;
  quat[0] = q0;
  quat[1] = q1;
  quat[2] = q2;
  quat[3] = q3;
}

int main(int argc, char **argv)
{
  GtkWidget *window,*vbox,*logo,*glarea;

  /* Attribute list for gtkglarea widget. Specifies a
     list of Boolean attributes and enum/integer
     attribute/value pairs. The last attribute must be
     GDK_GL_NONE. See glXChooseVisual manpage for further
     explanation.
  */
  int attrlist[] = {
    GDK_GL_RGBA,
    GDK_GL_RED_SIZE,1,
    GDK_GL_GREEN_SIZE,1,
    GDK_GL_BLUE_SIZE,1,
    GDK_GL_DOUBLEBUFFER,
    GDK_GL_NONE
  };

  /* initialize gtk */
  gtk_init(&argc, &argv);

  /* Check if OpenGL (GLX extension) is supported. */
  if (gdk_gl_query() == FALSE) {
    g_print("OpenGL not supported\n");
    return 0;
  }

  /* Create new top level window. */
  window = gtk_window_new( GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(window), "ahrs3d");

  /* Quit form main if got delete event */
  gtk_signal_connect(GTK_OBJECT(window), "delete_event",
		     GTK_SIGNAL_FUNC(gtk_main_quit), NULL);


  /* You should always delete gtk_gl_area widgets before exit or else
     GLX contexts are left undeleted, this may cause problems (=core dump)
     in some systems.
     Destroy method of objects is not automatically called on exit.
     You need to manually enable this feature. Do gtk_quit_add_destroy()
     for all your top level windows unless you are certain that they get
     destroy signal by other means.
  */
  gtk_quit_add_destroy(1, GTK_OBJECT(window));


  vbox = GTK_WIDGET(gtk_vbox_new(FALSE, 0));
  gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);


  logo = gtk_label_new("ahrs3d");


  /* Create new OpenGL widget. */
  glarea = GTK_WIDGET(gtk_gl_area_new(attrlist));
  /* Events for widget must be set before X Window is created */
  gtk_widget_set_events(GTK_WIDGET(glarea),
			GDK_EXPOSURE_MASK);
  /* set minimum size */
  /*  gtk_widget_set_usize(GTK_WIDGET(glarea), 200,200); */
  /* set default size */
  gtk_gl_area_size(GTK_GL_AREA(glarea), 640,400);


  /* Connect signal handlers */
  /* Redraw image when exposed. */
  gtk_signal_connect(GTK_OBJECT(glarea), "expose_event",
		     GTK_SIGNAL_FUNC(draw), NULL);
  /* When window is resized viewport needs to be resized also. */
  gtk_signal_connect(GTK_OBJECT(glarea), "configure_event",
		     GTK_SIGNAL_FUNC(reshape), NULL);
  /* Do initialization when widget has been realized. */
  gtk_signal_connect(GTK_OBJECT(glarea), "realize",
		     GTK_SIGNAL_FUNC(init), NULL);
  /* construct widget hierarchy  */
  gtk_container_add(GTK_CONTAINER(window),GTK_WIDGET(vbox));
  gtk_box_pack_start(GTK_BOX(vbox),   logo, FALSE, FALSE, 0);
  gtk_box_pack_start(GTK_BOX(vbox), glarea,  TRUE,  TRUE, 0);



  /* show all widgets */
  gtk_widget_show(GTK_WIDGET(glarea));
  gtk_widget_show(GTK_WIDGET(logo));
  gtk_widget_show(GTK_WIDGET(vbox));
  gtk_widget_show(window);

  /* set focus to glarea widget */
  GTK_WIDGET_SET_FLAGS(glarea, GTK_CAN_FOCUS);
  gtk_widget_grab_focus(GTK_WIDGET(glarea));

  /* animating */
  //  gtk_idle_add((GtkFunction)animate, glarea);
  g_timeout_add(REFRESH_RATE, (GtkFunction)animate, glarea);

  IvyInit ("IvyGtkButton", "IvyGtkButton READY", NULL, NULL, NULL, NULL);
  IvyBindMsg(on_IMU_MAG, NULL, "^77 IMU_MAG (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_IMU_ACCEL, NULL, "^77 IMU_ACCEL (\\S*) (\\S*) (\\S*)");
  IvyBindMsg(on_AHRS_STATE, NULL, "^77 AHRS_STATE (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*) (\\S*)");
  IvyStart("127.255.255.255");

  game_init();
  gtk_main();


  return 0;
}
