#include <caml/mlvalues.h>
#include <caml/callback.h>
#include <caml/memory.h>
#include <caml/alloc.h>
#include <caml/fail.h>

#include <stdio.h>
#ifdef WIN32
#include <gdk/gdkwin32.h>
#else
#include <X11/Xlib.h>
#include <X11/Intrinsic.h>
#include <gdk/gdkx.h>
#include <GL/glx.h>
#endif

#define MAX_FONTS 1000
#ifndef WIN32
static GLuint ListBase[MAX_FONTS];
static GLuint ListCount[MAX_FONTS];
#endif

CAMLprim value _gtkgl_hack_load_bitmap_font (const char *font) {
  static int FirstTime = 1;
  int first, last, count;
  int i ;

#ifndef WIN32
  XFontStruct *fontinfo;
  GLuint fontbase;
  Display* dpy = GDK_DISPLAY();

  if (FirstTime) {
	for (i=0;i<MAX_FONTS;i++) {
	  ListBase[i] = ListCount[i] = 0;
	}
	FirstTime = 0;
  }

  /* Chargement de la fonte X */
  fontinfo = XLoadQueryFont(dpy, font);
  if (!fontinfo) return 0;

  first = fontinfo->min_char_or_byte2;
  last = fontinfo->max_char_or_byte2;

  /* Nombre de caracteres definis par cette fonte */
  count = last-first+1;

  /* Creation des listes OpenGL a partir de la fonte */
  fontbase = glGenLists( (GLuint) (last+1) );
  if (fontbase==0) return 0 ;
  /*  printf("Fontbase=%d first=%d last=%d\n", fontbase, first, last) ; fflush(stdout) ;*/

  /* OpenGL doit l'utiliser */
  glXUseXFont(fontinfo->fid, first, count, (int) fontbase+first);


  for (i=0;i<MAX_FONTS;i++) {
	if (ListBase[i]==0) {
	  ListBase[i] = fontbase;
	  ListCount[i] = last+1;
	  break;
	}
  }

  return (Val_int(fontbase));
#else
  return (Val_int(0));
#endif
}

CAMLprim value _gtkgl_hack_unload_bitmap_font (value font) {
#ifndef WIN32
  int i;
  GLuint fontbase = Int_val(font) ;

  for (i=0;i<MAX_FONTS;i++) {
	if (ListBase[i]==fontbase) {
	  glDeleteLists( ListBase[i], ListCount[i] );
	  ListBase[i] = ListCount[i] = 0;
	  break ;
	}
  }
#endif
  return (Val_unit);
}
