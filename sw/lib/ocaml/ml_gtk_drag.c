#include <string.h>
#include <gtk/gtk.h>
#include <caml/mlvalues.h>
#include <caml/alloc.h>
#include <caml/memory.h>
#include <caml/callback.h>
#include <caml/fail.h>

extern value Val_GtkTreePath(GtkTreePath*);

#define Pointer_val(val) ((void*)Field(val,1))

#ifdef G_DISABLE_CAST_CHECKS
#define check_cast(f,v) f(Pointer_val(v))
#else
#define check_cast(f,v) (Pointer_val(v) == NULL ? NULL : f(Pointer_val(v)))
#endif

#define GtkTreeView_val(val) check_cast(GTK_TREE_VIEW,val)

CAMLprim value
ml_gtk_tree_view_get_drag_dest_row(value val_tree) {
  CAMLparam0();
  CAMLlocal1(ret);
  GtkTreePath *path;
  GtkTreeViewDropPosition pos;
  gtk_tree_view_get_drag_dest_row(GtkTreeView_val(val_tree), &path, &pos);
  ret = alloc_tuple(2);
  Store_field(ret,0,Val_GtkTreePath(path));
  Store_field(ret,1,Val_int(pos));
  CAMLreturn(ret);
}
