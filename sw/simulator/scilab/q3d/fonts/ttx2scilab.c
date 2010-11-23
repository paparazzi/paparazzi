


#include <glib.h>


static void on_start_element(GMarkupParseContext *context,
			     const gchar         *element_name,
			     const gchar        **attribute_names,
			     const gchar        **attribute_values,
			     gpointer             user_data,
			     GError             **error);
static void on_text(GMarkupParseContext *context,
		    const gchar         *text,
		    gsize                text_len,
		    gpointer             user_data,
		    GError             **error);
static void on_error (GMarkupParseContext *context,
		      GError              *error,
		      gpointer             user_data);



int main (int rgc, char** argv) {

  GMarkupParser gmp = { on_start_element, NULL, on_text, NULL, on_error};
  GMarkupParseContext *gmpc = g_markup_parse_context_new(&gmp, 0, NULL, NULL);

  GError* my_err = NULL;
  GIOChannel *gioc = g_io_channel_new_file("b_bhi.xml", "r", &my_err);
#define BUF_LEN 16384
  gchar buf[BUF_LEN];
  gsize bytes_read;
  GIOStatus st = g_io_channel_read_chars(gioc, buf, BUF_LEN, &bytes_read, &my_err);

  //  g_message("read file %d", bytes_read);

  if (!g_markup_parse_context_parse(gmpc, buf, bytes_read, &my_err)) {
    g_message("error parsing xml");
    return -1;
  }
  return 0;
}



static void on_start_element(GMarkupParseContext *context,
			     const gchar         *element_name,
			     const gchar        **attribute_names,
			     const gchar        **attribute_values,
			     gpointer             user_data,
			     GError             **error) {
  //  g_message("on start element (%s)", element_name);
  if (!strcmp(element_name, "pt")) {
    //    g_message("pt ", element_name);
    int i = 0;
    while (attribute_names[i]) {
      //      g_message("%s %s", attribute_names[i], attribute_values[i]);
      printf("%s ", attribute_values[i]);
      if (!strcmp(attribute_names[i], "on")) printf("\n");
      i++;
    }
  }
}

static void on_text(GMarkupParseContext *context,
		    const gchar         *text,
		    gsize                text_len,
		    gpointer             user_data,
		    GError             **error) {
  //  g_message("on text (%s)", text);


}

static void on_error (GMarkupParseContext *context,
		      GError              *error,
		      gpointer             user_data) {

  //  g_message("on error");
}

