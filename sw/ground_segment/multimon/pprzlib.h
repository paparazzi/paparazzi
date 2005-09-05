int pprz_demod_init(char *dev);

struct data { char* data_left; int len_left; char* data_right; int len_right;};
struct data* pprz_demod_read_data(void);
/** Returns NULL if no characters are available on the device */
