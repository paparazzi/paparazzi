/*
 *
 *  looks like some functions of the math lib (eg sqrt, atan2) insist to believe
 *  they are running on UNIX. Let's keep them happy !
 *
 */

#include <unistd.h>
extern off_t                  _lseek(int, off_t, int);
extern void                   abort(void);
extern int                    kill(pid_t, int);
extern void *                 _sbrk(int);
extern ssize_t                _write(int, const void *, size_t);
extern int                    _close(int);
extern ssize_t                _read(int, void *, size_t);

off_t                  _lseek(int a, off_t b, int c) { return b;}
void                   abort(void) {while(1){};}
pid_t                  getpid(void) { return 0;}
int                    kill(pid_t a, int b) {return 0;}
void *                 _sbrk(int a) {return 0;}
ssize_t                _write(int a, const void *b, size_t c) {return 0; }
int                    _close(int a) { return 0;}
ssize_t                _read(int a, void *b, size_t c) {return 0;}
