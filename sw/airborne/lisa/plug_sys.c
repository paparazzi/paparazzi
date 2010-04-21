#include <unistd.h>
extern off_t                  lseek(int, off_t, int);
extern void                   abort(void);
extern pid_t                  getpid(void);
extern int                    kill(pid_t, int);
extern void *                 _sbrk(int);
extern ssize_t                _write(int, const void *, size_t);
extern int                    _close(int);
extern ssize_t                _read(int, void *, size_t);

off_t                  _lseek(int a, off_t b, int c) {}
void                   abort(void) {}
pid_t                  getpid(void) {}
int                    kill(pid_t a, int b) {}
void *                 _sbrk(int a) {}
ssize_t                _write(int a, const void *b, size_t c) {}
int                    _close(int a) {}
ssize_t                _read(int a, void *b, size_t c) {}
