#ifdef FBW
extern void init_fbw( void );
extern void periodic_task_fbw( void );
extern void event_task_fbw( void );

#define Fbw(f) f ## _fbw()
#else
#define Fbw(f)
#endif

#ifdef AP
extern void init_ap( void );
extern void periodic_task_ap( void );
extern void event_task_ap( void );

#define Ap(f) f ## _ap()
#else
#define Ap(f)
#endif

int main( void ) __attribute__ ((noreturn));

int main( void ) {
  Fbw(init);
  Ap(init);
  while (1) {
    Fbw(periodic_task);
    Ap(periodic_task);
    Fbw(event_task);
    Ap(event_task);
  }
}
