extern unsigned long timer0_millis;

// this function replaces the arduino millis() funcion
unsigned long DIYmillis()
{
   unsigned long m;
   unsigned long m2;

   // timer0_millis could change inside timer0 interrupt and we don´t want to disable interrupts 
   // we can do two readings and compare.
   m = timer0_millis;
   m2 = timer0_millis;
   if (m!=m2)               // timer0_millis corrupted?
      m = timer0_millis;   // this should be fine...
   return m;
}

void DIYdelay(unsigned long ms)
{
   unsigned long start = DIYmillis();
   while (DIYmillis() - start <= ms)
      ;
}
