

                                        Some usefull hints about the usage of the osd driver.
                                     Forgive me in advance for any typos, Chris Efstathiou 12-Oct-2020

FUNCTION osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)

attributes =  BLINK, INVERT, C_JUST (Center Justified), R_JUST (Right Justified)
L_JUST (Left Justified) which is also the default behavior, same as attribute FALSE or 0
Attributes can be ORed example "C_JUST|BLINK|INVERT"
char_nb = the number of chars to reserve (clear) 
row = OSD ROW NUMBER
column = OSD COLUMN NUMBER
BE CAREFULL, THE MATEK OSD DOS NOT HAVE SMALL ENGLISH LETTERS, ONLY CAPITAL LETTERS!!!!
The reason the osd display uses a switch statement is because the OSD can only accept one transfer
at a time so each time the periodic function is called  a single string trasfer occurs.

FUNCTION static void osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)

osd_sprintf() transforms the float number to ASCII suitable for the OSD and stores it to the 'osd_string[]' char array.
attributes =  BLINK, INVERT, C_JUST (Center Justified), R_JUST (Right Justified)
Attributes can be ORed example BLINK|INVERT
char_nb = the number of chars to reserve (clear) 
row = OSD ROW NUMBER
column = OSD COLUMN NUMBER

EXAMPLES:

Example1: osd_put_s("HDG", FALSE, 3, 0, 15); 
prints H D G AND ALLOCATES 3 OSD CHARACTERS AT ROW 0, COLUMN 15
ANY SURPLUS CHARACTERS ARE OMMITED

Example2: osd_put_s("HEADING", FALSE, 4, 0, 15); 
prints H E A D AND ALLOCATES 4 OSD CHARACTERS AT ROW 0, COLUMN 15
ANY SURPLUS CHARACTERS ARE OMMITED

Example3: temp = ((float)electrical.vsupply);
          osd_sprintf(osd_string, "%.1fV", temp );
          osd_put_s(osd_string, L_JUST, 5, 1, 2); 
osd_sprintf() transforms the float number to ASCII suitable for the OSD and stores it to the 'osd_string[]' char array.
osd_put_s() prints the 'osd_string[]' char array  AND ALLOCATES 5 OSD CHARACTERS AT ROW 1, COLUMN 2
IN osd_sprintf(osd_string, "%.1fV", temp ) WE SPECIFIED A FLOAT WITH ANE DECDIMAL POINT SO IF THE FLOAT EQUALS 254,1 
THEN WE NEED TO ALLOCATE 5 OSD CHARACTERS, '2','5','4', the comma ',' and finally the decimal digit '1'
ANY SURPLUS CHARACTERS ARE OMMITED


FUNCTION static bool _osd_sprintf(char* buffer, char* string, float value)

A VERY VERY STRIPED DOWN sprintf function suitable only for the paparazzi OSD.
ANY SPECIAL CHARACTER CODE MUST BE A 3 DIGIT NUMBER WITH THE LEADING ZEROS!!!!
THE SPECIAL CHARACTER CAN BE PLACED BEFORE OR AFTER THE FLOAT OR ANY OTHER CHARACTER

EXAMPLES:
special char 160 is an OSD character displaying a small "DIST" 
float_foo = 10,1234

Example1: osd_sprintf(osd_string, "%160c%.2fM", float_foo); 
prints DIST 1 0 , 1 2 M with a total of 7 OSD characters

Example2: osd_sprintf(osd_string, "%.0f%160cM", float_foo);
prints 1 0 DIST M  with a total of 4 OSD characters

Example3: osd_sprintf(osd_string, "%.0fM", float_foo);
prints 1 0 M  with a total of 3 OSD characters

Example4: osd_sprintf(osd_string, "%.0f", float_foo);
prints 1 0  with a total of 2 OSD characters



