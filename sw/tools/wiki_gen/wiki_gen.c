#include <stdio.h>

void parse_name(char* subs)
{
  int status = 0;
  int i = 0;
  printf("\nThis block\n\n  <subsystem name=\"");

  for (i=0;i<256;i++)
  {
    if (subs[i] == '_')
    {
      status = 1;
      break;
    }
    if (subs[i] == '.')
    {
      status = 2;
      break;
    }
    printf("%c",subs[i]);
  }

  if (status == 1)
  {
    printf("\" type=\"");
    for (i++;i<256;i++)
    {

      if (subs[i] == '.')
      {
        break;
      }
      printf("%c",subs[i]);
    }
  }

  printf("\" >\n\nreplaces\n\n");

}


int empty_line(char* line)
{
  for (int i=0;i<256;i++)
  {
    if (
         (line[i] == ' ') ||
         (line[i] == '\t') ||
         (line[i] == '\n') ||
         (line[i] == '\r')
       )
    {
    }
    else if (line[i] == 0)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  return 1;
}

void print_subsys(char* file)
{
  char line[256];
  FILE* fp = 0;

  parse_name(file);

  fp = fopen(file , "r");
  if ( fp != NULL)
  {

    while (fgets(line, sizeof(line), fp) != NULL)
    {
       if (empty_line(line) == 0)
       {
         printf("  ");
         fputs(line,stdout);
       }
    }
    fclose(fp);
  }
  else
  {
     printf("Error opening '%s'\n",file);
  }

}

int main(int argc, char** argv)
{
  char buff[256];
  FILE* list_subs = 0;

  if (argc < 2)
  {
    printf("Need a file name\n");
    return -1;
  }

  list_subs = fopen(argv[1] , "r");
  if ( list_subs != NULL)
  {

    while (fgets(buff, sizeof(buff), list_subs) != NULL)
    {
       for (int i=0;i<256;i++)
       {
         if (buff[i] == '\n')
           buff[i] = 0;
       }
       print_subsys(buff);
    }
    fclose(list_subs);
  }


  printf("converting");
  return 0;

}
