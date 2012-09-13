#include <stdio.h>
#include <string.h>


char buf[200];

float get_time(char* str)
{
	float time = 0.0f;
	sscanf(str,"%f", &time);
	return time;
}

void add_time(char* str, float offset, FILE* fp)
{
	float time = 0.0f;
	char* c = strchr(str, ' ');
	if (c == 0)
		return;

	sscanf(str,"%f", &time);
	fprintf(fp, "%.3f%s", time+offset, c);
}

int merge(char* fn1, char* fn2)
{
	FILE* f1 = fopen(fn1,"r+w");
	FILE* f2 = fopen(fn2,"r");
	int res = 0;
	char str[200];
	float offset = 0.0f;
	int firstline = 1;

	if ((f1 > 0) && (f2 > 0))
	{
		//
		printf("Searching End Time in File '%s'\n",fn1);

		while(fgets(str,sizeof(str),f1) != NULL)
		{
		}

		printf("Last Line:\n%s\n",str);
		offset = get_time(str);
		printf("Last Time Stamp: %f\n", offset);

		while(fgets(str,sizeof(str),f2) != NULL)
		{
			// Find initial time
			//if (firstline)
			//{
			//	firstline = 0;
			//	offset -= get_time(str);
			//}

			// Change Time and add to first file:
			add_time(str, offset, f1);
		}

	}
	else
	{
		fprintf(stderr,"Failed to open file\n");
		res = -1;
	}

	if (f1 > 0)
		fclose(f1);
	if (f2 > 0)
		fclose(f2);

	return -1;
}




int main(int argc, char** argv)
{
	if (argc < 3)
	{
		fprintf(stderr, "Program needs 2 logfile names to merge\n");
		return -1;
	}

	printf("Merging '%s' and '%s'\n",argv[1], argv[2]);

	return merge(argv[1],argv[2]);
}
