#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
#include <ivy.h>
#include <ivyloop.h>

/* callback associated to "Hello" messages */
void HelloCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	const char* arg = (argc < 1) ? "" : argv[0];
	IvySendMsg ("Bonjour%s", arg);
}

/* callback associated to "Bye" messages */
void ByeCallback (IvyClientPtr app, void *data, int argc, char **argv)
{
	IvyStop ();
}

main (int argc, char**argv)
{
	/* handling of -b option */
	const char* bus = 0;
	char c;
	while (c = getopt (argc, argv, "b:") != EOF) {
		switch (c) {
		case 'b':
			bus = optarg;
			break;
		}
	}

	/* handling of environment variable */
	if (!bus)
		bus = getenv ("IVYBUS");

	/* initializations */
	IvyInit ("IvyTranslater", "Hello le monde", 0, 0, 0, 0);
	IvyStart (bus);

	/* binding of HelloCallback to messages starting with 'Hello' */
	IvyBindMsg (HelloCallback, 0, "^Hello(.*)");

	/* binding of ByeCallback to 'Bye' */
	IvyBindMsg (ByeCallback, 0, "^Bye$");

	/* main loop */
	IvyMainLoop(0);
}
