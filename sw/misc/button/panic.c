/*==============================================
       PoC for USB Panic Button under unix
   --------------------------------------------
   by: Benjamin Kendinibilir
   needs: libusb and libhid
   compile: gcc usbpanicbutton.c -o upb -lhid
   run: sudo ./upb (needs root)
  ==============================================*/

#include <hid.h>
#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <glib.h>
#include <sys/ioctl.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>

#define VENDOR_ID 0x1130
#define PRODUCT_ID 0x0202
#define IFACE_NO 0
#define PACKET_SIZE 8
#define PATH1 0x00010000
#define PATH2 0x00000000
#define PATH_LEN 2

#define TIMEOUT_PERIOD 20

int loop = 1;
hid_return ret;
HIDInterface* hid;
char packet[PACKET_SIZE];
int const path_out[] = { PATH1, PATH2 };
HIDInterfaceMatcher matcher = { VENDOR_ID, PRODUCT_ID, NULL, NULL, 0 };
GMainLoop *ml;
int ac_id, block_nr;

void endloop(int signum) {
	loop = 0;
	g_main_loop_quit(ml);
}

int btn_init(void) {

	#ifdef DEBUG
	hid_set_debug(HID_DEBUG_ALL);
	hid_set_debug_stream(stderr);
	hid_set_usb_debug(0);
	#endif

	ret = hid_init();
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_init failed with return code %d\n", ret);
		return 1;
	}

	hid = hid_new_HIDInterface();
	if (hid == 0) {
		fprintf(stderr, "hid_new_HIDInterface() failed, out of memory?\n");
		return 1;
	}

	ret = hid_force_open(hid, IFACE_NO, &matcher, 3);
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_force_open failed with return code %d\n", ret);
		return 1;
	}

	#ifdef DEBUG
	ret = hid_write_identification(stdout, hid);
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_write_identification failed with return code %d\n", ret);
		return 1;
	}

	ret = hid_dump_tree(stdout, hid);
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_dump_tree failed with return code %d\n", ret);
		return 1;
	}
	#endif

	signal(SIGINT, endloop);
	signal(SIGHUP, endloop);
	signal(SIGTERM, endloop);

	printf("waiting for panic button action:\n");

	return 0;
}

static gboolean btn_periodic(gpointer data __attribute__ ((unused))) {

	ret = hid_get_input_report(hid, path_out, PATH_LEN, packet, PACKET_SIZE);
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_get_input_report failed with return code %d\n", ret);
	}

	if(packet[0] == 0x1) {
	        IvySendMsg("dl JUMP_TO_BLOCK %d %d", ac_id, block_nr);
		printf("\aplop\n");
		fflush(stdout);
	}

	return loop;
}

int btn_close(void) {

	printf("cleaning up... ");

	ret = hid_close(hid);
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_close failed with return code %d\n", ret);
		return 1;
	}

	hid_delete_HIDInterface(&hid);

	ret = hid_cleanup();
	if (ret != HID_RET_SUCCESS) {
		fprintf(stderr, "hid_cleanup failed with return code %d\n", ret);
		return 1;
	}

	printf("exit.\n");

	return 0;
}


int main(int argc, char** argv) {

	if (argc != 3) {
		printf("usage: panic <ac_id> <block_nr>\n");
		return 1;
	}
	ac_id = strtol(argv[1], NULL, 10);
	block_nr = strtol(argv[2], NULL, 10);

	ml = g_main_loop_new(NULL, FALSE);

	IvyInit ("IvyCtrlButton", "IvyCtrlButton READY", NULL, NULL, NULL, NULL);
	IvyStart("127.255.255.255");

	btn_init();

	g_timeout_add(TIMEOUT_PERIOD, btn_periodic, NULL);
	g_main_loop_run(ml);

	btn_close();

	return 0;
}

