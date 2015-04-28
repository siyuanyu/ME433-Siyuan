/*******************************************************
 Windows HID simplification

 Alan Ott
 Signal 11 Software

 8/22/2009

 Copyright 2009
 
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/

#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"

#define MAX_STR 255

int main(int argc, char* argv[])
{
	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;
	int loopcount = 0;
	char message[65];
	short x[100];
	short y[100];
	short z[100];
	int rownum;
	FILE *ofp;
	ofp = fopen("/Users/ryanyu/Desktop/accels.txt", "w");

	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0x3f, NULL);

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %s\n", wstr);

	// Read the Product String
	res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);

	// Read the Serial Number String
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);

	// Read Indexed String 1
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	wprintf(L"Indexed String 1: %s\n", wstr);

	// Toggle LED (cmd 0x80). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x0;
	scanf("%d", &rownum);
	buf[2] = rownum;
	scanf("%s", message);
	int j;
	for (j = 3; j < 65; j++)
		buf[j] = message[j-3];

	res = hid_write(handle, buf, 65);


while(loopcount < 100) {
	// Request state (cmd 0x81). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x1;
	res = hid_write(handle, buf, 65);

	if (buf[1] == 1) {
		// Read requested state
		res = hid_read(handle, buf, 65);

		x[loopcount] = ~(buf[2] | (buf[3] << 8))+1;
		y[loopcount] = ~(buf[4] | (buf[5] << 8))+1;
		z[loopcount] = ~(buf[6] | (buf[7] << 8))+1;
		printf("x: %d\n", x[loopcount]);
		printf("y: %d\n", y[loopcount]);
		printf("z: %d\n", z[loopcount]);
		//Print out the returned buffer.
		//for (i = 2; i < 8; i++)
		//	printf("buf[%d]: %d\n", i, buf[i]);
		loopcount++;
	}
}
	for (i=0; i<100; i++) {

		fprintf(ofp,"%d %d %d\r\n",x[i],y[i],z[i]);

	}
	// Finalize the hidapi library
	res = hid_exit();

	return 0;
}
