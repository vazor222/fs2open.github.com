#include <stdio.h>
#include <tchar.h>
#include <vector>
#include <string>
#include <assert.h>
#include "xwingbrflib.h"



XWingBriefing::XWingBriefing()
{
}


XWingBriefing::~XWingBriefing()
{
}


XWingBriefing *XWingBriefing::load(const char *fname)
{
	FILE *f = fopen(fname, "rb");
	if (f == NULL)
		return NULL;
	fseek(f, 0, SEEK_END);
	int len = ftell(f);
	fseek(f, 0, SEEK_SET);
	unsigned char *data = (unsigned char *)malloc(len);
	fread(data, len, 1, f);
	fclose(f);

	//struct xwi_header *h = (struct xwi_header *)data;
	//if (h->version != 2)
	//	return NULL;

	XWingBriefing *b = new XWingBriefing();
	return b;
}