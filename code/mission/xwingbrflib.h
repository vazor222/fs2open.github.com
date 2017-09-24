#pragma once
#include <map>
#include <array>



std::map<int, std::vector<std::string> > create_animation_map()
{
	std::map<int, std::vector<std::string> > m;
	m[1] = { "wait_for_click" };
	m[10] = { "clear_text" };
	m[11] = { "show_title", "i", "text_id" };
	m[12] = { "show_main", "i", "text_id" };
	m[15] = { "center_map", "f", "x", "f", "y" };
	m[16] = { "zoom_map", "f", "x", "f", "y" };
	m[21] = { "clear_boxes" };
	m[22] = { "box_1", "i", "ship_id" };
	m[23] = { "box_2", "i", "ship_id" };
	m[24] = { "box_3", "i", "ship_id" };
	m[25] = { "box_4", "i", "ship_id" };
	m[26] = { "clear_tags" };
	m[27] = { "tag_1", "i", "tag_id", "f", "x", "f", "y" };
	m[28] = { "tag_2", "i", "tag_id", "f", "x", "f", "y" };
	m[29] = { "tag_3", "i", "tag_id", "f", "x", "f", "y" };
	m[30] = { "tag_4", "i", "tag_id", "f", "x", "f", "y" };
	return m;
}
const std::map<int, std::vector<std::string> > ANIMATION_COMMANDS = create_animation_map();




class XWingBriefing
{
public:
	XWingBriefing();
	~XWingBriefing();

	static 	XWingBriefing *load(const char *fname);

	std::string message1;
};

