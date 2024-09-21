#include "liblvgl/lvgl.h"
#include "lunar/api.hpp"
#include <string>

LV_IMG_DECLARE(HS);
LV_IMG_DECLARE(logo);

int autonState = 0; 

static void clicked1(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * btn = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) 
		autonState = 1;
}

static void clicked2(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * btn = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) 
		autonState = 2;
}

static void clicked3(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * btn = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) 
		autonState = 3;
}

static void clicked4(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * btn = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) 
		autonState = 4;
}

static void clicked5(lv_event_t * e){
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * btn = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) 
		autonState = 5;
}
void brainScreen(){
    lv_init(); // initialize lvgl
	
	lv_obj_t * negBlue = lv_btn_create(lv_scr_act()); // creating a button object btn and getting active screen
	lv_obj_t * negBlueLabel = lv_label_create(negBlue);
	lv_obj_t * negRed = lv_btn_create(lv_scr_act()); // creating a button object btn and getting active screen
	lv_obj_t * negRedLabel = lv_label_create(negRed);
	lv_obj_t * posBlue = lv_btn_create(lv_scr_act()); // creating a button object btn and getting active screen
	lv_obj_t * posBlueLabel = lv_label_create(posBlue);
	lv_obj_t * posRed = lv_btn_create(lv_scr_act()); // creating a button object btn and getting active screen
	lv_obj_t * posRedLabel = lv_label_create(posRed);
	lv_obj_t * skills = lv_btn_create(lv_scr_act()); // creating a button object btn and getting active screen
	lv_obj_t * skillsLabel = lv_label_create(skills);
	lv_obj_t * img = lv_img_create(lv_scr_act());
	lv_obj_t * logoc = lv_img_create(lv_scr_act());
	lv_obj_t * selectedA;
	lv_obj_t * selectedC;
	selectedC = lv_label_create(lv_scr_act()); //create label and puts it on the screen
	lv_label_set_text(selectedC, "Selected: "); //sets label text

	selectedA = lv_label_create(lv_scr_act()); //create label and puts it on the screen
	lv_label_set_text(selectedA, "Default"); //sets label text
	std::string output = "";
	while(1){
		lv_obj_set_pos(negBlue,60,0); // Set button at position X:50 Y:50 pixel as units
		lv_obj_set_size(negBlue,120,120); // Set button size with width:40 length:20 pixel as units
		lv_obj_add_event_cb(negBlue, clicked1, LV_EVENT_ALL, NULL); 
		
		lv_label_set_text(negBlueLabel, "NegBlue"); // Set text on the button
		lv_obj_center(negBlueLabel);
		
		lv_obj_set_pos(negRed,180,0); // Set button at position X:50 Y:50 pixel as units
		lv_obj_set_size(negRed,120,120); // Set button size with width:40 length:20 pixel as units
		lv_obj_add_event_cb(negRed, clicked2, LV_EVENT_ALL, NULL); 
		
		lv_label_set_text(negRedLabel, "NegRed"); // Set text on the button
		lv_obj_center(negRedLabel);
		
		lv_obj_set_pos(posBlue,60,120); // Set button at position X:50 Y:50 pixel as units
		lv_obj_set_size(posBlue,120,120); // Set button size with width:40 length:20 pixel as units
		lv_obj_add_event_cb(posBlue, clicked3, LV_EVENT_ALL, NULL); 
		
		lv_label_set_text(posBlueLabel, "posBlue"); // Set text on the button
		lv_obj_center(posBlueLabel);
		
		lv_obj_set_pos(posRed,180,120); // Set button at position X:50 Y:50 pixel as units
		lv_obj_set_size(posRed,120,120); // Set button size with width:40 length:20 pixel as units
		lv_obj_add_event_cb(posRed, clicked4, LV_EVENT_ALL, NULL); 
		
		lv_label_set_text(posRedLabel, "posRed"); // Set text on the button
		lv_obj_center(posRedLabel);

		lv_obj_set_pos(skills,315,85); // Set button at position X:50 Y:50 pixel as units
		lv_obj_set_size(skills,150,150); // Set button size with width:40 length:20 pixel as units
		lv_obj_add_event_cb(skills, clicked5, LV_EVENT_ALL, NULL); 
		
		lv_label_set_text(skillsLabel, "Skills"); // Set text on the button
		lv_obj_center(skillsLabel);

		lv_img_set_src(img, &HS);
		lv_obj_set_pos(img,60,0);

		lv_img_set_src(logoc, &logo);
		lv_obj_set_pos(logoc,0,0);

		if(autonState == 1) {  lv_label_set_text(selectedA, "PosRed"); }//sets label text 
		else if(autonState == 2){  lv_label_set_text(selectedA, "NegRed"); }//sets label text
		else if(autonState == 3){  lv_label_set_text(selectedA, "PosBlue"); }//sets label text
		else if(autonState == 4){  lv_label_set_text(selectedA, "NegBlue"); }//sets label text
		else if(autonState == 5){  lv_label_set_text(selectedA, "Skills"); }//sets label text

		lv_obj_set_pos(selectedC,345,25);
		lv_obj_set_pos(selectedA,345,45);
		pros::delay(100);
	}
}