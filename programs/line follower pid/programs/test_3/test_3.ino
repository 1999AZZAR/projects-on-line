//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 03 Test OLED and Buttons

//----------------------------------------Include Library
#include "U8glib.h"
//----------------------------------------

//----------------------------------------Initialize u8g 
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);  // I2C / TWI 
//----------------------------------------

#define Button_Pin_1 5
#define Button_Pin_2 6
#define Button_Pin_3 7

bool Button_1;
bool Button_2;
bool Button_3;

//========================================================================void draw(void)
void draw(void) {
  // graphic commands to redraw the complete screen should be placed here  
  u8g.setFont(u8g_font_unifont);
  
  u8g.setPrintPos(0, 10); 
  u8g.print("OLED 128x64");

  Button_1 = digitalRead(Button_Pin_1);
  Button_2 = digitalRead(Button_Pin_2);
  Button_3 = digitalRead(Button_Pin_3);

  if (Button_1 == LOW) {
    u8g.setPrintPos(0, 25);
    u8g.print("Button1 Press");
  } else {
    u8g.setPrintPos(0, 25);
    u8g.print("Button1 UnPress");
  }

  if (Button_2 == LOW) {
    u8g.setPrintPos(0, 40);
    u8g.print("Button2 Press");
  } else {
    u8g.setPrintPos(0, 40);
    u8g.print("Button2 UnPress");
  }

  if (Button_3 == LOW) {
    u8g.setPrintPos(0, 55);
    u8g.print("Button3 Press");
  } else {
    u8g.setPrintPos(0, 55);
    u8g.print("Button3 UnPress");
  }
}
//========================================================================

//========================================================================VOID SETUP
void setup(void) {
  pinMode(Button_Pin_1, INPUT_PULLUP);
  pinMode(Button_Pin_2, INPUT_PULLUP);
  pinMode(Button_Pin_3, INPUT_PULLUP);
}
//========================================================================

//========================================================================VOID LOOP
void loop(void) {
  // picture loop (Loop for display on OLED)
  u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  delay(500);
}
//========================================================================
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<