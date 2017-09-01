#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8MultiArray.h>
#include <simple_display_msgs/draw.h>

#include <math.h>
#include "Time/Time.h"
#include "Adafruit_GFX/Adafruit_GFX.h"   // Core graphics library
#include "RGB-matrix-Panel-master/RGBmatrixPanel.h" // Hardware-specific library

// Similar to F(), but for PROGMEM string pointers rather than literals
//#define F2(progmem_ptr) (const __FlashStringHelper *)progmem_ptr

#define CLK 11  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2

RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, true);

bool isIdle=true;

ros::NodeHandle_<ArduinoHardware, 5, 5, 512, 512>  nh;
//unsigned char drawCmdTemp[512];
//ros::Time currentDrawStart;
//simple_display_msgs::drawRequest currentDrawCmd;


bool doDrawCommand(simple_display_msgs::DrawCommand* layer){
  /*ros::Time layerStart = ros::Time(currentDrawStart.sec, currentDrawStart.nsec) += layer->delay;
  ros::Time layerEnd = (ros::Time(currentDrawStart.sec, currentDrawStart.nsec) += layer->delay) += layer->duration;

	double nowSec = now.toSec();
	double startSec = layerStart.toSec();
	double endSec = layerEnd.toSec();

	if(nowSec > startSec && nowSec < endSec){
    return false;
  }*/

  // Load color info
  uint16_t color = 0;
  if(layer->color_space == simple_display_msgs::DrawCommand::COLOR_888 && layer->color_length == 3){
    color = matrix.Color888(layer->color[0], layer->color[1], layer->color[2], true);
  }
  else if(layer->color_space == simple_display_msgs::DrawCommand::COLOR_HSV && layer->color_length == 3){
    color = matrix.ColorHSV(layer->color[0], layer->color[1], layer->color[2], true);
  }
  else{
    color = matrix.Color888(layer->color[0], layer->color[1], layer->color[2], true);
  }

  // Parse shape parameters
  if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_SCREEN){
    matrix.fillScreen(color);
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_PIXEL && layer->shape_data_length == 2){
    matrix.drawPixel(layer->shape_data[0], layer->shape_data[1], color);
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_LINE && layer->shape_data_length == 4){
    matrix.drawLine(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], layer->shape_data[3], color);
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_V_LINE && layer->shape_data_length == 3){
    matrix.drawFastVLine(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], color);
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_H_LINE && layer->shape_data_length == 3){
    matrix.drawFastHLine(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], color);
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_RECT && layer->shape_data_length == 4){
    if(layer->fill){
      matrix.fillRect(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], layer->shape_data[3], color);
    }
    else{
      matrix.drawRect(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], layer->shape_data[3], color);
    }
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_CIRCLE && layer->shape_data_length == 3){
    if(layer->fill){
      matrix.fillCircle(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], color);
    }
    else{
      matrix.drawCircle(layer->shape_data[0], layer->shape_data[1],
                    layer->shape_data[2], color);
    }
  }
  else if(layer->shape == simple_display_msgs::DrawCommand::SHAPE_TRIANGLE && layer->shape_data_length == 3){
    if(layer->fill){
      matrix.fillTriangle(layer->shape_data[0], layer->shape_data[1],
                          layer->shape_data[2], layer->shape_data[3],
                          layer->shape_data[4], layer->shape_data[5], color);
    }
    else{
      matrix.drawTriangle(layer->shape_data[0], layer->shape_data[1],
                          layer->shape_data[2], layer->shape_data[3],
                          layer->shape_data[4], layer->shape_data[5], color);
    }
  }
  else{
    //matrix.fillScreen(0);
  }

  if(layer->swap_buffers){
    matrix.swapBuffers(true);
    //isIdle=true;
  }

  return true;
}


void drawCb(const simple_display_msgs::drawRequest& req, simple_display_msgs::drawResponse& res){
  //req.serialize(drawCmdTemp);
  //currentDrawCmd.deserialize(drawCmdTemp);
  //currentDrawStart = nh.now();

  bool activeLayer = false;
  for(int i=0; i<req.layers_length; i++){
    doDrawCommand(&req.layers[i]);
  }

  res.success = true;
  isIdle=false;
}


//ros::ServiceServer<simple_display_msgs::drawRequest, simple_display_msgs::drawResponse> drawSrv("~/head/draw", drawCb);

int leftEyeX = 8;
int leftEyeY = 7;
int rightEyeX = 23;
int rightEyeY = 7;

void messageCb( const std_msgs::Int8MultiArray& positions){
  leftEyeX = positions.data[0];
  leftEyeY = positions.data[1];
  rightEyeX = positions.data[2];
  rightEyeY = positions.data[3];
}

ros::Subscriber<std_msgs::Int8MultiArray> eyeSub("~/head/eye_positions", &messageCb );

void setup() {
  matrix.begin();
  matrix.setTextWrap(false); // Allow text to run off right edge
  matrix.setTextSize(1);
  matrix.fillScreen(0);
  matrix.swapBuffers(false);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(eyeSub);
}

void loop() {
  nh.spinOnce();

  if(isIdle){
    matrix.fillScreen(0);


    if(nh.connected()){
	    float i = (((float)(millis()%2500)) / 2500) * M_PI;
			int x = (sin(i) * 127.0) + 127.0;
			int b = (sin(i) * 70.0) + 184.0;
			matrix.fillRect(leftEyeX,leftEyeY, 2, 2, matrix.Color888(x,x,b,true));
			matrix.fillRect(rightEyeX,rightEyeY, 2, 2, matrix.Color888(x,x,b,true));
    }
    else{
      float i = (((float)(millis()%3500)) / 3500) * M_PI;
			int x = (sin(i) * 127.0) + 127.0;
      matrix.drawPixel(31,15, matrix.Color888(x,x,x,true));
    }


    matrix.swapBuffers(false);
  }
  else{
    /*ros::Time t = nh.now();
    bool activeLayer = false;
    for(int i=0; i<currentDrawCmd.layers_length; i++){
      activeLayer |= doDrawCommand(t, &currentDrawCmd.layers[i]);
    }

    if(!activeLayer){

      if(currentDrawCmd.loop){
				currentDrawStart = nh.now();
      }
      else{
        isIdle = true;
      }

    }*/
  }
}
