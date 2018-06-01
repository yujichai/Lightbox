import processing.serial.*;

import processing.sound.*;

//AudioDevice device;
SoundFile[] file;

Serial myPort;
int val;
// Define the number of samples 
int numsounds = 3;

void setup(){
  size(200,200);
  background(255, 255, 255);
  String portName = Serial.list()[1];
  //println(Serial.list());
  myPort = new Serial(this, portName, 9600);
  
  // Create a Sound renderer and an array of empty soundfiles
  //device = new AudioDevice(this, 48000, 32);
  file = new SoundFile[numsounds];
  
  // Load 5 soundfiles from a folder in a for loop. By naming the files 1., 2., 3., n.aif it is easy to iterate
  // through the folder and load all files in one line of code.
  
    file[0] = new SoundFile(this, "START.wav");
    file[1] = new SoundFile(this, "OVER.wav");
    file[2] = new SoundFile(this, "TILL.wav");
    //file[3] = new SoundFile(this, "celebrating.wav");
    //file[4] = new SoundFile(this, "celebrating 2.mp3");
}

void draw(){
  if( myPort.available() > 0) {
    val = myPort.read();
    if(val == 1) {
      file[0].play();
      background(0, 0, 255);
    }
    if(val == 2) {
      file[1].play();
      background(255, 0, 0);
    }
    if(val == 3) {
      file[2].play();
      background(0, 255, 0);
    }
    //delay(1);
  }
}



void keyPressed() {
  switch(key){
    case '1':
      file[1].play();
      break;
    case '2':
      file[2].play();
      break;
  }
}