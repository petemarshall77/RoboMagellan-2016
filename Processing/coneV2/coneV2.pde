// Learning Processing
// Daniel Shiffman
// http://www.learningprocessing.com

// Example 16-11: Simple color tracking

import processing.video.*;
import processing.net.*;
// Variable for capture device
Capture video;
Server myServer;
// A variable for the color we are searching for.
color trackColor; 

void setup() {
  size(640,480);
  video = new Capture(this,width,height);
  // Start off tracking for orange
  trackColor = color(255,103,0);
  video.start();
  smooth();
  
  //Setup Server
  
  myServer = new Server(this,9788);
}

void draw() {
  // Capture and display the video
  if (video.available()) {
    video.read();
  }
  video.loadPixels();
  image(video,0,0);

  // Before we begin searching, the "world record" for closest color is set to a high number that is easy for the first pixel to beat.
  float threshold = 40; 


  int countXY = 0;
  int sumX=0;
  int sumY=0;

  // Begin loop to walk through every pixel
  for (int x = 0; x < video.width; x ++ ) {
    for (int y = 0; y < video.height; y ++ ) {
      int loc = x + y*video.width;
      // What is current color
      color currentColor = video.pixels[loc];
      float r1 = red(currentColor);
      float g1 = green(currentColor);
      float b1 = blue(currentColor);
      float r2 = red(trackColor);
      float g2 = green(trackColor);
      float b2 = blue(trackColor);

      // Using euclidean distance to compare colors
      float d = dist(r1,g1,b1,r2,g2,b2); // We are using the dist( ) function to compare the current color with the color we are tracking.

      if (d < threshold) {
        sumX += x;
        sumY += y;
        countXY++;
        
      }
    }
  }

  // We only consider the color found if its color distance is less than 10. 
  // This threshold of 10 is arbitrary and you can adjust this number depending on how accurate you require the tracking to be.
if(countXY != 0){
    // Draw a circle at the tracked pixel
    fill(trackColor);
    strokeWeight(4.0);
    stroke(0);
    ellipse(sumX/countXY,sumY/countXY,16,16);
    /////////Send the Data to network//////
    myServer.write(sumX/countXY + " " + countXY + "\n");
    } else {
    myServer.write("0 0\n");
    }
}

void mousePressed() {
  // Save color where the mouse is clicked in trackColor variable
  int loc = mouseX + mouseY*video.width;
  trackColor = video.pixels[loc];
}
