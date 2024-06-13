///////////////////////////////////
//Programa para detectar personas
//con kinect y opencv
//de manera cenital
//
// Diego Suarez 2023
///////////////////////////////////


import oscP5.*;
import netP5.*;

import gab.opencv.*;   //Libreria opencv
import KinectPV2.*;    //Libreria kinect
import java.awt.Rectangle;  //Libreria de java para manejar los bounding box

KinectPV2 kinect;      //Control Kinect
OpenCV opencv;          //Control opencv

OscP5 oscP5;
NetAddress dest;

float polygonFactor = 1;

int threshold = 10;

//Distancias entre las cuales queremos detectar en mm
int maxD = 3000; //1.5m
int minD = 50; //50 mm

int contadorPersonas=0;
float posicionesX=0;
float posicionesY=0;

////////////
//SETUP
void setup() {
  size(1536, 424, P3D);
  opencv = new OpenCV(this, 512, 424);
  kinect = new KinectPV2(this);

  kinect.enableDepthImg(true);
  kinect.enableBodyTrackImg(true);
  kinect.enablePointCloud(true);

  kinect.init();

  // Initialize OSC communication
  oscP5 = new OscP5(this, 12000);
  dest = new NetAddress("192.168.0.123", 6448);
}
//END SETUP  
////////////

////////////
//DRAW
void draw() {
  background(0);

  noFill();
  strokeWeight(3);

  //Dibujamos los datos de profundidad
  image(kinect.getDepthImage(), 0, 0);

  //Dibujamos los datos de point cloud
  image(kinect.getPointCloudDepthImage(), 512, 0);

  opencv.loadImage(kinect.getPointCloudDepthImage());
  opencv.gray();
  opencv.threshold(threshold);
  PImage dst = opencv.getOutput();

  //A continuacion dibujamos los contornos encontrados
  ArrayList<Contour> contours = opencv.findContours(false, false);


  if (contours.size() > 0) {
    contadorPersonas=0;
    posicionesX=0;
    posicionesY=0;
    for (Contour contour : contours) {
      //obtenemos su bounfing box y dibujamos el rectangulo
      Rectangle persona= contour.getBoundingBox();
      if (persona.width>20) {
       
        //dibujamosun circulo en el centro
        float posxOSC=persona.x+persona.width/2;
        float posyOSC=persona.y+persona.height/2;
        //SOLO ENTRA AQUI CON PERSONAS EN EL CENTRO
        if(posxOSC>60 && posxOSC<450){
          contadorPersonas++;
          posicionesX= posicionesX+posxOSC;
          posicionesY= posicionesY+ posyOSC;
           noFill();
          stroke(0, 255, 0);
          strokeWeight(3);
          rect(persona.x, persona.y, persona.width, persona.height);
          ellipse(posxOSC, posyOSC, 10, 10);

        

        // For visual confirmation on the sketch
        fill(255, 0, 0);

       
        }
        
      }


      contour.setPolygonApproximationFactor(polygonFactor);
      if (contour.numPoints() > 50) {
        noFill();
        stroke(0, 200, 200);
        strokeWeight(1);
        beginShape();
        for (PVector point : contour.getPolygonApproximation ().getPoints()) {
          vertex(point.x + 512*2, point.y);
        }
        endShape();
      }
    }//END FOR
    float px=posicionesX/contadorPersonas;
    float py=posicionesY/contadorPersonas;
    if(contadorPersonas>0){
      // Send OSC message
        OscMessage msg = new OscMessage("/wek/inputs");
        msg.add(px);
        msg.add(py);
        oscP5.send(msg, dest);
         println("OSC message sent: X=" + px + ", Y=" + py);
    }
      
  }
 

  //Escribimos datos en pantalla
  noStroke();
  fill(0);
  rect(0, 0, 130, 100);
  fill(255, 0, 0);
  text("fps: "+frameRate, 20, 20);
  text("threshold: "+threshold, 20, 40);
  text("minD: "+minD, 20, 60);
  text("maxD: "+maxD, 20, 80);

  //Ponemos las distancias maximas y minimas a las que detectamos
  kinect.setLowThresholdPC(minD);
  kinect.setHighThresholdPC(maxD);
}
//END DRAW
////////////
