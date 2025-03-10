import processing.serial.*;

float[] fingerAngles = new float[15]; // 15 values (5 fingers * 3 angles)
Serial myPort; // The serial port

// Variables for camera control
float cameraAngleX = 0;
float cameraAngleY = 0;
float previousMouseX;
float previousMouseY;
boolean isDragging = false;

// Scale multiplier for the palm
float palmScale = 3.5; // You can adjust this value to scale the palm

// Position offset for the palm
PVector palmOffset = new PVector(20, 120, 50); // You can adjust these values to reposition the palm

void setup() {
  size(1200, 1000, P3D); // Increased canvas size for a bigger hand

  // Initialize serial communication
  String portName = "COM3"; // Adjust this to match your port
  myPort = new Serial(this, portName, 115200);
  myPort.bufferUntil('\n');
}

void draw() {
  background(255);
  lights();
  translate(width / 2, height / 2, -600); // Center the scene and move back for a larger view

  // Apply camera rotations based on mouse interaction
  rotateX(cameraAngleX);
  rotateY(cameraAngleY);

  // Draw hand base (palm)
  fill(210, 180, 140); // Skin tone color for palm
  stroke(160, 130, 100);
  strokeWeight(1);
  drawPalmMesh();
  
  // Draw fingers, each starting at different points on the hand base with different lengths
  drawFinger(-80, 60, 50, fingerAngles[0], fingerAngles[1], fingerAngles[2], 70, 60, 50); // Thumb (still controlled by sliders)
  drawFinger(-40, 0, 70, fingerAngles[3], fingerAngles[4], fingerAngles[5], 100, 90, 80); // Index Finger
  drawFinger(0, 0, 80, fingerAngles[6], fingerAngles[7], fingerAngles[8], 110, 100, 90); // Middle Finger
  drawFinger(40, 0, 70, fingerAngles[9], fingerAngles[10], fingerAngles[11], 100, 90, 80); // Ring Finger
  drawFinger(80, 0, 60, fingerAngles[12], fingerAngles[13], fingerAngles[14], 90, 80, 70); // Little Finger
}

void drawPalmMesh() {
  pushMatrix();
  translate(palmOffset.x, palmOffset.y, palmOffset.z); // Apply the position offset to the palm
  PVector[] ringBase = {
    new PVector(-30 * palmScale, 30 * palmScale, -5 * palmScale),
    new PVector(-15 * palmScale, 30 * palmScale, 3 * palmScale),
    new PVector(0 * palmScale, 30 * palmScale, 6 * palmScale),
    new PVector(15 * palmScale, 30 * palmScale, 3 * palmScale),
    new PVector(30 * palmScale, 30 * palmScale, -5 * palmScale)
  };
  PVector[] ringKnucklesArr = {
    new PVector(-25 * palmScale, -35 * palmScale, 2 * palmScale),
    new PVector(-10 * palmScale, -35 * palmScale, 10 * palmScale),
    new PVector(10 * palmScale, -35 * palmScale, 10 * palmScale),
    new PVector(25 * palmScale, -35 * palmScale, 2 * palmScale)
  };
  PVector[] ringKnuckles = (PVector[]) append(ringKnucklesArr, new PVector(0 * palmScale, -35 * palmScale, -5 * palmScale));
  PShape palmMesh = createShape(GROUP);
  palmMesh.addChild(connectRings(ringBase, ringKnuckles));
  palmMesh.addChild(capRing(ringBase, true));
  palmMesh.addChild(capRing(ringKnuckles, false));
  shape(palmMesh);
  popMatrix();
}

PShape connectRings(PVector[] ringA, PVector[] ringB) {
  PShape sh = createShape();
  sh.beginShape(QUADS);
  for (int i = 0; i < ringA.length; i++) {
    sh.vertex(ringA[i].x, ringA[i].y, ringA[i].z);
    sh.vertex(ringA[(i + 1) % ringA.length].x, ringA[(i + 1) % ringA.length].y, ringA[(i + 1) % ringA.length].z);
    sh.vertex(ringB[(i + 1) % ringB.length].x, ringB[(i + 1) % ringB.length].y, ringB[(i + 1) % ringB.length].z);
    sh.vertex(ringB[i].x, ringB[i].y, ringB[i].z);
  }
  sh.endShape(CLOSE);
  return sh;
}

PShape capRing(PVector[] ring, boolean invert) {
  PShape cap = createShape();
  cap.beginShape(TRIANGLE_FAN);
  PVector center = new PVector();
  for (PVector v : ring) center.add(v);
  center.div(ring.length);
  cap.vertex(center.x, center.y, center.z);
  for (int i = 0; i <= ring.length; i++)
    cap.vertex(ring[(invert ? ring.length - i : i) % ring.length].x, ring[(invert ? ring.length - i : i) % ring.length].y, ring[(invert ? ring.length - i : i) % ring.length].z);
  cap.endShape(CLOSE);
  return cap;
}

void drawFinger(float startX, float startY, float startZ, float joint1, float joint2, float joint3, float length1, float length2, float length3) {
  pushMatrix();
  translate(startX, startY, startZ);

  // Segment 1
  rotateX(radians(joint1));
  box(20, length1, 20);
  translate(0, -length1 / 2, 0);
  
  // Segment 2
  rotateX(radians(joint2));
  translate(0, -length1 / 2, 0);
  box(20, length2, 20);
  translate(0, -length2 / 2, 0);
  
  // Segment 3
  rotateX(radians(joint3));
  translate(0, -length2 / 2, 0);
  box(20, length3, 20);
  
  popMatrix();
}

void serialEvent(Serial myPort) {
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    inString = trim(inString);
    String[] sensors = split(inString, ',');
    
    // Remove any empty strings at the end of the array
    if (sensors[sensors.length - 1].equals("")) {
      sensors = shorten(sensors);
    }
    
    if (sensors.length == 8) {
      for (int i = 0; i < 4; i++) {
        float midSegmentValue = float(sensors[i * 2]);
        float lowerSegmentValue = float(sensors[i * 2 + 1]);
        fingerAngles[3 + (i * 3)] = (lowerSegmentValue)*0.5; // Adjust the angle based on your calibration
        fingerAngles[4 + (i * 3)] = (midSegmentValue)*0.5;
        fingerAngles[5 + (i * 3)] = fingerAngles[4 + i * 3] * 0.5; // Auto bend upper segment
      }
    }
  }
}
void mousePressed() {
  isDragging = true;
  previousMouseX = mouseX;
  previousMouseY = mouseY;
}

void mouseDragged() {
  if (isDragging) {
    float deltaX = mouseX - previousMouseX;
    float deltaY = mouseY - previousMouseY;
    cameraAngleY += radians(deltaX) * 0.2; // Increased multiplier to make dragging more effective
    cameraAngleX -= radians(deltaY) * 0.2; // Increased multiplier to make dragging more effective
    previousMouseX = mouseX;
    previousMouseY = mouseY;
  }
}

void mouseReleased() {
  isDragging = false;
}
