import processing.serial.*;

Serial myPort;
String serialData = "";

float roll = 0, pitch = 0, yaw = 0;
float qw = 1, qx = 0, qy = 0, qz = 0;
float confidence = 0;
float updateRate = 0;

float smoothRoll = 0, smoothPitch = 0, smoothYaw = 0;
float smoothing = 0.85;

boolean showAxes = true;
boolean showGrid = true;
boolean showInfo = true;
boolean darkMode = true;
int backgroundColor;
int textColor;

float camDistance = 400;
float camRotX = 0;
float camRotY = 0;
int lastMouseX, lastMouseY;

void setup() {
  size(1200, 800, P3D);
  
  println("Available serial ports:");
  printArray(Serial.list());
  
  try {
    myPort = new Serial(this, "COM3", 115200);
    myPort.bufferUntil('\n');
    println("\nConnected to: " + Serial.list()[0]);
  } catch (Exception e) {
    println("ERROR: Could not open serial port!");
  }
  
  updateColorScheme();
  textAlign(LEFT, TOP);
  
  println("\nControls:");
  println("Mouse Drag: Rotate view");
  println("SPACE: Dark/Light mode");
  println("A: Toggle axes");
  println("G: Toggle grid");
  println("R: Reset camera");
}

void draw() {
  background(backgroundColor);
  
  // Don't smooth for now - debug with raw values
  // smoothRoll = lerp(smoothRoll, roll, 1 - smoothing);
  // smoothPitch = lerp(smoothPitch, pitch, 1 - smoothing);
  // smoothYaw = lerp(smoothYaw, yaw, 1 - smoothing);
  
  pushMatrix();
  
  translate(width/2, height/2, 0);
  rotateX(radians(camRotX));
  rotateY(radians(camRotY));
  translate(0, 0, -camDistance);
  
  lights();
  ambientLight(100, 100, 100);
  directionalLight(200, 200, 200, -1, 1, -1);
  
  if (showGrid) {
    drawGrid();
  }
  
  if (showAxes) {
    drawWorldAxes();
  }
  
  // APPLY QUATERNION ROTATION DIRECTLY
  applyQuaternionRotation(qw, qx, qy, qz);
  
  drawIMU();
  
  if (showAxes) {
    drawLocalAxes();
  }
  
  popMatrix();
  
  if (showInfo) {
    drawInfoPanel();
  }
  
  drawFPS();
}

// ADD THIS NEW FUNCTION
void applyQuaternionRotation(float w, float x, float y, float z) {
  // Convert quaternion to rotation matrix
  // This is the correct way to apply quaternion rotation in Processing
  
  float xx = x * x;
  float xy = x * y;
  float xz = x * z;
  float xw = x * w;
  float yy = y * y;
  float yz = y * z;
  float yw = y * w;
  float zz = z * z;
  float zw = z * w;
  
  // Build rotation matrix
  applyMatrix(
    1 - 2*(yy + zz),     2*(xy - zw),     2*(xz + yw), 0,
        2*(xy + zw), 1 - 2*(xx + zz),     2*(yz - xw), 0,
        2*(xz - yw),     2*(yz + xw), 1 - 2*(xx + yy), 0,
                  0,               0,               0, 1
  );
}

void drawIMU() {
  pushMatrix();
  
  fill(70, 130, 220, 230);
  stroke(255);
  strokeWeight(2);
  box(200, 50, 100);
  
  pushMatrix();
  translate(0, -25, 0);
  fill(255, 70, 70, 230);
  noStroke();
  box(180, 5, 80);
  popMatrix();
  
  pushMatrix();
  translate(0, 0, 50);
  fill(70, 255, 70, 230);
  noStroke();
  box(30, 30, 5);
  popMatrix();
  
  fill(40, 40, 40);
  pushMatrix();
  translate(-80, 0, 40);
  sphere(5);
  popMatrix();
  
  pushMatrix();
  translate(80, 0, 40);
  sphere(5);
  popMatrix();
  
  pushMatrix();
  translate(-80, 0, -40);
  sphere(5);
  popMatrix();
  
  pushMatrix();
  translate(80, 0, -40);
  sphere(5);
  popMatrix();
  
  pushMatrix();
  translate(0, -27, 0);
  fill(30, 30, 30);
  noStroke();
  box(40, 3, 40);
  popMatrix();
  
  popMatrix();
}

void drawWorldAxes() {
  strokeWeight(3);
  
  // X axis (Red) - pointing right
  stroke(255, 0, 0);
  line(0, 0, 0, 200, 0, 0);
  pushMatrix();
  translate(220, 0, 0);
  fill(255, 0, 0);
  noStroke();
  sphere(8);
  popMatrix();
  
  // Y axis (Green) - pointing away (into screen)
  stroke(0, 255, 0);
  line(0, 0, 0, 0, 0, -200);  // Negative Z in Processing = away
  pushMatrix();
  translate(0, 0, -220);
  fill(0, 255, 0);
  noStroke();
  sphere(8);
  popMatrix();
  
  // Z axis (Blue) - pointing up
  stroke(0, 0, 255);
  line(0, 0, 0, 0, -200, 0);  // Negative Y in Processing = up
  pushMatrix();
  translate(0, -220, 0);
  fill(0, 0, 255);
  noStroke();
  sphere(8);
  popMatrix();
  
  // Axis labels
  fill(textColor);
  textSize(20);
  
  pushMatrix();
  translate(230, 0, 0);
  rotateY(-radians(camRotY));
  rotateX(-radians(camRotX));
  text("X", 0, 0);
  popMatrix();
  
  pushMatrix();
  translate(0, 0, -230);
  rotateY(-radians(camRotY));
  rotateX(-radians(camRotX));
  text("Y", 0, 0);
  popMatrix();
  
  pushMatrix();
  translate(0, -230, 0);
  rotateY(-radians(camRotY));
  rotateX(-radians(camRotX));
  text("Z", 0, 0);
  popMatrix();
}

void drawLocalAxes() {
  strokeWeight(2);
  
  stroke(255, 100, 100);
  line(0, 0, 0, 120, 0, 0);
  
  stroke(100, 255, 100);
  line(0, 0, 0, 0, 120, 0);
  
  stroke(100, 100, 255);
  line(0, 0, 0, 0, 0, 120);
}

void drawGrid() {
  stroke(darkMode ? 80 : 180);
  strokeWeight(1);
  
  int gridSize = 600;
  int gridStep = 50;
  
  pushMatrix();
  translate(0, 100, 0);
  rotateX(HALF_PI);
  
  for (int i = -gridSize; i <= gridSize; i += gridStep) {
    line(i, -gridSize, 0, i, gridSize, 0);
    line(-gridSize, i, 0, gridSize, i, 0);
  }
  
  popMatrix();
}

void drawInfoPanel() {
  fill(darkMode ? color(0, 0, 0, 200) : color(255, 255, 255, 220));
  noStroke();
  rect(10, 10, 350, 260, 10);
  
  fill(textColor);
  textSize(16);
  textAlign(LEFT, TOP);
  
  int y = 25;
  int lineHeight = 25;
  
  textSize(20);
  fill(darkMode ? color(100, 200, 255) : color(0, 100, 200));
  text("IMU Sensor Fusion", 25, y);
  y += 35;
  
  textSize(16);
  fill(textColor);
  text("Euler Angles (degrees):", 25, y);
  y += lineHeight;
  
  fill(255, 100, 100);
  text("  Roll:  " + nf(roll, 0, 2) + " deg", 25, y);
  y += lineHeight;
  
  fill(100, 255, 100);
  text("  Pitch: " + nf(pitch, 0, 2) + " deg", 25, y);
  y += lineHeight;
  
  fill(100, 100, 255);
  text("  Yaw:   " + nf(yaw, 0, 2) + " deg", 25, y);
  y += lineHeight + 10;
  
  fill(textColor);
  text("Quaternion:", 25, y);
  y += lineHeight;
  text("  w: " + nf(qw, 0, 4), 25, y);
  y += lineHeight;
  text("  x: " + nf(qx, 0, 4) + "  y: " + nf(qy, 0, 4) + "  z: " + nf(qz, 0, 4), 25, y);
  y += lineHeight + 10;
  
  fill(textColor);
  text("Confidence: " + nf(confidence, 0, 1) + "%", 25, y);
  y += lineHeight;
  
  if (updateRate > 90) {
    fill(100, 255, 100);
  } else if (updateRate > 50) {
    fill(255, 200, 100);
  } else {
    fill(255, 100, 100);
  }
  text("Update Rate: " + nf(updateRate, 0, 1) + " Hz", 25, y);
}

void drawFPS() {
  fill(textColor);
  textSize(14);
  textAlign(RIGHT, TOP);
  text("FPS: " + nf(frameRate, 0, 1), width - 20, 20);
  
  textAlign(LEFT, BOTTOM);
  if (myPort != null && myPort.active()) {
    fill(100, 255, 100);
    text("Connected", 20, height - 20);
  } else {
    fill(255, 100, 100);
    text("Disconnected", 20, height - 20);
  }
  
  textAlign(CENTER, BOTTOM);
  fill(textColor, 150);
  textSize(12);
  text("SPACE: light/dark | H: help", width/2, height - 10);
}

void serialEvent(Serial myPort) {
  String data = myPort.readStringUntil('\n');
  
  if (data != null) {
    data = trim(data);
    
    if (!data.startsWith("{")) {
      return;
    }
    
    try {
      JSONObject json = parseJSONObject(data);
      
      if (json != null) {
        JSONObject quat = json.getJSONObject("quat");
        if (quat != null) {
          qw = quat.getFloat("w");
          qx = quat.getFloat("x");
          qy = quat.getFloat("y");
          qz = quat.getFloat("z");
        }
        
        JSONObject euler = json.getJSONObject("euler");
        if (euler != null) {
          roll = euler.getFloat("roll");
          pitch = euler.getFloat("pitch");
          yaw = euler.getFloat("yaw");
        }
        
        confidence = json.getFloat("confidence");
        updateRate = json.getFloat("rate");
      }
    } catch (Exception e) {
      // Ignore parse errors
    }
  }
}

void mouseDragged() {
  if (mouseButton == LEFT) {
    camRotY += (mouseX - lastMouseX) * 0.5;
    camRotX -= (mouseY - lastMouseY) * 0.5;
    camRotX = constrain(camRotX, -89, 89);
  }
  
  lastMouseX = mouseX;
  lastMouseY = mouseY;
}

void mousePressed() {
  lastMouseX = mouseX;
  lastMouseY = mouseY;
}

void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  camDistance += e * 20;
  camDistance = constrain(camDistance, 100, 1000);
}

void keyPressed() {
  if (key == ' ') {
    darkMode = !darkMode;
    updateColorScheme();
  } else if (key == 'a' || key == 'A') {
    showAxes = !showAxes;
  } else if (key == 'g' || key == 'G') {
    showGrid = !showGrid;
  } else if (key == 'i' || key == 'I') {
    showInfo = !showInfo;
  } else if (key == 'r' || key == 'R') {
    camRotX = 0;
    camRotY = 0;
    camDistance = 400;
  } else if (key == 'h' || key == 'H') {
    printHelp();
  } else if (key == 's' || key == 'S') {
    saveFrame("IMU_screenshot_####.png");
    println("Screenshot saved!");
  }
}

void updateColorScheme() {
  if (darkMode) {
    backgroundColor = color(20, 20, 30);
    textColor = color(255);
  } else {
    backgroundColor = color(240, 240, 245);
    textColor = color(0);
  }
}

void printHelp() {
  println("\nCONTROLS:");
  println("Mouse Drag: Rotate camera");
  println("Mouse Wheel: Zoom");
  println("SPACE: Toggle dark/light");
  println("A: Toggle axes");
  println("G: Toggle grid");
  println("I: Toggle info");
  println("R: Reset camera");
  println("S: Screenshot");
}