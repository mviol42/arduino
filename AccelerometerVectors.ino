#include <math.h>;
//Arm states
//0=Wiggle
//1=ready
//2=punching
//3=activate
int armState = 0;
int prevArmState = 0;
int a2 = A2;
int a3 = A3;
int a5 = A5;
int roundrobin = 0;
int numvals = 5;
int xvals[5];
int yvals[5];
int zvals[5];
int pin13 = 13;
boolean pin13state = false;
double xstats[6];
double ystats[6];
double zstats[6];
int readyThreshold = 20;
int punchThreshold = 250;
int stopThreshold  = 250;

int ledReady = 5;
int ledPunch = 6;
int ledActivate = 9;

double gravity[3];
double punchVector[3];
double stopVector[3];

double pg[3];
double sg[3];
//All time outs in milliseconds
long initialTime = 0;
int wiggleWait = 750;
int readyTimeOut = 1000;
int punchingTimeOut = 2000;
int activateTimeOut = 3000;
int ledTimeOut = 500;
double stopDegrees = 120;


boolean xon = true;
boolean yon = true;
boolean zon = true;

void setup() {
  Serial.begin(9600);
  setupAccelerometer();

  pinMode(pin13, OUTPUT);
  Serial.println(printHeader("x") + "\t " + printHeader("y") + "\t" + printHeader("z"));
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8,  OUTPUT);
  pinMode(7, OUTPUT);

}



void loop() {

  toggleLed();

  ledActivating();

  gatherAccelerometerVals();

  if (armState == 0) {
    Serial.println("Wiggling");
    if ((initialTime + wiggleWait < millis()) && detectReady()) {
      armState = 1;
      initialTime = millis();
      gravity[0] = xstats[2];
      gravity[1] = ystats[2];
      gravity[2] = zstats[2];


    }
  }
  else if (armState == 1) {
    Serial.println("Ready: " + String(gravity[0]) + ", " + String(gravity[1]) + ", " + String(gravity[2]));
    if (initialTime + readyTimeOut < millis()) {
      armState = 0;
      initialTime = millis();

    }
    if (detectPunch()) {
      armState = 2;
      initialTime = millis();


    }
  }
  else if (armState == 2) {
    Serial.println("Punching: " + String(punchVector[0]) + ", " + String(punchVector[1]) + ", " + String(punchVector[2]));
    if (initialTime + punchingTimeOut < millis()) {
      armState = 0;
      initialTime = millis();

    }
    if (detectStop()) {
      armState = 3;
      initialTime = millis();

    }
  }
  else if (armState == 3) {
    Serial.println("Activating: " + String(stopVector[0]) + ", " + String(stopVector[1]) + ", " + String(stopVector[2]));
    if (initialTime + activateTimeOut < millis()) {
      armState = 0;
      initialTime = millis();

    }
  }
}
boolean detectReady() {
  int sumRange = xstats[3] + ystats[3] + zstats[3];
  Serial.println(String(sumRange));
  if (sumRange < readyThreshold) {
    return true;
  }
  return false;
}
boolean detectPunch() {
  int sumRange = xstats[3] + ystats[3] + zstats[3];
  Serial.println(String(sumRange));
  if (sumRange > punchThreshold) {
    //Find maximum range among x stats, y stats, and z stats
    //Use those stats to get the index of our punch vector
    //Build punch vector from x vals, y vals, and z vals using punch index
    int punchIndex = 0;
    if (xstats[3] >= ystats[3] && xstats[3] >= zstats[3]) {
      punchIndex = findMaxDiffIndex(xstats);
    } else if (ystats[3] >= xstats[3] && ystats[3] >= zstats[3]) {
      punchIndex = findMaxDiffIndex(ystats);
    } else {
      punchIndex = findMaxDiffIndex(zstats);
    }
    punchVector[0] = xvals[punchIndex];
    punchVector[1] = yvals[punchIndex];
    punchVector[2] = zvals[punchIndex];

    return true;
  }
  return false;
}

boolean detectStop() {
  int sumRange = xstats[3] + ystats[3] + zstats[3];
  Serial.println(String(sumRange));
  if (sumRange > stopThreshold) {
    //Find maximum range among x stats, y stats, and z stats
    //Use those stats to get the index of our stop vector
    //Build stop vector from x vals, y vals, and z vals using stop index
    //Calculate thetaPunch using gravity and punch vector
    //Calculate thetaStop using gravity and stop vector
    //Sign of thetaPunch should be oppisite of thetaStop
    int stopIndex = 0;
    if (xstats[3] >= ystats[3] && xstats[3] >= zstats[3]) {
      stopIndex = findMaxDiffIndex(xstats);
    } else if (ystats[3] >= xstats[3] && ystats[3] >= zstats[3]) {
      stopIndex = findMaxDiffIndex(ystats);
    } else {
      stopIndex = findMaxDiffIndex(zstats);
    }
    stopVector[0] = xvals[stopIndex];
    stopVector[1] = yvals[stopIndex];
    stopVector[2] = zvals[stopIndex];
    Serial.println("Stop: " + String(stopVector[0]) + ", " + String(stopVector[1]) + ", " + String(stopVector[2]));

    for (int i = 0; i < 3; i++) {
      pg[i] = punchVector[i] - gravity[i];
      sg[i] = stopVector[i] - gravity[i];

    }
    double stopDotPunch = dotProduct(pg, sg);
    Serial.println("dotProduct: " + String(stopDotPunch));
    if (stopDotPunch > stopDegrees * PI / 180.0) {
      return true;
    }
  }
  return false;
}

int findMaxDiffIndex(double stats[]) {
  double minDif = abs(stats[0] - stats[2]);
  double maxDif = abs(stats[1] - stats[2]);
  if (minDif > maxDif) {
    return stats[4];
  }
  return stats[5];
  return false;
}

//Assume a 3-D vector
double magnitude(double vector[]) {
  return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}
double dotProduct(double vector1[], double vector2[]) {
  double mag1 = magnitude(vector1);
  double mag2 = magnitude(vector2);  // Check for zeros
  double cosineTheta = (vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[3] * vector2[3]) / (mag1 * mag2);
  double theta = acos(cosineTheta);
  return theta;

}
//X is index 0, Y is index 1, z is index 2
void crossProduct(double crossProduct[], double vector1[], double vector2[]) {
  crossProduct[0] = vector1[1] * vector2[2] - vector2[1] * vector1[2];
  crossProduct[1] = vector1[2] * vector2[0] - vector2[2] * vector1[0];
  crossProduct[2] = vector1[0] * vector2[1] - vector2[0] * vector1[1];

}
void computeStats(double stats[], int vals[], int vlength) {
  float vmin = 1024;
  float vmax = -1;
  float vsum = 0;
  int maxIndex = 0;
  int minIndex = 0;
  for (int i = 0; i < vlength; i++) {
    if (vmin > vals[i]) {
      vmin = vals[i];
      minIndex = i;
    }
    if (vmax < vals[i]) {
      vmax = vals[i];
      maxIndex = i;
    }
    vsum = vsum + vals[i];

  }
  stats[0] = vmin;
  stats[1] = vmax;
  stats[2] = vsum / vlength;
  stats[3] = abs(vmax - vmin);
  stats[4] = minIndex;
  stats[5] = maxIndex;
}
String printHeader(String axis) {
  return axis + "min\t " + axis + "max\t " + axis + "avg\t " + axis + "range ";
}
String printStats(float stats[]) {
  //  return (axis + ", min:" + String(stats[0]) + ",\tmax:" + String(stats[1]) + ",\tavg:" + String(stats[2]) + ",\trange:" + String(stats[3]));
  return  String(stats[0]) + "\t" + String(stats[1]) + "\t" + String(stats[2]) + "\t" + String(stats[3]);
}

void toggleLed() {
  if (pin13state) {
    pin13state = false;
    digitalWrite(13, LOW);
  } else {
    pin13state = true;
    digitalWrite(13, HIGH);
  }
}
void setupAccelerometer() {
  if (xon) {
    pinMode(a2, INPUT);
  }
  if (yon) {
    pinMode(a3, INPUT);
  }
  if (zon) {
    pinMode(a5, INPUT);
  }
}

int skipPrint = 0;
void gatherAccelerometerVals() {
  if (xon) {
    xvals[roundrobin] = analogRead(a2);
    computeStats(xstats, xvals, numvals);
    //    printStats("x", xstats);
  }
  if (yon) {
    yvals[roundrobin] = analogRead(a3);
    computeStats(ystats, yvals, numvals);
    //    printStats("y", ystats);
  }
  if (zon) {
    zvals[roundrobin] = analogRead(a5);
    computeStats(zstats, zvals, numvals);
    //    printStats("z", zstats);
  }
  //  Serial.println(String(roundrobin) + ",\t" + String(xvals[roundrobin]) + ",\t" + String(yvals[roundrobin]) + ",\t" +  String(zvals[roundrobin]));
  if (skipPrint % 20 == 0) {
    skipPrint = 0;
    //    Serial.println(printStats(xstats) + "\t" + printStats(ystats) + "\t" + printStats(zstats));
  } else {
    skipPrint++;
  }

  roundrobin++;
  if (roundrobin > numvals - 1) {
    roundrobin = 0;
  }
}

void ledActivating() {
  if (armState == 0 && prevArmState!= 0) {
    prevArmState = 0;
    digitalWrite(ledReady, HIGH);
    digitalWrite(ledPunch, LOW);
    digitalWrite(ledActivate, LOW);
    Serial.println("Wiggling Led On");
//    if (initialTime + ledTimeOut < millis()) {
//      digitalWrite(10, LOW);
//      digitalWrite(9, LOW);
//      digitalWrite(8, LOW);
//      digitalWrite(7, LOW);
//      Serial.println("Wiggling Led Off");
//
//    }
  }
  else if (armState == 1 && prevArmState!= 1) {
    prevArmState = 1;
  
    digitalWrite(ledReady, HIGH);
    digitalWrite(ledPunch , HIGH);
    digitalWrite(ledActivate, LOW);
    Serial.println("Ready Led");
  }
  else if (armState == 2 && prevArmState!= 2) {
    prevArmState = 2;

    digitalWrite(ledReady, HIGH);
    digitalWrite(ledPunch , HIGH);
    digitalWrite(ledActivate, HIGH);
    initialTime = millis();
    Serial.println("Punch Led");

  }
//ledReady 
//ledPunch 
//ledActivate
//    if (initialTime + ledTimeOut < millis()) {
//      digitalWrite(10, LOW);
//      digitalWrite(9, LOW);
//      digitalWrite(8, LOW);
//      digitalWrite(7, LOW);
//      Serial.println("Activate Led Off");
//
//    }
  
}


