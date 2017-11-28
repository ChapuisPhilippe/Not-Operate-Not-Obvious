class Repere{
  float x;
  float y;
  float z;
  float lmax; 
  int xDirection =    1;
  int yDirection =    1;
  int zDirection =    1;

Repere (float tempx, float tempy, float tempz, float templmax){
  x     = tempx;
  y     = tempy;
  z     = tempz;
  lmax  = templmax;
}
  // Methode action de representation des vecteurs le point rouge origine, la ligne = la direction
  void display() {
  //noStroke();
    fill(55);
    ellipse(x, y , 6, 6);
    stroke(255,0,0);
    line(x , y , z, x+(lmax*xDirection), y, z );
    text("X",x+(lmax*xDirection),y, z);
    stroke(0,255,0); 
    line(x , y , z, x, y+(lmax*yDirection), z );
    text("Y",x , y+(lmax*yDirection), z);
    stroke(0,0,255);
    line(x , y , z, x, y ,z + (lmax*zDirection) );
    text("Z",x, y , z+(lmax*zDirection));
    sphere(5);
}
}

