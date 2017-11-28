/**
 * Robot 3D 
 * 
 * 
 *  ICI le modèle très simple represente un objet Avion avec commande en propulsion et d'ailerons uniquement Autonomous Unit Vehicule
 *  On a 4*3 = 12 variables d'état, 
 * 
 */
 import Jama.*;  
// geometrique data

float l    =  50.0;
float sval =   1.0;
float dtl  =  0.01;

boolean  recording = false;


// le repère et 1es objets dans une liste Cube[] 

Repere myrep      = new Repere(0,0,0,l);
A_U_V[] unit_auv  = new A_U_V[1]; 


// Dimension du cube
float bounds = 600;

void setup() {
  size(800, 800, P3D);

  for (int i = 0; i < unit_auv.length; i++) {
            
    unit_auv[i] =  new A_U_V(i+1);
  }

}

void draw() {
  background(255);
  lights();


  pushMatrix();
  // On centre tout
  
  translate(width/2, height/2, 00.0);
  myrep.display();




  
  if(keyPressed) // {  sval += 0.01;} // else { sval -= 0.01;}
  sval = constrain(sval, 0.50, 20.0);
  scale(sval);
  rotateY(map(mouseX,0, width,-PI, PI));
  rotateX(map(mouseY,0,height, PI,-PI));  
//  myrep.display();
   fill(255,0,255);
   line(0,0,0,0,200,0)  ; text("x_Gal",  0,200,  0);
   line(0,0,0,200,0,0)  ; text("y_Gal",200,  0,  0);
   line(0,0,0,0,0,200)  ; text("z_Gal",  0,  0,200);
   noFill();
     pushMatrix();
      translate(0,0, -300.0);
      ellipse(0,0,600,600);
      ellipse(0,0,1200,1200);
      popMatrix();

  stroke(255);


  // Cube externe transparant pour les bornes 
  noFill(); 
  stroke(255,0,255);
  box(bounds);

  // Boucle sur tous les objets
  
  for (A_U_V c : unit_auv) {
    c.update();
    c.display();
  }
 popMatrix();
 
   if (recording) {
    saveFrame("output/frames####.png");
  } 
  
}
void keyPressed() {
  
  // If we press r, start or stop recording!
  if (key == 'r' || key == 'R') {
    recording = !recording;
  }
    if (key == 'u' || key == 'U') {
   {  sval += 0.01;}
  }
      if (key == 'd' || key == 'D') {
   {  sval -= 0.01;}
  }
}


