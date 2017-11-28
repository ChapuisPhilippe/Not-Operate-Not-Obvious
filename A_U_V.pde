
//  Class A_U_V
//
//  Modèle de vecteur d'etat  = px,py,pz,psi,theta,phi,v  position cdg + orientation dans galileen + vitesse suivant x
//  Ici on suppose 
//

class A_U_V {

   int number;

  // Position, velocity vectors
  // PVector pour les variables angulaires psi theta phi et les derivées des variables angulaires
  // PVector pour le tau de rotation o
  // PVector pour la commande. 

  PVector pos     ;  // Position du Cdg dans le repère Galileen (3 variables d'etat)
  PVector r       ;  // Angles d'Euler (3 variables d'etat psi theta phi)
  PVector v       ;  // Vitesse du Cdg dans le repère local  
  PVector o       ;  // Vecteur instantané de rotation omega (o1,o2,o3)
  PVector U       ;  // Les entrées en acceleration u1 acceleration tangentielle u2 , u3 les gouvernes d'elevons

// Constructeur de la classe

  A_U_V(int number) {
  
    this.number    = number;

    pos    = new PVector( 0.00      , 0.00 ,   -.5)  ;
    r      = new PVector( 0.00      , 0.00 ,   0.0) ; // ici on a phi theta psi
    v      = new PVector( 10.00     , 0.00 ,   0.00)  ;
    o      = new PVector( 0.00      , 0.00 ,   0.00)  ; // ici on omega 1 rot autour de xloc omega 2 rot autour de yloc omega 3 rot autour de zloc 
    U      = new PVector( 0.00      , 0.00 ,   0.00)  ;
   } 
   
   
    
  // Fonction Update c'est la boucle D'euler qui porte sur le controle et le vecteur d'etat. 

  void update() { 

// Test fonctionnement sur les variables d'etats pour debugage graphique ------ une commande symetrique doit :
// En braquant les ailerons vers le haut l'aeronef prend du tangage positif. -- une commande differentielle doit :
// provoquer du roulis lorsque la voilure plongeante a son aileron vers le haut (à cabrer) donne le sens du roulis


//  pos.x += 0.01;     //x dans rep galileen 
//  pos.y += 0.01;     //y dans rep galilen 
//  pos.z += 0.01;     //z dans rep galileen positif vers le bas
//  r.x += -0.01;      //Angle phi gite roulis 
//  r.y += 0.005;       //Angle tetha assiette tangage 
//  r.z += 0.01;       //Angle psi Cap Lacet
//    U.x =   10.00 ;
//    U.y =   -0.05 ;
//    U.z =   -0.01;
//  o.x += 0.01;      //Omega1 
//  o.y += 0.1;       //Omega2 
//  o.z += 0.01;      //Omega3

   calcul_U(pos,r,v,U)      ; 

   euler(pos,r,v,o,U)   ; 

   
// ------------- Euler equations d'evolution ------------------


  }
  
  //----------------------------- Commande on fabrique U u1,u2,u3 propulsion,assiette,gite en consignes bornées -------------------------
  
  void calcul_U(PVector pos, PVector r,PVector v,PVector U) {

  float vbar     =  30.0;
  float rbar     = 300.0; 
  float zbar     = -300;
  

  float psibar   =  atan2(pos.y,pos.x) + atan((sqrt(pos.x*pos.x+pos.y*pos.y)-rbar)/50) + 1*PI/2 ;

  float thetabar =  -0.20*atan((zbar-pos.z)/10); 
  float phibar   =   0.5*atan(5*(atan(tan(0.5*psibar-0.5*r.z))));

   U.x =    10.00*(1+(2/PI)*atan(vbar-v.mag()));
   
   U.y =  - 0.3*((2/PI)*atan(5*(thetabar - r.y)) + abs(sin(r.x)));
   
   U.z =  - 0.3*((2/PI)*atan(phibar - r.x)); 

//    U.x =   15.00 ;
//    U.y =   0.01 ;
//    U.z =   0.100;

// println(U.x,U.y,U.z);
  }

  void euler(PVector pos,PVector r,PVector v ,PVector o, PVector U) {
   

    float alpha ;
    alpha = atan(v.z/v.x); 
    
    float beta  = asin(v.y/v.mag());    
//    println(alpha,beta);

    Matrix X         = new Matrix(3,1);
    Matrix dX        = new Matrix(3,1); //
    Matrix Angle     = new Matrix(3,1);
    Matrix dAngle    = new Matrix(3,1); //
    Matrix vit       = new Matrix(3,1);
    Matrix acc       = new Matrix(3,1); //
    Matrix o2        = new Matrix(3,1);
    Matrix vo2       = new Matrix(3,1); //
    Matrix FG        = new Matrix(3,1); //  
    Matrix FU        = new Matrix(3,1); //    
    Matrix FA        = new Matrix(3,1); // 
    Matrix Ucom      = new Matrix(3,1); //     
    Matrix ovecV     = new Matrix(3,1); // 

    double[][] ValRot    = {{0.,0.,0.},{0.,0.,0.},{0.,0.,0.}} ; 
    double[][] ValRphi   = {{1.,0.,0.},{0.,cos(r.x),-sin(r.x)},{0.,sin(r.x),cos(r.x)}} ; 
    double[][] ValRteta  = {{cos(r.y),0.,sin(r.y)},{0.,1.,0.},{-sin(r.y),0.,cos(r.y)}} ;
    double[][] ValRpsi   = {{cos(r.z),-sin(r.z),0.},{sin(r.z),cos(r.z),0.},{0.,0.,1.}} ;
    
    double[][] ValProjA  = {{1.,tan(r.y)*sin(r.x),tan(r.y)*cos(r.x)},{0.,cos(r.x),-sin(r.x)},{0.,sin(r.x)/cos(r.y),cos(r.x)/cos(r.y)}} ; 
    double[][] ValProjB  = {{-cos(alpha)*cos(beta),cos(alpha)*sin(beta),sin(alpha)},{sin(beta),cos(beta),0.0},{-sin(alpha)*cos(beta),sin(alpha)*sin(beta),-cos(alpha)}} ; 


    Matrix Rotphi  = new Matrix(ValRphi); //initialisations en local
    Matrix Rotteta = new Matrix(ValRteta);
    Matrix Rotpsi  = new Matrix(ValRpsi);
    Matrix Rotat   = new Matrix(ValRot);
    Matrix ProjA   = new Matrix(ValProjA);
    Matrix ProjB   = new Matrix(ValProjB);   
    
    Rotat = Rotpsi.times((Rotteta).times(Rotphi));  

//    Rotphi.print(3,4);

//  initialisation des variables locales
    
    X.set(0,0,pos.x) ; 
    X.set(1,0,pos.y) ;
    X.set(2,0,pos.z) ; 
//    X.print(3,4);
    Angle.set(0,0,r.x) ; 
    Angle.set(1,0,r.y) ;
    Angle.set(2,0,r.z) ;     
//    Angle.print(3,4)   ;

    vit.set(0,0,v.x) ; 
    vit.set(1,0,v.y) ;
    vit.set(2,0,v.z) ;  
//    vit.print(3,4)   ;
    
    o2.set(0,0,o.x) ; 
    o2.set(1,0,o.y) ;
    o2.set(2,0,o.z) ;    
//    o2.print(3,4)   ;

    dX     = Rotat.times(vit);
//    dX.print(3,6);
    
    dAngle = ProjA.times(o2);

//    dAngle.print(3,6);

    FG.set(0,0,        -sin(r.y)) ; 
    FG.set(1,0,cos(r.y)*sin(r.x)) ;
    FG.set(2,0,cos(r.y)*cos(r.x)) ;   
//     FG.print(3,6);    

    float vari1 = -0.3 + (10.0*alpha) + (10*o.y/v.mag()) + (2.0*U.z) + (0.3*U.y);

    float fu1   =  4.0 + pow(vari1,2) + abs(U.y) + 3 * abs(U.z);
    float fu2   = -50*beta + (10*o.z-3*o.x)/v.mag();
    float fu3   =  10 + 500*alpha + 400*o.y/v.mag()+ 50*U.z +10*U.y;
 
//    println(fu1,fu2,fu3);  
    
    FU.set(0,0,fu1) ; 
    FU.set(1,0,fu2) ;
    FU.set(2,0,fu3) ;  
//    FU.print(3,6);  
    Ucom.set(0,0,U.x) ; 
    Ucom.set(1,0,0) ;
    Ucom.set(2,0,0) ;  
    
    ovecV.set(0,0,-o.z*v.y + o.y*v.z) ; 
    ovecV.set(1,0,+o.z*v.x - o.x*v.z) ;
    ovecV.set(2,0,-o.y*v.x + o.x*v.y) ;   
    
    double coeff = v.mag()*v.mag()/500.0;
    double grav  = 9.81 ;
    
    FU.timesEquals(coeff);
    
    FA  = ProjB.times(FU);   
//    FA.print(3,6);     

    FG.timesEquals(grav);
//    FG.print(3,6); 

    acc = FG.plus(FA).plus(Ucom).minus(ovecV);

//    acc.print(3,5);  

    vo2.set(0,0,-o.z*o.y    - (v.mag()*v.mag()/10) * (       beta+  2*U.z +       (5*o.x-o.z)/v.mag())) ; 
    vo2.set(1,0, o.z*o.x    - (v.mag()*v.mag()/10) * (0.1+2*alpha-0.2*U.z + 3*U.y +   30*o.y /v.mag())) ;
    vo2.set(2,0, o.x*o.y/10 + (v.mag()*v.mag()/10) * (       beta+0.5*U.z +     (0.5*o.x-o.z)/v.mag())) ;   

// P(k+1) = P(k)+dP*delta temps boucle d'euler

     dX.timesEquals(dtl);
     dAngle.timesEquals(dtl);
     acc.timesEquals(dtl);
     vo2.timesEquals(dtl);
    
//    vit.print(3,6);   

    double px = dX.get(0,0); pos.x += (float)px;
    double py = dX.get(1,0); pos.y += (float)py;
    double pz = dX.get(2,0); pos.z += (float)pz;
    
    double rx = dAngle.get(0,0); r.x += (float)rx;
    double ry = dAngle.get(1,0); r.y += (float)ry;
    double rz = dAngle.get(2,0); r.z += (float)rz;
    
    double vx = acc.get(0,0); v.x += (float)vx;
    double vy = acc.get(1,0); v.y += (float)vy;
    double vz = acc.get(2,0); v.z += (float)vz;

    double ox = vo2.get(0,0);
    double oy = vo2.get(1,0); 
    double oz = vo2.get(2,0); 

     o.x += (float)ox;
     o.y += (float)oy;
     o.z += (float)oz;
     
     
//    println(pos.x,pos.y,pos.z,r.x,r.y,r.z,v.x,v.y,v.z,ox,oy,oz);


  }  



//----------------------- geometrie et tracés

  // La forme de l'avion
  void draw_A_U_V(PVector U ) {

    //    Dessin de l'avion
    //    Transformation pour obtenir le dessin conforme au referentiel direct choisit. On calcule dans le galileen confondu à l'origine avec le repère local de positionnement

 
   pushMatrix();
    stroke(255,0,255);
    line(0,0,0,0,100, 0)  ; text("x_loc1",0, 100, 0);
    line(0,0,0,100,0, 0)  ; text("y_loc1", 100,0,  0);
    line(0,0,0,0, 0,100)  ; text("z_loc1", 0, 0,100);
    rotateZ(PI/2);    

//   Repere locrep      = new Repere(0,0,0,l/2);   
     scale(.25);   
//   locrep.display(); 
   
   // Corps central en rouge
      fill(255, 0, 0, 200);
      box(200, 15, 15);
    
    // Pointe avant
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(140, 0, 0);
    rotateZ(PI/2);
    drawCylinder(0, 10, 40, 10);
    popMatrix();
    
    // draw aileron gauche 

    fill(0, 0, 255, 200);
    pushMatrix();
    translate(-50, 60, 0);
    rotateZ(HALF_PI); 
    rotateX(-(-U.y + U.z));
    translate( -5, 0, 30 * sin(-(-U.y + U.z)));
    box(90,30,1);
    line(0,0,0,0,100,  0)  ; text("x_El_g", 0,100, 0);  // x et y inversé car on est ici dans rep screen
    line(0,0,0,100,0,  0)  ; text("y_El_g", 100,0, 0);
    line(0,0,0,0, 0, 100)  ; text("z_El_g", 0, 0,100);
    popMatrix();

    // draw aileron droit 

    fill(0, 0, 255, 200);
    pushMatrix();
    translate(-50, -60, 0);
    rotateZ(HALF_PI);
    rotateX(-(-U.y - U.z));
    translate( 5, 0,  30 * sin(-(-U.y - U.z)));
    box(90,30,1);
    line(0,0,0,0,100,  0)  ; text("x_El_d", 0,100, 0);  // x et y inversé car on est ici dans rep screen
    line(0,0,0,100,0,  0)  ; text("y_El_d", 100,0, 0);
    line(0,0,0,0, 0, 100)  ; text("z_El_d", 0, 0,100);
    popMatrix();
    
    //  Voilure  et derive en vert
    fill(0, 255, 0, 200);

    beginShape(TRIANGLES);
    vertex(-30,  100, 2) ; vertex(80, 0,  2) ; vertex(-30, -100, 2);   // Voilure sup
    vertex(-30,  100,-2) ; vertex(80, 0, -2) ; vertex(-30, -100,-2);   // Voilure inf
    vertex(-100, -2, 0)    ; vertex(-110, -2,-45); vertex(-45, -2, 0);    // Derive coté gauche
    vertex(-100,  2, 0)    ; vertex(-110,  2,-45); vertex(-45,  2, 0);    // Derive coté droit
    endShape();

    beginShape(QUADS);
    vertex(-30, 100, 2); vertex(-30, 100, -2); vertex( -30, -100, -2); vertex( -30,  -100, 2); //bord de fuite de voilure
    vertex( 80,  0, -2); vertex(-30, 100, -2); vertex( -30,  100,  2); vertex(  80,     0, 2); // bord d'attaque voilure gauche (sens Y)
    vertex( 80,  0, -2); vertex(-30,-100, -2); vertex( -30, -100,  2); vertex(  80,     0, 2); // bord d'attaque voilure droite 
    
    vertex(  -100,  -2, 0); vertex(  -100,  2,  0); vertex( -45, 2,  0); vertex( -45,-2, 0); //bordure de derive
    vertex(  -100,  -2, 0); vertex(  -110, -2,-45); vertex(-110, 2,-45); vertex(-100, 2, 0); // bord de fuite derive
    vertex(   -45,  -2, 0); vertex(  -110, -2,-45); vertex(-110, 2,-45); vertex( -45, 2, 0); // bord d'attaque derive
    
       vertex(   -100,  2, -5); vertex( -100, -50, -5); vertex(-80, -40, -5); vertex( -65,  2,-5);  // Empenage coté gauche
       vertex(   -100, -2, -5); vertex( -100,  50, -5); vertex(-80,  40, -5); vertex( -65, -2,-5); // Empenage coté droit

    endShape(); 
    popMatrix();
  }
  




    void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
}

  // Methode d'affichage

  void display() {
    pushMatrix();

    translate(pos.y, pos.x, pos.z);                                  // Attention on est en rep screen
    rotateZ(-r.z);rotateY(-r.x); rotateX(-r.y); 
    noStroke();
    draw_A_U_V(U); // On externalise le tracé de l'avion On passe les commandes pour dessiner la position des gouvernes...
    popMatrix();
  }
//  -- fin d'objet
}
