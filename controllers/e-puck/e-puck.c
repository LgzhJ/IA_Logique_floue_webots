/*
 * Copyright 1996-2024 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>

#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

/* 1 : longe le mur à droite (ps1, ps2, ps3, ps6), 0 : longe le mur à gauche (ps1, ps4, ps5, ps6) */
const int MUR_A_DROITE = 1;


typedef struct {
  double a, b, c, d;
} Trapeze;

/* Entrée 1 : distance frontale = max(ps1, ps6)*/
/*
Trapeze avant_loin = {0.0, 0.0, 150.0, 210.0};
Trapeze avant_proche = {220.0, 280.0, 4095.0, 4095.0};
*/
Trapeze avant_loin = {0.0, 0.0, 90.0, 160.0};
Trapeze avant_proche = {160.0, 200.0, 4095.0, 4095.0};

/* Entrée 2 : distance latérale au mur suivi (ps2 ou ps5)*/
/*
Trapeze cote_loin = {0.0, 0.0, 150.0, 190.0};
Trapeze cote_moyen = {180.0, 280.0, 380.0, 460.0};
Trapeze cote_proche = {380.0, 580.0, 4095.0, 4095.0};
*/
Trapeze cote_loin = {0.0, 0.0, 90.0, 130.0};
Trapeze cote_moyen = {120.0, 200.0, 300.0, 380.0};
Trapeze cote_proche = {300.0, 500.0, 4095.0, 4095.0};

/*
Trapeze cote_loin = {0.0, 0.0, 90.0, 160.0};
Trapeze cote_moyen = {90.0, 160.0, 300.0, 380.0};
Trapeze cote_proche = {300.0, 380.0, 4095.0, 4095.0};*/

/* Entrée 3 : orientation du robot par rapport au mur
 * diff_avant_arriere = max(ps1, ps6) - (ps3 ou ps4)
 *  diff_avant_arriere > 0 : l'avant du robot pointe VERS le mur 
 *  diff_avant_arriere < 0 : l'avant du robot s'ÉLOIGNE du mur
 *  diff_avant_arriere ~= 0 : robot parallèle au mur ou aucun mur autour
 **/
Trapeze angle_eloigne = {-4095.0, -4095.0, -90.0, -30.0};
Trapeze angle_nul = {-90.0, -30.0, 30.0, 80.0};
Trapeze angle_vers_mur = {30.0, 80.0, 4095.0, 4095.0};

/* Sorties floues (vitesses en m/s)*/
Trapeze recul = {-0.1, -0.1, -0.05, 0.0};
Trapeze lent = {0.0, 0.0, 0.03, 0.05};
Trapeze moderee = {0.05, 0.06, 0.08, 0.09};
Trapeze rapide = {0.09, 0.12, 0.15, 0.15};

/**
 * Calcule le degré d'appartenance (mu(x)) dans le cas où x est sur l'une des deux pentes d'un trapèze
 * à l'aide de la fonction affine f(x)=ax+b
 */
double f_fct_affine(Trapeze t, double x) {
  double a, b;

  /* Pente montante : entre t.a et t.b */
  if (x >= t.a && x < t.b) {
    a = 1.0 / (t.b - t.a);
    b = -a * t.a;

    return a * x + b;
  }

  /* Pente descendante : entre t.c et t.d */
  if (x > t.c && x <= t.d) {
    a = -1.0 / (t.d - t.c);
    b = 1.0 - a * t.c;

    return a * x + b;
  }

  return 0.0;
}

/* Calcule le degré d'appartenance mu de x*/
double f_calcul_mu(Trapeze t, double dist) {
  /* En dehors du trapèze */
  if (dist < t.a || dist > t.d) return 0.0;

  /* Sur le plateau */
  if (dist >= t.b && dist <= t.c) return 1.0;

  /* Pente montante */
  if (dist > t.a && dist < t.b) return f_fct_affine(t, dist);

  /* Pente descendante */
  if (dist > t.c && dist < t.d) return f_fct_affine(t, dist);

  return 0.0;
}

double f_defuzzification(double mu_recul, double mu_lent, double mu_moderee, double mu_rapide) {
  double v_recul = -0.05;
  double v_lent = 0.03;
  double v_moderee = 0.07;
  double v_rapide = 0.12;
  double num = mu_recul * v_recul
             + mu_lent * v_lent
             + mu_moderee * v_moderee
             + mu_rapide * v_rapide;
  double den = mu_recul + mu_lent + mu_moderee + mu_rapide;

  if (den == 0.0) return 0.0;
  return num / den;
}

/**
* R1 : Si avant proche                                                    ALORS pivoter à gauche  (G recule, D lent)
* R2 : Si avant loin ET côté proche ET NON(diff_avant_arriere s'éloigne)  ALORS s'écarter du mur  (G modérée, D rapide)
* R3 : Si avant loin ET côté moyen ET diff_avant_arriere parallèle        ALORS tout droit        (G modérée, D modérée)
* R4 : Si avant loin ET côté loin ET NON(diff_avant_arriere vers le mur)  ALORS se rapprocher     (G rapide,  D modérée)
* R5 : Si avant loin ET côté moyen ET diff_avant_arriere vers le mur      ALORS léger à gauche    (G modérée, D rapide)
* R6 : Si avant loin ET côté moyen ET diff_avant_arriere s'éloigne        ALORS léger à droite    (G rapide,  D modérée)
* R7 : Si avant loin ET côté proche ET diff_avant_arriere s'éloigne       ALORS tout droit        
* R8 : Si avant loin ET côté loin ET diff_avant_arriere vers le mur       ALORS tout droit        
* R9 : Si avant loin ET côté loin ET diff_avant_arriere parallèle         ALORS virage à droite   (G lent, D recul)
*/
void f_moteur_inference(double d_avant, double d_cote, double diff_avant_arriere, double *v_gauche, double *v_droite) {
  double mu_av_loin = f_calcul_mu(avant_loin, d_avant);
  double mu_av_proche = f_calcul_mu(avant_proche, d_avant);

  double mu_c_loin = f_calcul_mu(cote_loin, d_cote);
  double mu_c_moyen = f_calcul_mu(cote_moyen, d_cote);
  double mu_c_proche = f_calcul_mu(cote_proche, d_cote);

  double mu_a_eloigne = f_calcul_mu(angle_eloigne, diff_avant_arriere);
  double mu_a_parallele = f_calcul_mu(angle_nul, diff_avant_arriere);
  double mu_a_vers_mur = f_calcul_mu(angle_vers_mur, diff_avant_arriere);

  /* Activation des règles */
  double r1 = mu_av_proche;
  double r2 = fmin(mu_av_loin, fmin(mu_c_proche, 1.0 - mu_a_eloigne));
  double r3 = fmin(mu_av_loin, fmin(mu_c_moyen, mu_a_parallele));
  double r4 = fmin(mu_av_loin, fmin(mu_c_loin, 1.0 - mu_a_vers_mur));
  double r5 = fmin(mu_av_loin, fmin(mu_c_moyen, mu_a_vers_mur));
  double r6 = fmin(mu_av_loin, fmin(mu_c_moyen, mu_a_eloigne));
  double r7 = fmin(mu_av_loin, fmin(mu_c_proche, mu_a_eloigne));
  double r8 = fmin(mu_av_loin, fmin(mu_c_loin, mu_a_vers_mur));
  double r9 = fmin(mu_av_loin, fmin(mu_c_loin, mu_a_parallele));

  /* Roue gauche */
  double g_recul = r1;
  double g_lent = r9;
  double g_moderee = fmax(fmax(r2, r3), fmax(r5, fmax(r7, r8)));
  //double g_moderee = fmax(fmax(r2, r3), r5);
  double g_rapide = fmax(r4, r6);

  /* Roue droite */
  double d_recul = r9;
  double d_lent = r1;
  double d_moderee = fmax(fmax(r3, r4), fmax(r6, fmax(r7, r8)));
  //double d_moderee = fmax(fmax(r3, r4), r6);
  double d_rapide = fmax(r2, r5);

  *v_gauche = f_defuzzification(g_recul, g_lent, g_moderee, g_rapide);
  *v_droite = f_defuzzification(d_recul, d_lent, d_moderee, d_rapide);
}

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i;
  double speed[2];
  double sensors_value[8];
  int time_step;
  int nb_step = 0;
  int camera_time_step;
  /* initialize Webots */
  wb_robot_init();

  if (strcmp(wb_robot_get_model(), "GCtronic e-puck2") == 0) {
    printf("e-puck2 robot\n");
    time_step = 64;
    camera_time_step = 64;
  } else {  // original e-puck
    printf("e-puck robot\n");
    time_step = 256;
    camera_time_step = 1024;
  }

  /* get and enable the camera and accelerometer */
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, camera_time_step);
  WbDeviceTag accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step);

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* get a handler to the position sensors and enable them. */
  left_position_sensor = wb_robot_get_device("left wheel sensor");
  right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, time_step);
  wb_position_sensor_enable(right_position_sensor, time_step);

  for (i = 0; i < 8; i++) {
    char device_name[4];
    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], time_step);
  }
  
   /* Affiche la vitesse max du robot*/
  double right_max_w = wb_motor_get_max_velocity(right_motor);  // rad/s
  printf("max right wheel speed: %g rad/s -> %g m/s\n", right_max_w, right_max_w * WHEEL_RADIUS);
  double left_max_w = wb_motor_get_max_velocity(left_motor);  // rad/s
  printf("max left wheel speed: %g rad/s -> %g m/s\n", left_max_w, left_max_w * WHEEL_RADIUS);

  int compteur_sans_mur = 0; 
  const int k = 50;
  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }

    /* Entrées frontales*/
    double d_avant = fmax(sensors_value[1], sensors_value[6]);
    double v_g, v_d;

    /* Aucun mur détecté autour du robot*/
    int pas_de_mur_droite = (sensors_value[2] > 50 && sensors_value[2] < 80);
    int pas_de_mur_gauche = (sensors_value[5] > 50 && sensors_value[5] < 80);

    if(pas_de_mur_droite && pas_de_mur_gauche) {
      compteur_sans_mur += 1;
    }else {
      compteur_sans_mur = 0;
    }

    double d_cote, diff_avant_arriere;
    if(MUR_A_DROITE) {
      d_cote = sensors_value[2];
      diff_avant_arriere = sensors_value[1] - sensors_value[3];
    }else {
      d_cote = sensors_value[5];
      diff_avant_arriere = sensors_value[6] - sensors_value[4];
    }

    /* Si le robot tourne en rond depuis k ou plus pas de temps alors avancer tout droit*/
    if (compteur_sans_mur >= k) {
      v_g = 0.15;
      v_d = 0.15;
    } else {
      f_moteur_inference(d_avant, d_cote, diff_avant_arriere, &v_g, &v_d);

      if(!MUR_A_DROITE) {
        /* Mur à gauche = image miroir : on échange les deux roues*/
        double tmp = v_g;
        v_g = v_d;
        v_d = tmp;
      }
    }

    speed[0] = v_g / WHEEL_RADIUS; /* m/s -> rad/s */
    speed[1] = v_d / WHEEL_RADIUS;

    nb_step += 1;
    if(MUR_A_DROITE) {
      printf("Temps: %d | ps1=%.0f ps6=%0.f ps3=%.0f cote=%.0f diff_avant_arriere=%.0f | vg=%.3f vd=%.3f m/s\n", nb_step, sensors_value[1], sensors_value[6],
             sensors_value[3], d_cote, diff_avant_arriere, v_g, v_d);
    }else {
      printf("Temps: %d | ps6=%0.f ps1=%.0f ps4=%.0f cote=%.0f diff_avant_arriere=%.0f | vg=%.3f vd=%.3f m/s\n", nb_step, sensors_value[6], sensors_value[1],
             sensors_value[4], d_cote, diff_avant_arriere, v_g, v_d);
    }

    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    wb_motor_set_velocity(right_motor, speed[1]);
  }

  wb_robot_cleanup();

  return 0;
}