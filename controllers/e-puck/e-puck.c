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

typedef struct {
  double a, b, c, d;
}Trapeze;

/* Distance*/
Trapeze loin = {0.0, 0.0, 100.0, 300.0};
Trapeze moyen = {200.0, 300.0, 300.0, 400.0};
Trapeze proche = {300.0, 500.0, 4095.0, 4095.0};

/* Orientation*/
Trapeze gauche = {-90.0, -90.0, -45.0, 0.0};
Trapeze tout_droit = {-45.0, 0.0, 0.0, 45.0};
Trapeze droite = {0.0, 45.0, 90.0, 90.0};

/* Vitesse*/
Trapeze lent = {0.0, 0.0, 0.05, 0.075};
Trapeze moderee = {0.0625, 0.075, 0.075, 0.0875};
Trapeze rapide = {0.075, 0.1, 0.15, 0.15};

/**
* Calcule le degré d'appartenance (mu(x)) sur l'une des deux pentes d'un trapèze
* à l'aide de la fonction affine f(x)=ax+b
*/
double f_fct_affine(Trapeze t, double x) {
  double a, b;

  /* Pente montante : entre t.a et t.b*/
  if (x >= t.a && x < t.b) {
    a = 1.0 / (t.b - t.a);
    b = -a * t.a;

    return a * x + b;
  }

  /* Pente descendante : entre t.c et t.d*/
  if (x > t.c && x <= t.d) {
    a = -1.0 / (t.d - t.c);
    b = 1.0 - a * t.c;

    return a * x + b;
  }

  return 0.0;
}

double f_calcul_mu(Trapeze t, double dist) {
  /* En dehors du trapèze*/
  if(dist < t.a || dist > t.d) return 0.0;
  
  /* Sur le plateau*/
  if(dist >= t.b && dist <= t.c) return 1.0;
  
  /* Pente montante*/
  if(dist > t.a && dist < t.b) return f_fct_affine(t, dist);

  /* Pente descendante*/
  if(dist > t.c && dist < t.d) return f_fct_affine(t, dist);

  return 0.0;
}

double f_max_sensors_value(double *tab, int i, int j) {
  double max=0.0;
  for(int k=i; k<=j; k++){
    if(tab[k]>max) {
      max=tab[k];
    }
  }
  return max;
}

double f_defuzzification(double mu_lent, double mu_moderee, double mu_rapide) {
  double v_lent = 0.05;
  double v_moderee = 0.075;
  double v_rapide = 0.1;
  double num = + mu_lent * v_lent
             + mu_moderee * v_moderee
             + mu_rapide * v_rapide;
  double den = mu_lent + mu_moderee + mu_rapide;

  if (den == 0.0) return 0.0;
  return num / den;
}

void f_moteur_inference(double dg, double dd, double *v_gauche, double *v_droite) {
  double mu_loin_dg = f_calcul_mu(loin, dg);
  double mu_moyen_dg = f_calcul_mu(moyen, dg);
  double mu_proche_dg = f_calcul_mu(proche, dg);

  double mu_loin_dd = f_calcul_mu(loin, dd);
  double mu_moyen_dd = f_calcul_mu(moyen, dd);
  double mu_proche_dd = f_calcul_mu(proche, dd);

  /* Activation des 5 règles (NON = 1 - mu, OU = max, ET = min) */
  double r1 = fmin(mu_proche_dg, 1.0 - mu_proche_dd);
  double r2 = fmin(mu_proche_dd, 1.0 - mu_proche_dg);
  double r3 = fmin(mu_loin_dg,   mu_loin_dd);
  double r4 = fmax(mu_moyen_dg,  mu_moyen_dd);
  double r5 = fmin(mu_proche_dg, mu_proche_dd);

  double r6 = fmin(mu_moyen_dg, mu_loin_dd);
  double r7 = fmin(mu_loin_dg, mu_moyen_dd);

  /* Roue gauche */
  double g_lent = r2;
  double g_moderee = fmax(r4, r5);
  double g_rapide = fmax(r1, r3);

  /* Roue droite */
  double d_lent = fmax(r1, r5);
  double d_moderee = r4;
  double d_rapide = fmax(r2, r3);

  *v_gauche = f_defuzzification(g_lent, g_moderee, g_rapide);
  *v_droite = f_defuzzification(d_lent, d_moderee, d_rapide);
}

static void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor) {
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);
  double dl = l * WHEEL_RADIUS;         // distance covered by left wheel in meter
  double dr = r * WHEEL_RADIUS;         // distance covered by right wheel in meter
  double da = (dr - dl) / AXLE_LENGTH;  // delta orientation
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);
}

int main(int argc, char *argv[]) {
  /* define variables */
  WbDeviceTag distance_sensor[8], left_motor, right_motor, left_position_sensor, right_position_sensor;
  int i, j;
  double speed[3];
  double sensors_value[8];
  double braitenberg_coefficients[8][2] = {{0.942, -0.22}, {0.63, -0.1}, {0.5, -0.06},  {-0.06, -0.06},
                                           {-0.06, -0.06}, {-0.06, 0.5}, {-0.19, 0.63}, {-0.13, 0.942}};
  int time_step; 
  int nb_step=0;
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
  

  /* main loop */
  while (wb_robot_step(time_step) != -1) {
    /* get sensors values */
    for (i = 0; i < 8; i++) {
      sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
    }
    double max_capteurs_gauches = f_max_sensors_value(sensors_value, 1, 2);
    double max_capteurs_droites = f_max_sensors_value(sensors_value, 5, 6);

    /* compute odometry and speed values*/
    //compute_odometry(left_position_sensor, right_position_sensor);

    double v_g, v_d;
    f_moteur_inference(max_capteurs_droites, max_capteurs_gauches, &v_g, &v_d);
    speed[0] = v_g / WHEEL_RADIUS;   /* m/s -> rad/s */
    speed[1] = v_d / WHEEL_RADIUS;
    
    /*for (i = 0; i < 2; i++) {
      speed[i] = 0.0;
      for (j = 0; j < 8; j++) {
        speed[i] += braitenberg_coefficients[j][i] * (1.0 - (sensors_value[j] / RANGE));

        if()
      }
    }*/
    
    nb_step+=1;
    printf("Temps: %d \n", nb_step);
    /* set speed values */
    wb_motor_set_velocity(left_motor, speed[0]);
    printf("Current left wheel speed: %g rad/s -> %g m/s\n", speed[0], speed[0] * WHEEL_RADIUS);
    wb_motor_set_velocity(right_motor, speed[1]);
    printf("Current right wheel speed: %g rad/s -> %g m/s\n", speed[1], speed[1] * WHEEL_RADIUS);
  }

  wb_robot_cleanup();

  return 0;
}

