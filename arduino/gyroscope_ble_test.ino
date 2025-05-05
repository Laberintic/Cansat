
// https://docs.arduino.cc/libraries/arduino_bmi270_bmm150/
#include "Arduino_BMI270_BMM150.h"


float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.5f; // la ganancia del filtro, cuanto mayor mas agresiva la correcion.
// experimentalmente he determinado que asi da resultados decentes. TODO: optimixar beta.

// variables para los datos obtenidos directamente del giroscopio, accelerometro y magnetometro, respectivamente.
float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;

unsigned long ultima_actualizacion;


/* filtro de Madgwick para fusion de sensores, implementacion "casera".  */
// https://ahrs.readthedocs.io/en/stable/filters/madgwick.html
// implementacion original: https://github.com/xioTechnologies/Fusion
void filtroMadgwick(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    /*float mx, float my, float mz,*/ float dt) {
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return;
  ax /= norm;
  ay /= norm;
  az /= norm;
  
  // estimacion del vector gravedad a partir del quaternion estimado anteriormente:
  // (transformamos el cuaternion de la rotacion en un vector direccion)

  float vx = 2.0f * (q1*q3 - q0*q2);
  float vy = 2.0f * (q0*q1 + q2*q3);
  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // calculamos el error mediante el producto vectorial entre el vector calculado y el dado por el accelerometro.
  float ex = (ay * vz - az * vy);
  float ey = (az * vx - ax * vz);
  float ez = (ax * vy - ay * vx);

  // corregimos el vector estimado en base al error y proporcional al valor beta
  gx += beta * ex;
  gy += beta * ey;
  gz += beta * ez;

  // para poder trabajar con la velocidad angular dada por el gyroscopio y a la vez con la rotacion 
  // dada por el cuaternion estimado, realizaremos la siguiente operacion.
  // esta formula se deriva de lo siguiente:
  /*
    tratamos de obtener la tasa de variacion de la direccion, es decir la velocidad. 
    La obtendremos haciendo 0.5*q⊗w, siendo ⊗ la multiplicacion de cuaterniones.
    0.5 por la naturaleza de estos, en la que un giro se representa con angulos medios.
  */
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

   // volvemos a intagrar (respecto al diferencial de tiempo) para obtener la posicion dada la velocidad.
   q0 += qDot0 * dt;
   q1 += qDot1 * dt;
   q2 += qDot2 * dt;
   q3 += qDot3 * dt; 

   // acabaremos por normalizar el cuaternion
   norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
   q0 /= norm;
   q1 /= norm;
   q2 /= norm;
   q3 /= norm;

  // // realizamos el mismo proceso para aplicar la correcion del magnetometro (sobre todo para corregir yaw)
  // float hx = 2.0f * (q1 * q3 - q0 * q2);
  // float hy = 2.0f * (q0 * q1 + q2 * q3);
  // float hz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // // calculamos el error
  // float mxError = (my * hz - mz * hy);
  // float myError = (mz * hx - mx * hz);
  // float mzError = (mx * hy - my * hx);

  // // modificamos ligeramente la estimacion
  // gx += beta * mxError;
  // gy += beta * myError;
  // gz += beta * mzError;

  // // integramos para añadir la correccion
  // qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  // qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  // qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  // qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // // derivamos para obtener la direccion
  // q0 += qDot0 * dt;
  // q1 += qDot1 * dt;
  // q2 += qDot2 * dt;
  // q3 += qDot3 * dt;

  // // Y finalmente normalizamos
  // norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  // q0 /= norm;
  // q1 /= norm;
  // q2 /= norm;
  // q3 /= norm;
}


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  ultima_actualizacion = micros();
}


void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable() /*&& IMU.magneticFieldAvailable()*/) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    //IMU.readMagneticField(mx, my, mz);

    // pasamos los datos del giroscopio a rad/s, ya que son dados en grados/s
    gx = gx * PI / 180.0f;
    gy = gy * PI / 180.0f;
    gz = gz * PI / 180.0f;
  
    // obtenemos el tiempo que ha pasado desde la ultima actualizacion (el diferencial del tiempo en segundos)
    unsigned long ahora = micros();
    float dt = (ahora - ultima_actualizacion) / 1.0e6;
    ultima_actualizacion = ahora;

    // ejecutamos el filtro de Madgwick
    filtroMadgwick(gx, gy, gz, ax, ay, az, /*mx, my, mz,*/ dt);

    // finalmente transformaremos el cuaternion calculado a angulos de Trait-Bryan (o de Euler)
    // implemetacion en python traducida a c de: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    // se le añade la trasnformacion a angulos, la pagina lo deja en radianes.

    float roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / PI;
    float t2 = 2.0f*(q0*q2 - q3*q1);
    if (t2 > 1.0f) t2 = 1.0f; // se limita t2 dentro de [-1, 1] ya que si se supera (por error de calculo) daria error la funcion asin.
    if (t2 < -1.0f) t2 = -1.0f;
    float pitch = asin(t2) * 180.0f / PI;
    float yaw = atan2(2.0f * (q0 * q2 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / PI;

    // // debug prints
    // Serial.print("dt: "); Serial.print(dt, 6);
    // Serial.print(" Quaternion: "); Serial.print(q0); Serial.print(", ");
    // Serial.print(q1); Serial.print(", "); Serial.print(q2); Serial.print(", "); Serial.print(q3);
    // Serial.print(" Yaw: "); Serial.print(yaw);
    // Serial.print(" Pitch: "); Serial.print(pitch);
    // Serial.print(" Roll: "); Serial.println(roll);

    Serial.print(yaw); Serial.print(" ");
    Serial.print(pitch); Serial.print(" ");
    Serial.println(roll);


    //delay(5);
  }
}