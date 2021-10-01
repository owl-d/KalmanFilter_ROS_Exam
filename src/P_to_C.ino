#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <IMU.h>


ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("kalman", &imu_msg);


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

cIMU imu;

void matrix_dot(double* C, double* A, double* B, int n, int l, int m){
  for(int i = 0; i < n; i++){
    for(int j = 0; j < m; j++){
      *(C+i*m+j) = 0;
      for(int k = 0; k < l; k++)
        *(C+i*m+j) += *(A + i*l + k) * (*(B + j + m*k));
    }
  }
}

int InvMatrix(int n, const double* A, double* b){
  double m;
  register int i, j, k;
  double* a = new double[n*n];
  
  if(a==NULL)
    return 0;
  for(i=0; i<n*n; i++)
    a[i]=A[i];
  for(i=0; i<n; i++){
    for(j=0; j<n; j++){
    b[i*n+j]=(i==j)?1.:0.;
    }
  }
  for(i=0; i<n; i++){
    if(a[i*n+i]==0.)
    {
    if(i==n-1)
    {
      delete[] a;
      return 0;
    }
    for(k=1; i+k<n; k++)
    {
      if(a[i*n+i+k] != 0.)
      break;
    }
    
    if(i+k>=n)
    {
      delete[] a;
      return 0;
    }
    
    for(j=0; j<n; j++)
    {
      m = a[i*n+j];
      a[i*n+j] = a[(i+k)*n+j];
      a[(i+k)*n+j] = m;
      m = b[i*n+j];
      b[i*n+j] = b[(i+k)*n+j];
      b[(i+k)*n+j] = m;
    }
    }
    m = a[i*n+i];
    for(j=0; j<n; j++)
    {
      a[i*n+j]/=m;
      b[i*n+j]/=m;
    }
    for(j=0; j<n; j++)
    {
    if(i==j)
    continue;
    m = a[j*n+i];
    for(k=0; k<n; k++)
    {
      a[j*n+k] -= a[i*n+k]*m;
      b[j*n+k] -= b[i*n+k]*m;
    }
    }
    }
    delete[] a;
    return 1;
}


void create_A(double* gyro, double* A){
  A[0] = 1;
  A[1] = -0.02/2*gyro[0];
  A[2] = -0.02/2*gyro[1];
  A[3] = -0.02/2*gyro[2];
  A[4] = 0.02/2*gyro[0];
  A[5] = 1;
  A[6] = 0.02/2*gyro[2];
  A[7] = -0.02/2*gyro[1];
  A[8] = 0.02/2*gyro[1];
  A[9] = -0.02/2*gyro[2];
  A[10] = 1;
  A[11] = 0.02/2*gyro[0];
  A[12] = 0.02/2*gyro[2];
  A[13] = 0.02/2*gyro[1];
  A[14] = -0.02/2*gyro[0];
  A[15] = 1;
}


void setup()
{

  Serial.begin(11520);
  
  nh.initNode();
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  imu.begin();
}

void loop()
{
  static uint32_t pre_time;

  imu.update();
  double gyro[3];
  double acc[3];
  double mag[3];
  double quat[4];
  double H[12];
  double Q[16] = {1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6, 0, 0, 0, 0, 1e-6};
  double R[9] = {2, 0, 0, 0, 2, 0, 0, 0, 2};
  double V[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  double P[16] = {0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001};
  double A[16];
  double K[16];
  double x[4] = {1, 0, 0, 0};
  double xp[4];
  double Pp[16];
  double z1[3];
  double z2[3];
  double h[3];

  if (millis()-pre_time >= 50)
  {
    pre_time = millis();

    imu_msg.header.stamp    = nh.now();
    imu_pub.publish(&imu_msg);
    imu_msg.header.frame_id = "imu_link";

//    imu_msg.angular_velocity_covariance[0] = 0.02;
//    imu_msg.angular_velocity_covariance[1] = 0;
//    imu_msg.angular_velocity_covariance[2] = 0;
//    imu_msg.angular_velocity_covariance[3] = 0;
//    imu_msg.angular_velocity_covariance[4] = 0.02;
//    imu_msg.angular_velocity_covariance[5] = 0;
//    imu_msg.angular_velocity_covariance[6] = 0;
//    imu_msg.angular_velocity_covariance[7] = 0;
//    imu_msg.angular_velocity_covariance[8] = 0.02;
//
//    imu_msg.linear_acceleration_covariance[0] = 0.04;
//    imu_msg.linear_acceleration_covariance[1] = 0;
//    imu_msg.linear_acceleration_covariance[2] = 0;
//    imu_msg.linear_acceleration_covariance[3] = 0;
//    imu_msg.linear_acceleration_covariance[4] = 0.04;
//    imu_msg.linear_acceleration_covariance[5] = 0;
//    imu_msg.linear_acceleration_covariance[6] = 0;
//    imu_msg.linear_acceleration_covariance[7] = 0;
//    imu_msg.linear_acceleration_covariance[8] = 0.04;
//
//    imu_msg.orientation_covariance[0] = 0.0025;
//    imu_msg.orientation_covariance[1] = 0;
//    imu_msg.orientation_covariance[2] = 0;
//    imu_msg.orientation_covariance[3] = 0;
//    imu_msg.orientation_covariance[4] = 0.0025;
//    imu_msg.orientation_covariance[5] = 0;
//    imu_msg.orientation_covariance[6] = 0;
//    imu_msg.orientation_covariance[7] = 0;
//    imu_msg.orientation_covariance[8] = 0.0025;


    //sensor value
    gyro[0] = imu.gyroData[0]/180*3.14/16.4;
    gyro[1] = imu.gyroData[1]/180*3.14/16.4;
    gyro[2] = imu.gyroData[2]/180*3.14/16.4;
    Serial.print("gyro :");
    Serial.print(gyro[0]);
    Serial.print(" ");
    Serial.print(gyro[1]);
    Serial.print(" ");
    Serial.println(gyro[2]);

    acc[0] = imu.accData[0]/16384*9.8;
    acc[1] = imu.accData[1]/16384*9.8;
    acc[2] = imu.accData[2]/16384*9.8;
    Serial.print("acc :");
    Serial.print(acc[0]);
    Serial.print(" ");
    Serial.print(acc[1]);
    Serial.print(" ");
    Serial.println(acc[2]);

    mag[0] = imu.magData[0];
    mag[1] = imu.magData[1];
    mag[2] = imu.magData[2];
    Serial.print("mag :");
    Serial.print(mag[0]);
    Serial.print(" ");
    Serial.print(mag[1]);
    Serial.print(" ");
    Serial.println(mag[2]);


    //Priori System Estimate : gyro
    create_A(gyro, A);
    matrix_dot(xp, A, x, 4, 4, 1);
    double Pp_a[16];
    matrix_dot(Pp_a, A, P, 4, 4, 4);
    double A_T[16] = {A[0], A[4], A[8], A[12], A[1], A[5], A[9], A[13], A[2], A[6], A[10], A[14], A[3], A[7], A[11], A[15]};
    double Pp_A[16];
    matrix_dot(Pp_A, Pp_a, A_T, 4, 4, 4);
    double Pp[16] = {Pp_A[0]+Q[0], Pp_A[1]+Q[1], Pp_A[2]+Q[2], Pp_A[3]+Q[3], Pp_A[4]+Q[4], Pp_A[5]+Q[5], Pp_A[6]+Q[6], Pp_A[7]+Q[7], Pp_A[8]+Q[8], Pp_A[9]+Q[9], Pp_A[10]+Q[10], Pp_A[11]+Q[11], Pp_A[12]+Q[12], Pp_A[13]+Q[13], Pp_A[14]+Q[14], Pp_A[15]+Q[15]};

    //Correction Stage1 : with acc
    H[0] = -2*x[2];
    H[1] = 2*x[3];
    H[2] = -2*x[0];
    H[3] = 2*x[1];
    H[4] = 2*x[1];
    H[5] = 2*x[0];
    H[6] = 2*x[3];
    H[7] = 2*x[2];
    H[8] = 2*x[0];
    H[9] = -2*x[1];
    H[10] = -2*x[2];
    H[11] = 2*x[3];
    
    h[0] = 2*x[1]*x[3] - 2*x[0]*x[2];
    h[1] = 2*x[0]*x[1] + 2*x[2]*x[3];
    h[2] = x[0]*2*2 - x[1]*2*2 - x[2]*2*2 + x[3]*2*2;
    h[0] = h[0]*9.8;
    h[1] = h[1]*9.8;
    h[2] = h[2]*9.8;
    
    z1[0] = acc[0];
    z1[1] = acc[1];
    z1[2] = acc[2];
    
    double H_T[12]={ H[0], H[4], H[8], H[1], H[5], H[9], H[2], H[6], H[10], H[3], H[7], H[11]};
    double Pp_h[12];
    matrix_dot(Pp_h, H, Pp, 3, 4, 4);
    double Pp_H[9];
    matrix_dot(Pp_H, Pp_h, H_T, 3, 4, 3);

    double V_T[9]={V[0], V[3], V[6], V[1], V[4], V[7], V[2], V[5], V[8]};
    double R_v[9];
    matrix_dot(R_v, V, R, 3, 3, 3);
    double R_V[9];
    matrix_dot(R_V, R_v, V_T, 3, 3, 3);
    
    double temp[16] = {Pp_H[0]+R_V[0], Pp_H[1]+R_V[1], Pp_H[2]+R_V[2], Pp_H[3], Pp_H[4]+R_V[3], Pp_H[5]+R_V[4], Pp_H[6]+R_V[5], Pp_H[7], Pp_H[8]+R_V[6], Pp_H[9]+R_V[7], Pp_H[10]+R_V[8], Pp_H[11], Pp_H[12], Pp_H[13], Pp_H[14], Pp_H[15]};
    
    double inverse_temp[16];
    InvMatrix(4,(double*)temp, (double*)inverse_temp);


    double H_ti[16];
    matrix_dot(H_ti, Pp, H_T, 4, 4, 4);
    matrix_dot(K, H_ti, inverse_temp, 4, 4, 4);
    
    double z1_h[3] = {z1[0]-h[0], z1[1]-h[1], z1[2]-h[2]};
    double qe1[4];
    matrix_dot(qe1, K, z1_h, 4, 4, 1);
    qe1[3] = 0;
    double q1[16] ={ xp[0] + qe1[0], xp[1] + qe1[1], xp[2] + qe1[2], A[3] + qe1[3], xp[4] , xp[5], xp[6], xp[7], xp[8] , xp[9], xp[10], xp[11], xp[12], xp[13], xp[14], xp[15]};


    double K_h[16];
    matrix_dot(K_h, K, H, 4, 4, 4);
    double I_Kh[16] = { 1 - K_h[0], -K_h[1], -K_h[2], -K_h[3], -K_h[4], 1 - K_h[5], -K_h[6], -K_h[7], -K_h[8], -K_h[9], 1 - K_h[10], -K_h[11], -K_h[12], -K_h[13], -K_h[14], 1 - K_h[15]};
    double P1[16];
    matrix_dot(P1, Pp, I_Kh, 4, 4, 4);

    
    //Correction Stage2 : with mag
    H[0] = 2*x[3];
    H[1] = 2*x[2];
    H[2] = 2*x[1];
    H[3] = 2*x[0];
    H[4] = 2*x[0];
    H[5] = -2*x[1];
    H[6] = -2*x[2];
    H[7] = -2*x[3];
    H[8] = -2*x[1];
    H[9] = -2*x[0];
    H[10] = 2*x[3];
    H[11] = 2*x[2];
        
    h[0] = 2*x[1]*x[2] + 2*x[0]*x[3];
    h[1] = x[0]*2*2 - x[1]*2*2 - x[2]*2*2 - x[3]*2*2;
    h[2] = 2*x[2]*x[3] - 2*x[0]*x[1];
    
    z2[0] = mag[0];
    z2[1] = mag[1];
    z2[2] = mag[2];

    double R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    matrix_dot(Pp_h, H, Pp, 3, 4, 4);
    matrix_dot(Pp_H, Pp_h, H_T, 3, 4, 3);
    matrix_dot(R_v, V, R, 3, 3, 3);
    matrix_dot(R_V, R_v, V_T, 3, 3, 3);

    temp[0] = Pp_H[0]+R_V[0];
    temp[1] = Pp_H[1]+R_V[1];
    temp[2] = Pp_H[2]+R_V[2];
    temp[3] = Pp_H[3];
    temp[4] = Pp_H[4]+R_V[3];
    temp[5] = Pp_H[5]+R_V[4];
    temp[6] = Pp_H[6]+R_V[5];
    temp[7] = Pp_H[7];
    temp[8] = Pp_H[8]+R_V[6];
    temp[9] = Pp_H[9]+R_V[7];
    temp[10] = Pp_H[10]+R_V[8];
    temp[11] = Pp_H[11];
    temp[12] = Pp_H[12];
    temp[13] = Pp_H[13];
    temp[14] = Pp_H[14];
    temp[15] = Pp_H[15];
    
    InvMatrix(4,temp,inverse_temp);
    matrix_dot(H_ti, Pp, H_T, 4, 4, 4);
    matrix_dot(K, H_ti, inverse_temp, 4, 4, 4);
    
    double z2_h[3] = {z1[0]-h[0], z1[1]-h[1], z1[2]-h[2]};
    double qe2[4];
    matrix_dot(qe2, K, z1_h, 4, 4, 1);
    qe2[1] = 0;
    qe2[2] = 0;
    
    x[0] = q1[0] + qe1[0];
    x[1] = q1[1] + qe2[1];
    x[2] = q1[2] + qe2[2];
    x[3] = q1[3] + qe2[3];
    
    matrix_dot(K_h, K, H, 4, 4, 4);
    I_Kh[0] = 1 - K_h[0];
    I_Kh[1] = -K_h[1];
    I_Kh[2] = -K_h[2];
    I_Kh[3] = -K_h[3];
    I_Kh[4] = -K_h[4];
    I_Kh[5] = 1 - K_h[5];
    I_Kh[6] = -K_h[6];
    I_Kh[7] = -K_h[7];
    I_Kh[8] = -K_h[8];
    I_Kh[9] = -K_h[9];
    I_Kh[10] = 1 - K_h[10];
    I_Kh[11] = -K_h[11];
    I_Kh[12] = -K_h[12];
    I_Kh[13] = -K_h[13];
    I_Kh[14] = -K_h[14];
    I_Kh[15] = 1 - K_h[15];
  
    matrix_dot(P, P1, I_Kh, 4, 4, 4);

    double Q_abs = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    quat[0] = x[0]/Q_abs;
    quat[1] = x[1]/Q_abs;
    quat[2] = x[2]/Q_abs;
    quat[3] = x[3]/Q_abs;

    

    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.rotation.w = quat[0];
    tfs_msg.transform.rotation.x = quat[1];
    tfs_msg.transform.rotation.y = quat[2];
    tfs_msg.transform.rotation.z = quat[3];

    Serial.print("quaternion : ");
    Serial.print(quat[0]);
    Serial.print(" ");
    Serial.print(quat[1]);
    Serial.print(" ");
    Serial.print(quat[2]);
    Serial.print(" ");
    Serial.println(quat[3]);

    imu_msg.orientation.w = quat[3];
    imu_msg.orientation.x = quat[0];
    imu_msg.orientation.y = quat[1];
    imu_msg.orientation.z = quat[2];

    imu_pub.publish(&imu_msg);
    

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);
  }

  nh.spinOnce();
}
