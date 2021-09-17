#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <IMU.h>


ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);


geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;

cIMU imu;

void matrix_dot(int* C, int* A, int* B, int n){
for(int i = 0; i < n; i++){
for(int j = 0; j < n; j++){
*(C+i*n+j) = *(A + i*n + 0 ) * (*(B + j + 0)) + *(A + i*n + 1 ) * (*(B + j + 3)) + *(A + i*n + 2 ) * (*(B + j + 6));
}
}

// *(C) = (*(A)) * (*B) + *(A + 1) * (*(B + 3)) + *(A + 2) * (*(B + 6));
// *(C + 1) = (*(A)) * (*(B + 1)) + *(A + 1) * (*(B + 4)) + *(A + 2) * (*(B + 7));
// *(C + 2) = (*(A)) * (*(B + 2)) + *(A + 1) * (*(B + 5)) + *(A + 2) * (*(B + 8));
// *(C + 3) = *(A + 3) * (*B) + *(A + 4) * (*(B + 3)) + *(A + 5) * (*(B + 6));
// *(C + 4) = *(A + 3) * (*(B + 1)) + *(A + 4) * (*(B + 4)) + *(A + 5) * (*(B + 7));
// *(C + 5) = *(A + 3) * (*(B + 2)) + *(A + 4) * (*(B + 5)) + *(A + 5) * (*(B + 8));
// *(C + 6) = *(A + 6) * (*B) + *(A + 7) * (*(B + 3)) + *(A + 8) * (*(B + 6));
// *(C + 7) = *(A + 6) * (*(B + 1)) + *(A + 7) * (*(B + 4)) + *(A + 8) * (*(B + 7));
// *(C + 8) = *(A + 6) * (*(B + 2)) + *(A + 7) * (*(B + 5)) + *(A + 8) * (*(B + 8));

printf("1\n");
for(int i = 0; i < 9; i++){
printf("%d\t", * (C+i));
if ( (i+1) % 3 == 0)
printf("\n");
}
printf("\n");

}

void setup()
{

  Serial.begin(11520);
  
  nh.initNode();
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  double gyro[3][1]
  double acc[3][1]
  double mag[3][1]
  double quat[4][1]

  imu.begin();
}

void loop()
{
  static uint32_t pre_time;

  imu.update();

  if (millis()-pre_time >= 50)
  {
    pre_time = millis();

    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    gyro[1][0] = imu.gyroData[0]/16.4;
    gyro[2][0] = imu.gyroData[1]/16.4;
    gyro[3][0] = imu.gyroData[2]/16.4;

    acc[1][0] = imu.accData[0]/16384*9.8;
    acc[2][0] = imu.accData[1]/16384*9.8;
    acc[3][0] = imu.accData[2]/16384*9.8;

    mag[1][0] = imu.magData[0];
    mag[2][0] = imu.magData[1];
    mag[3][0] = imu.magData[2];


    #calculate


    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.rotation.w = quat[0];
    tfs_msg.transform.rotation.x = quat[1];
    tfs_msg.transform.rotation.y = quat[2];
    tfs_msg.transform.rotation.z = quat[3];

    Serial.print(imu.accData[0]);
    Serial.print(imu.accData[1]);
    Serial.println(imu.accData[2]);
    

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);
  }

  nh.spinOnce();
}
