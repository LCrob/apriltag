
#include <stdio.h>
#include <math.h>
#include <Eigen/Core>



float deg2rad(float orientation){

   return orientation*M_PI /180.0;
}




Eigen::Matrix3f Rx(float angle)
{

  Eigen::Matrix3f R;
  R.setIdentity();
  R <<   1 ,     0  ,          0,
         0 , cos(angle) ,  -sin(angle) ,
         0 , sin(angle) ,  cos(angle)   ; 
return R;

}

Eigen::Matrix3f Ry(float angle)
{
  Eigen::Matrix3f R;
  R.setIdentity();

  R << cos(angle) ,   0   ,   sin(angle),
        0             , 1     ,  0 ,
        -sin(angle)  , 0 ,  cos(angle)   ;

return R;

}

Eigen::Matrix3f Rz(float angle)
{

  Eigen::Matrix3f R;
  R.setIdentity();

  R << cos(angle) ,   -sin(angle)   ,  0,
       sin(angle)     , cos(angle)    ,  0 ,
       0              , 0 ,          1   ; 

return R;
}



void fromRotationToRPYAngle(float& roll,float&  pitch, float& yaw,const Eigen::Matrix3f R)
{

  //yaw   = atan2(R(1,0), R(0,0)  );
  //pitch = atan2( -R(2,0)  , sqrt( pow((R(2,1)),2) + pow(R(2,2),2)) );
  //roll  = atan2( R(2,1) , R(2,2) );

  yaw=atan2(R(1,0),R(0,0));
  pitch=atan2( -R(2,0)   ,   sqrt(  pow(R(2,1),2)+ pow(R(2,2),2)    )  );
  roll=atan2(R(2,1),R(2,2));

}
