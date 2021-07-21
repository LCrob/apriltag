#ifndef TEST_C
#define TEST_C

    float deg2rad(float orientation);
    Eigen::Matrix3f Rx(float angle );
    Eigen::Matrix3f Ry(float angle );
    Eigen::Matrix3f Rz(float angle );  
    void fromRotationToRPYAngle(float& roll,float&  pitch, float& yaw,const Eigen::Matrix3f R);

#endif /* TEST_H */
