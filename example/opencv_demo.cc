/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/



//./opencv_demo -f tag36h11 -p tcp://192.168.204.140:7500


#include <iostream>

#include "opencv2/opencv.hpp"
#include <fstream>
#include <json.hpp>
#include <Publisher.hpp>
#include <signal.h>
#include <getopt.h>
#include <thread>
#include <markerPoseData.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <mathUtils.h>

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "common/homography.h"
#include "apriltag_pose.h"
}

using namespace std;
using namespace cv;

Common::Publisher* aprilcodePublisher; //("tcp://*:8500");

std::string publisherAddress;
std::string fam;
bool publisherAddress_specified;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

//from rotation To Quaternion

/*
void fromRotationToQuaternion( Eigen::Quaternion<float>& q , apriltag_pose_t& a ) {


    Eigen::Vector3f traceVec;
    traceVec <<a.R->data[0] , a.R->data[4] , a.R->data[8];

    if( traceVec.sum() > 0 ) {
        double s = 0.5f / sqrtf(traceVec.sum()+ 1.0f);
        q.w() = 0.25f / s;
        q.x() = ( a.R->data[7] - a.R->data[5] ) * s;
        q.y() = ( a.R->data[2] - a.R->data[6] ) * s;
        q.z() = ( a.R->data[3] - a.R->data[1] ) * s;
    }
    else {
        if ( a.R->data[0] > a.R->data[4] && a.R->data[0] > a.R->data[8] ) {
            double s = 2.0f * sqrtf( 1.0f + a.R->data[0] - a.R->data[4] - a.R->data[8]);
            q.w() = (a.R->data[7] - a.R->data[5] ) / s;
            q.x() = 0.25f * s;
            q.y() = (a.R->data[1] + a.R->data[3] ) / s;
            q.z() = (a.R->data[2] + a.R->data[6] ) / s;
        }
        else if (a.R->data[4] > a.R->data[8]) {
            double s = 2.0f * sqrtf( 1.0f + a.R->data[4] - a.R->data[0] - a.R->data[8]);
            q.w() = (a.R->data[2] - a.R->data[6] ) / s;
            q.x() = (a.R->data[1] + a.R->data[3] ) / s;
            q.y() = 0.25f * s;
            q.z() = (a.R->data[5] + a.R->data[7] ) / s;
        } else {
            double s = 2.0f * sqrtf( 1.0f + a.R->data[8] - a.R->data[0] - a.R->data[4] );
            q.w() = (a.R->data[3] - a.R->data[1] ) / s;
            q.x() = (a.R->data[2] + a.R->data[6] ) / s;
            q.y() = (a.R->data[5] + a.R->data[7] ) / s;
            q.z() = 0.25f * s;
        }
    }

    //std::cout<<std::setprecision(4) <<" W = " << q.w() <<" , X = " << q.x() <<" , Y = " << q.y() <<" , Z = " << q.z()  << std::endl;
}


void fromQuaternionToRPY(float& roll,float& pitch,float& yaw,Eigen::Quaternion<float>& q) {
    double sqw = q.w()*q.w();
    double sqx = q.x()*q.x();
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();
    double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
    double test = q.x()*q.y() + q.z()*q.w();
    //if (test > 0.499*unit) { // singularity at north pole
        if (test > 0.400*unit) { // singularity at north pole
            pitch = 2 * atan2(q.x(),q.w());
            yaw = M_PI/2;
            roll = 0;
        return;
    }
    //if (test < -0.499*unit) { // singularity at south pole
    if (test < -0.400*unit) {
        pitch = -2 * atan2(q.x() ,q.w());
        yaw = -M_PI/2;
        roll = 0;
        return;
    }

    pitch = atan2(2*q.y()*q.w()-2*q.x()*q.z() , sqx - sqy - sqz + sqw);
    yaw = asin(2*test/unit);
    roll = atan2(2*q.x()*q.w()-2*q.y()*q.z() , -sqx + sqy - sqz + sqw);

}

*/

bool checkInBoundaries(apriltag_detection_t* det )
{
    
    for(size_t i = 0 ; i < 4 ; i++)
    {
        if(    ( det->p[i][1] > 20 && det->p[i][1] < 470 )        &&  ( det->p[i][0] >20  && det->p[i][0] < 630  )   )
            continue;
        else
        {
            return false;
        }
    }
    return true;

};

int main(int argc, char *argv[])
{

    /*inizio opzioni apriltag Detector*/
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_string(getopt, 'p', "pubAddr", "ppubAddr", "pubAddr");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize camera
    //VideoCapture cap(0);
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    publisherAddress = getopt_get_string(getopt, "pubAddr");
    const char *famname = getopt_get_string(getopt, "family");

    //    const char *famname = fam.c_str();
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
        } else if (!strcmp(famname, "tag25h9")) {
            tf = tag25h9_create();
        } else if (!strcmp(famname, "tag16h5")) {
            tf = tag16h5_create();
        } else if (!strcmp(famname, "tagCircle21h7")) {
            tf = tagCircle21h7_create();
        } else if (!strcmp(famname, "tagCircle49h12")) {
            tf = tagCircle49h12_create();
        } else if (!strcmp(famname, "tagStandard41h12")) {
            tf = tagStandard41h12_create();
        } else if (!strcmp(famname, "tagStandard52h13")) {
            tf = tagStandard52h13_create();
        } else if (!strcmp(famname, "tagCustom48h12")) {
            tf = tagCustom48h12_create();
        }
        else {
            printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
            exit(-1);
    }


    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");


    aprilcodePublisher = new Common::Publisher(publisherAddress);
    std::string out;

    Mat frame, gray;
    
    float x = 0 ;
    float y =0 ;
    float z  =0;
    float roll=0; float pitch = 0; float yaw=0;
    nlohmann::json j;
    markerPoseData marker,pastMarker;
    pastMarker.x = 0;
    pastMarker.y = 0;
    pastMarker.z = 0;
    pastMarker.roll = 0;
    pastMarker.pitch = 0;
    pastMarker.yaw = 0;
    pastMarker.to_json(j);

    std::string  tmp_string ;
    const char* out_cstring ;;
    
    while (true) {

        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        //detections->size tells how many apriltag are detected in the image
        //cout<<"Size if zarray (zarray->size) " << detections->size << endl;

        // Draw detection outlines            
        apriltag_pose_t pose;

            
        
            
        for (int i = 0; i < zarray_size(detections); i++) {


            apriltag_detection_t *det;

            zarray_get(detections, i, &det);

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = 0.054;
            info.fx = 632.74999736;//683.75954508;//661.25;
            info.fy = 633.5834324;//687.37260599;//667.81;
            info.cx = 312.43271146;//264.12598696;//299.04; 
            info.cy = 229.88570203;//237.96954446;//250.50;
            int flag;
            flag = 0;
            double err = estimate_tag_pose( &info,  &pose, &flag);
            
            if(flag)
            {
                //j["id"] = det->id;
                //pastMarker.x = x;
                //pastMarker.y = y;
               // pastMarker.z = z;
                //pastMarker.roll = roll;
               // pastMarker.pitch = pitch;
               // pastMarker.yaw = yaw;
                std::cout << "FLAG"<<std::endl;
                marker.inView = 0;
                marker.to_json(j);
                tmp_string = j.dump();
                out_cstring = tmp_string.c_str();
                
                aprilcodePublisher->send("MARKER", out_cstring, strlen(out_cstring) );
                continue;
            }
            x = pose.t->data[0] ;
            y = pose.t->data[1] ;
            z = pose.t->data[2] ;
            
            Eigen::Matrix3f R;
            R << pose.R->data[0] ,pose.R->data[1]  ,pose.R->data[2],
                pose.R->data[3] ,pose.R->data[4] ,pose.R->data[5],
                pose.R->data[6] ,pose.R->data[7], pose.R->data[8]; 
            
            fromRotationToRPYAngle(roll,pitch,yaw,R);
            
           
            std::cout<< setprecision(4) << "x = " << x <<  "y = " << y << "z = " << z << std::endl;
           
            //std::cerr << err <<std::endl;
            //estimate_pose_for_tag_homography( &info,  &pose);


            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

            /*bool in_bound = checkInBoundaries(det);
            if(!in_bound)
            {
                continue;
            }*/
            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                                        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

            j["id"] = det->id;
            marker.x = x;
            marker.y = y;
            marker.z = z;
            marker.roll = roll;
            marker.pitch = pitch;
            marker.yaw = yaw;
            marker.inView = 1;

            marker.to_json(j);
            tmp_string = j.dump();
            out_cstring = tmp_string.c_str();
            aprilcodePublisher->send("MARKER", out_cstring, strlen(out_cstring) );
            
            pastMarker.x = x;
            pastMarker.y = y;
            pastMarker.z = z;
            pastMarker.roll = roll;
            pastMarker.pitch = pitch;
            pastMarker.yaw = yaw;
            pastMarker.inView = 1;
            pastMarker.to_json(j);


        }

        imshow("immagine" , frame);
        waitKey(10);
        apriltag_detections_destroy(detections);

         if (ctrl_c_pressed){

                std::cout << std::endl << "---------------------------------------------------------" << std::endl;
                std::cout <<              "-                   STOP REQUESTED                      -";
                std::cout << std::endl << "---------------------------------------------------------" << std::endl <<std::endl;

                break;
        }



        //float yaw   = atan2(pose.R->data[1], pose.R->data[0]  );//  atan2(R(2,1),R(1,1));
        //float pitch = atan2( pose.R->data[2]  , sqrt( pow(pose.R->data[5],2) + pow(pose.R->data[8],2)) );  //atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2)));
        //float roll  = atan2(pose.R->data[5] , pose.R->data[8] ); //atan2(R(3,2),R(3,3));

            //std::this_thread::sleep_for(std::chrono::milliseconds(2000));
       //    }
    }

    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }


    getopt_destroy(getopt);

    return 0;
}



