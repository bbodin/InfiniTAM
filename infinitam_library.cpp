/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <perfstats.h>
#include <timings.h>
#include <ITAMApplication.h>
#include <SLAMBenchAPI.h>
#include <ITMMainEngine.h>
#include <Eigen/Core>


static ITMMainEngine* mainEngine ;

static  sb_uchar3*   inputRGB;
static  uint16_t*    inputDepth;
static ITAMConfiguration * config;
static Vector2i img_size;
static ITMUChar4Image* rgbwrapper ;
static ITMShortImage * depthwrapper ;

static ITMUChar4Image *rgb      ;
static ITMUChar4Image *depth    ;
static ITMUChar4Image *raycast  ;
static ITMLibSettings *internalSettings;

static ITMRGBDCalib calibration;

void sb_new_slam_configuration(SLAMBenchConfiguration ** slam_settings) {
    *slam_settings = new ITAMConfiguration();
}

bool sb_init_slam_system(SLAMBenchConfiguration * slam_settings)  {

    uint32_t mxcsr;
      uint32_t mask = 0;
      mask |= 0 << 12;   // precision mask
      mask |= 0 << 11;   // under flow mask
      mask |= 0 << 10;   // over flow mask
      mask |= 1 << 9;    // division by zero mask
      mask |= 0 << 7;    // invalid operator mask
      mask = ~mask;
      asm volatile ("stmxcsr %0" : "=m"(mxcsr));
      mxcsr &= mask;
      asm volatile ("ldmxcsr %0" : : "m"(mxcsr));



    config = dynamic_cast<ITAMConfiguration*> (slam_settings);

    // Teddy RGB camera settings :
    // 0.787907813,1.049802083,0.550714063,0.5670875

    // Teddy D camera settings :
    // 0.896421875,1.196654167,0.541360938,0.518814583

    sb_float4 camera =  make_sb_float4(
            config->get_camera().x,
            config->get_camera().y,
            config->get_camera().z,
            config->get_camera().w);

    camera.x = camera.x *  config->get_input_size().x;
    camera.y = camera.y *  config->get_input_size().y;
    camera.z = camera.z *  config->get_input_size().x;
    camera.w = camera.w *  config->get_input_size().y;

    internalSettings = new ITMLibSettings();

    img_size = {(int)config->get_input_size().x, (int)config->get_input_size().y};



    calibration.intrinsics_rgb.SetFrom(camera.x, camera.y,camera.z, camera.w, config->get_input_size().x, config->get_input_size().y);
    calibration.intrinsics_d.SetFrom(camera.x, camera.y,camera.z, camera.w, config->get_input_size().x, config->get_input_size().y);
    calibration.intrinsics_d.SetFrom( 573.71,574.394,346.471,249.031,640,480);


    Matrix4f calib;
    calib.m00 = 1.0f; calib.m10 = 0.0f; calib.m20 = 0.0f;   calib.m30 = 0.0f;
    calib.m01 = 0.0f; calib.m11 = 1.0f; calib.m21 = 0.0f;   calib.m31 = 0.0f;
    calib.m02 = 0.0f; calib.m12 = 0.0f; calib.m22 = 1.0f;   calib.m32 = 0.0f;
    calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f;   calib.m33 = 1.0f;


    // Teddy D trafo_rgb_to_depth calib :
    calib.m00 =  0.9997490f; calib.m10 = 0.00518867f; calib.m20 =  0.0217975f;   calib.m30 =  0.2243070f;
    calib.m01 = -0.0051649f; calib.m11 = 0.99998600f; calib.m21 = -0.0011465f;   calib.m31 = -0.5001670f;
    calib.m02 = -0.0218031f; calib.m12 = 0.00103363f; calib.m22 =  0.9997620f;   calib.m32 =  0.0151706f;
    calib.m03 =  0.0000000f; calib.m13 = 0.00000000f; calib.m23 =  0.0000000f;   calib.m33 =  1.0000000f;

    calibration.trafo_rgb_to_depth.SetFrom(calib);



    //std::stringstream src("affine 0.0002 0.0");
    std::stringstream src("kinect 1135.09 0.0819141");

    std::string word;
    float a,b;
    src >> word >> a >> b;
    if (src.fail()) return false;
    ITMDisparityCalib::TrafoType type = ITMDisparityCalib::TRAFO_KINECT;


    if (word.compare("kinect") == 0) {
        type = ITMDisparityCalib::TRAFO_KINECT;
        src >> a;
    } else if (word.compare("affine") == 0) {
        type = ITMDisparityCalib::TRAFO_AFFINE;
    } else {
        std::stringstream wordstream(word);
    }


    if ((a == 0.0f) && (b == 0.0f)) {
        type = ITMDisparityCalib::TRAFO_AFFINE;
        a = 1.0f/1000.0f; b = 0.0f;
    }
    calibration.disparityCalib.SetFrom(a, b, type);


    rgbwrapper    = new ITMUChar4Image (img_size,true, false) ;
    depthwrapper  = new ITMShortImage  (img_size,true, false) ;

    mainEngine = new ITMMainEngine( internalSettings, &calibration, img_size, img_size);

    inputDepth = (uint16_t*) malloc(sizeof(uint16_t) * config->get_input_size().x* config->get_input_size().y);
    inputRGB = (sb_uchar3*) malloc(sizeof(sb_uchar3) * config->get_input_size().x* config->get_input_size().y);




        return true;

}

bool sb_update_frame (void * data, frame_format type) {
    switch (type) {
    case DEPTH_FRAME : memcpy ( inputDepth,  data , img_size.x * img_size.y * sizeof (uint16_t)); return true;
    case RGB_FRAME   : memcpy ( inputRGB  ,  data , img_size.x * img_size.y * sizeof (sb_uchar3)); return true;
       default : return false;
       };
}
bool sb_process_once ()  {

    //if (imuSource != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);

        for (int i = 0 ; i < img_size.x * img_size.y ; i++) {
            rgbwrapper->GetData(MEMORYDEVICE_CPU)[i] = {inputRGB[i].x,inputRGB[i].y,inputRGB[i].z,255};
            depthwrapper->GetData(MEMORYDEVICE_CPU)[i] = inputDepth[i];

        }

        double timings[2] = {0.0,0.0};

        timings[0] = tock();
        mainEngine->ProcessFrame(rgbwrapper, depthwrapper);

#ifndef COMPILE_WITHOUT_CUDA
        ITMSafeCall(cudaThreadSynchronize());
#endif


        timings[1] = tock();

        config->sample("computation",  timings[1] - timings[0],PerfStats::Type::TIME);

    return true;

}

sb_float3 sb_get_position () {

    ITMTrackingState *state = mainEngine->GetTrackingState();

	sb_float3 res = {state->pose_d->GetT()[0],state->pose_d->GetT()[1],state->pose_d->GetT()[2]};
    return  res;
}

Eigen::Matrix4f  sb_get_pose ()  {
    assert(false);
    Eigen::Matrix4f res;
    return res;
}
bool             sb_get_tracked  ()  {
    return true;
}

bool sb_save_map (const char * filename , map_format format) {

        switch (format) {
        default : return false;
        }

}

bool sb_clean_slam_system() {

    delete mainEngine;
    delete internalSettings;

    if (raycast) delete raycast;
    if (depth) delete depth;
    if (rgb) delete rgb;
    if (inputRGB) delete inputRGB;

    return true;
}


bool sb_initialize_ui(SLAMBenchUI * ui ) {

    rgb = new ITMUChar4Image(img_size, true, false);
    depth = new ITMUChar4Image(img_size, true, false);
    raycast = new ITMUChar4Image(img_size, true, false);


    ui->init(0, RGB24, img_size.x,img_size.y);
    ui->init(1, RGBA32,  img_size.x,img_size.y);
    ui->init(2, RGBA32,  img_size.x,img_size.y);
    ui->init(3, RGBA32,  img_size.x,img_size.y);
    return true;
}

bool sb_update_ui(SLAMBenchUI * ui) {

    mainEngine->GetImage(raycast, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
    mainEngine->GetImage(depth, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    mainEngine->GetImage(rgb, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_RGB);

    ui->update(0, (void*) inputRGB);
    ui->update(1, (void*) raycast->GetData(MEMORYDEVICE_CPU));
    ui->update(2, (void*) depth->GetData(MEMORYDEVICE_CPU));
    ui->update(3, (void*) rgb->GetData(MEMORYDEVICE_CPU));


    return true;
}

