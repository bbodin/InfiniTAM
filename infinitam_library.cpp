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

static ITAMConfiguration * config;
static Vector2i inputSize;

static  sb_uchar3*   inputRGB;
static  uint16_t*    inputDepth;

static ITMUChar4Image* rgbwrapper ;
static ITMShortImage * depthwrapper ;
static ITMUChar4Image *rgb      ;
static ITMUChar4Image *depth    ;
static ITMUChar4Image *raycast  ;

static ITMLibSettings *internalSettings;

static ITMRGBDCalib calibration;


/**
 * Initialization.
 */

void sb_new_slam_configuration(SLAMBenchConfiguration ** slam_settings) {
    *slam_settings = new ITAMConfiguration();
}


bool sb_init_slam_system(SLAMBenchConfiguration * slam_settings)  {

    config = dynamic_cast<ITAMConfiguration*> (slam_settings);


    /**
     * Retrieve RGB and Depth sensors,
     *  - check input_size are the same
     *  - check camera are the same
     *  - get input_file
     */

    RGBFrame* rgb_frame = NULL;
    DepthFrame* d_frame = NULL;

    for (Sensor *s : slam_settings->get_sensors()) {
        if (s->get_format() == RGB_FRAME) {
            rgb_frame = dynamic_cast<RGBFrame*>(s);
        }
        if (s->get_format() == DEPTH_FRAME) {
            d_frame = dynamic_cast<DepthFrame*>(s);
        }
    }

    if (not (rgb_frame and d_frame)) {
        std::cerr << "Invalid sensors found, RGB or Depth not found." << std::endl;
        return false;
    }

    if  (rgb_frame->getSize() != d_frame->getSize()) {
        std::cerr << "Invalid sensors found, different sizes" << std::endl;
        return false;
    }




    sb_float4 depth_camera =  make_sb_float4(
                d_frame->getIntrinsics()[0],
                d_frame->getIntrinsics()[1],
                d_frame->getIntrinsics()[2],
                d_frame->getIntrinsics()[3]);

     depth_camera.x = depth_camera.x * d_frame->getSize()[0];
     depth_camera.y = depth_camera.y * d_frame->getSize()[1];
     depth_camera.z = depth_camera.z * d_frame->getSize()[0];
     depth_camera.w = depth_camera.w * d_frame->getSize()[1];


     sb_float4 rgb_camera =  make_sb_float4(
                rgb_frame->getIntrinsics()[0],
                rgb_frame->getIntrinsics()[1],
                rgb_frame->getIntrinsics()[2],
                rgb_frame->getIntrinsics()[3]);

     rgb_camera.x = rgb_camera.x * rgb_frame->getSize()[0];
     rgb_camera.y = rgb_camera.y * rgb_frame->getSize()[1];
     rgb_camera.z = rgb_camera.z * rgb_frame->getSize()[0];
     rgb_camera.w = rgb_camera.w * rgb_frame->getSize()[1];


    // Teddy RGB camera settings :
    // 0.787907813,1.049802083,0.550714063,0.5670875

    // Teddy D camera settings :
    // 0.896421875,1.196654167,0.541360938,0.518814583


    internalSettings = new ITMLibSettings();

    internalSettings->useBilateralFilter = config->useBilateralFilter;
    internalSettings->useApproximateRaycast = config->useApproximateRaycast;
    internalSettings->modelSensorNoise = config->modelSensorNoise;
    internalSettings->trackerType = config->trackerType;
    internalSettings->noICPRunTillLevel = config->noICPRunTillLevel;
    internalSettings->skipPoints = config->skipPoints;

    internalSettings->depthTrackerICPThreshold = config->depthTrackerICPThreshold;
    internalSettings->depthTrackerTerminationThreshold = config->depthTrackerTerminationThreshold;
    internalSettings->useSwapping = config->useSwapping;

    internalSettings->noHierarchyLevels = config->trackingRegime.size();
    delete internalSettings->trackingRegime;
    internalSettings->trackingRegime = new TrackerIterationType[internalSettings->noHierarchyLevels];
    for (int i = 0 ; i < internalSettings->noHierarchyLevels ; i++) {
        internalSettings->trackingRegime[i] = config->trackingRegime[i];
    }


    internalSettings->sceneParams.voxelSize = config->voxelSize;
    internalSettings->sceneParams.viewFrustum_min = config->viewFrustum_min;
    internalSettings->sceneParams.viewFrustum_max = config->viewFrustum_max;
    internalSettings->sceneParams.mu = config->mu;
    internalSettings->sceneParams.maxW = config->maxW;
    internalSettings->sceneParams.stopIntegratingAtMaxW = config->stopIntegratingAtMaxW;




    inputSize[0] = rgb_frame->getSize()[0] ;
    inputSize[1] = rgb_frame->getSize()[1] ;



    calibration.intrinsics_rgb.SetFrom(rgb_camera.x, rgb_camera.y,rgb_camera.z, rgb_camera.w, rgb_frame->getSize()[0], rgb_frame->getSize()[1]);
    calibration.intrinsics_d.SetFrom(depth_camera.x, depth_camera.y,depth_camera.z, depth_camera.w, d_frame->getSize()[0], d_frame->getSize()[1]);

    //calibration.intrinsics_d.SetFrom( 573.71,574.394,346.471,249.031,640,480);


    Matrix4f calib;

    // ICLNUIM
    //calib.m00 = 1.0f; calib.m10 = 0.0f; calib.m20 = 0.0f;   calib.m30 = 0.0f;
    //calib.m01 = 0.0f; calib.m11 = 1.0f; calib.m21 = 0.0f;   calib.m31 = 0.0f;
    //calib.m02 = 0.0f; calib.m12 = 0.0f; calib.m22 = 1.0f;   calib.m32 = 0.0f;
    //calib.m03 = 0.0f; calib.m13 = 0.0f; calib.m23 = 0.0f;   calib.m33 = 1.0f;


    // Teddy D trafo_rgb_to_depth calib :
    //calib.m00 =  0.9997490f; calib.m10 = 0.00518867f; calib.m20 =  0.0217975f;   calib.m30 =  0.2243070f;
    //calib.m01 = -0.0051649f; calib.m11 = 0.99998600f; calib.m21 = -0.0011465f;   calib.m31 = -0.5001670f;
    //calib.m02 = -0.0218031f; calib.m12 = 0.00103363f; calib.m22 =  0.9997620f;   calib.m32 =  0.0151706f;
    //calib.m03 =  0.0000000f; calib.m13 = 0.00000000f; calib.m23 =  0.0000000f;   calib.m33 =  1.0000000f;


    // Ensure RGB is identity to set depth as the trnaslation
    // TODO : Could do the matrix transformation but well,  not now

    if  (rgb_frame->getDirection() !=  Eigen::Matrix4f::Identity()) {
        std::cerr << "Invalid direction for rgb" << std::endl;
        return false;
    }

    // Teddy D trafo_rgb_to_depth calib :
    calib.m00 = d_frame->getDirection()(0,0); calib.m10 = d_frame->getDirection()(1,0); calib.m20 = d_frame->getDirection()(2,0); calib.m30 = d_frame->getDirection()(3,0);
    calib.m01 = d_frame->getDirection()(0,1); calib.m11 = d_frame->getDirection()(1,1); calib.m21 = d_frame->getDirection()(2,1); calib.m31 = d_frame->getDirection()(3,1);
    calib.m02 = d_frame->getDirection()(0,2); calib.m12 = d_frame->getDirection()(1,2); calib.m22 = d_frame->getDirection()(2,2); calib.m32 = d_frame->getDirection()(3,2);
    calib.m03 = d_frame->getDirection()(0,3); calib.m13 = d_frame->getDirection()(1,3); calib.m23 = d_frame->getDirection()(2,3); calib.m33 = d_frame->getDirection()(3,3);



    calibration.trafo_rgb_to_depth.SetFrom(calib);



    //std::stringstream src("affine 0.0002 0.0");
    //std::stringstream src("kinect 1135.09 0.0819141");

    ITMDisparityCalib::TrafoType type = ITMDisparityCalib::TRAFO_KINECT;
    switch (d_frame->getDisparityType()) {
        case DISPARITY_KINECT:
            type = ITMDisparityCalib::TRAFO_KINECT;
            break;
        case DISPARITY_AFFINE:
            type = ITMDisparityCalib::TRAFO_AFFINE;
            break;
        default :
            exit(1);
    }

    calibration.disparityCalib.SetFrom(d_frame->getDisparityParams()[0], d_frame->getDisparityParams()[1], type);


    rgbwrapper    = new ITMUChar4Image ( inputSize,true, false) ;
    depthwrapper  = new ITMShortImage  ( inputSize,true, false) ;

    mainEngine = new ITMMainEngine( internalSettings, &calibration, inputSize, inputSize);

    inputDepth = (uint16_t*) malloc(sizeof(uint16_t) *  inputSize[0] * inputSize[1]);
    inputRGB = (sb_uchar3*) malloc(sizeof(sb_uchar3) *  inputSize[0] * inputSize[1]);




    return true;

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

/*
 * Process frames.
 */

bool sb_update_frame (Sensor* s)  {
    switch (s->get_format()) {
    case DEPTH_FRAME : dynamic_cast<DepthFrame*>(s)->getDepthFrame(inputDepth); return true;
    case RGB_FRAME   : dynamic_cast<RGBFrame*>(s)->getRGBFrame((sb_uchar3*)inputRGB);  return true;
       default : return false;
    };
}

bool sb_process_once ()  {

    //if (imuSource != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, inputIMUMeasurement);

        for (int i = 0 ; i < inputSize[0] * inputSize[1] ; i++) {
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





/*
 * Output part.
 *
 */


sb_float3 sb_get_position () {

    ITMTrackingState *state = mainEngine->GetTrackingState();
    sb_float3 res = {state->pose_d->GetT()[0],state->pose_d->GetT()[1],state->pose_d->GetT()[2]};
    return  res;
}

Eigen::Matrix4f  sb_get_pose ()  {
    // TODO : This getter has not been checked yet
    Eigen::Matrix4f res;
    mainEngine->GetTrackingState()->pose_d->GetM().getValues(&res(0,0));
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


/*
 * Rendering part
 */


bool sb_initialize_ui(SLAMBenchUI * ui ) {

    rgb = new ITMUChar4Image(inputSize, true, false);
    depth = new ITMUChar4Image(inputSize, true, false);
    raycast = new ITMUChar4Image(inputSize, true, false);


    ui->init(0, RGB24, inputSize[0], inputSize[1]);
    ui->init(1, RGBA32, inputSize[0],inputSize[1]);
    ui->init(2, RGBA32, inputSize[0],inputSize[1]);
    ui->init(3, RGBA32, inputSize[0],inputSize[1]);
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

