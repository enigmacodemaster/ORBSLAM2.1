/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

#include <unistd.h>
#include <boost/filesystem.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <fstream>
#include <lcm/lcm-cpp.hpp>
#include "SLAMDATA.h"

lcm::LCM lcm_slam;

namespace ORB_SLAM2
{

// ===== RCIR ==== Modified constructor to include all paths for map save and load
System::System(const std::string& strVocFile, const std::string& strSettingsFile, 
               const std::string& path_to_map, const std::string& strEnc,
               const int operation_mode, const eSensor sensor, const bool bUseViewer): 
               mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),
               mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false),
               path_to_map_(path_to_map), operation_mode_(operation_mode) 
{
    // Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if (!fsSettings.isOpened()) 
    {
        std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load ORB vocabulary
    std::cout << std::endl;
    std::cout << "Loading ORB Vocabulary...." << std::endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if (!bVocLoad) 
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Falied to open at: " << strVocFile << std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "Loaded ORB Vocabulary!" << std::endl;

    find_operation_mode(strSettingsFile);

    if (operation_mode_ == 0) 
    {   // 0: SLAM mode
        //Create KeyFrame Database
        std::cout << "---------" << endl;
        std::cout << "SLAM mode" << endl; 
        std::cout << "---------" << endl;
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Map
        mpMap = new Map();

        set_prev_map_state(0);
    }
    else if (operation_mode_ == 1) 
    {   // 1: Localization mode
        // Load existing map
        std::cout << "-----------------" << endl;
        std::cout << "Localization mode" << endl; 
        std::cout << "-----------------" << endl;
        {
            std::ifstream ifs(path_to_map_);
            boost::archive::binary_iarchive iar(ifs);
            iar >> mpMap;
        }
        std::cout << std::endl;
        std::cout << "Loaded previous map!" << std::endl;

        // Set keyframe db's vocabulary;
        mpKeyFrameDatabase = mpMap->get_keyframe_db();
        mpKeyFrameDatabase->set_vocabulary(mpVocabulary);

        // Set keyframe's vocabulary
        mpMap->set_keyframe_vocabulary(mpVocabulary);

        // Set prev_map_state_
        set_prev_map_state(1);
    }
    else if(operation_mode_ == 2)
    {
        std::cout << std::endl;
        std::cout << "operation_mode_ == 2" << std::endl;
    }
    
    // Create temporary maps
    //Map* temp_vo_map = new Map();
    //Map* filter_map = new Map();

    // Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    //mpMapDrawer = new MapDrawer(mpMap, temp_vo_map, strSettingsFile);

    // Initialize the CacheEncoder thread and launch
    mpEncoder = new Encoder(strEnc, strSettingsFile);
    mptEncoder = new thread(&ORB_SLAM2::Encoder::CacheEncoder, mpEncoder);

    // Initialize the Tracking thread
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, mpEncoder, strSettingsFile, mSensor);

    // Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);

    // Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary,
                                   mSensor!=MONOCULAR, strSettingsFile);

    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    // Initialize the Viewer thread
    if (bUseViewer) 
    {
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    std::cout << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "SAIC Fisheye SLAM initialization is done!" << std::endl;
    std::cout << "-----------------------------------------" << std::endl;

}  

void System::find_operation_mode(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);


    std::string map_name = fSettings["map_name"];

    boost::filesystem::path settings_file_fullpath(strSettingPath);
    boost::filesystem::path basepath = settings_file_fullpath.parent_path();

    boost::filesystem::path path_to_map;
    path_to_map = basepath /  boost::filesystem::path(map_name);
    path_to_map_ = path_to_map.string();
    
    if (boost::filesystem::exists(path_to_map_)) 
    {
        operation_mode_ = 1; // 1: Localization mode

        /* Check if previous localization pose file exists
        fullpath = basepath
        / boost::filesystem::path(
        path_to_map.stem().string() + "_localization_poses.txt");
        if (boost::filesystem::exists(fullpath)) 
        {
            std::cerr << "Localization poses with the same file name already exists!" << std::endl;
            exit(-1);
        }
        */

        boost::filesystem::path path_to_scale;
        path_to_scale = basepath /  boost::filesystem::path("Scale.txt");
        std::string path_to_scale_ = path_to_scale.string();

        ifstream filedata;

        filedata.open(path_to_scale_);
        if (!filedata.is_open())
        {
            cerr << "Cannot open the scale txt file!" << endl;
        }
        else
        {
            float data;
            int count = 1;

            while (!filedata.eof())
            {
                filedata >> data;

                if(count == 1)
			    {
                    sx = data;
                    count++;
                }

                if(count == 2)
			    {
                    sy = data;
                    count++;
                }
            }
        }
    } 
    else 
    {
        operation_mode_ = 0; // 0: SLAM mode
    }
}

/*
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}
*/

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);
//cout << "coming out GrabImageMonocular" << endl;
    //--------------lcm

    if(Tcw.rows  == 4 && Tcw.cols == 4)
    {
        cv::Mat Rcw_lcm =Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw_lcm = Tcw.rowRange(0,3).col(3);

        cv::Mat Rwc_lcm = Rcw_lcm.t();
        cv::Mat Ow_lcm = -Rwc_lcm*tcw_lcm;

        cv::Mat Twc_lcm;

        Twc_lcm = cv::Mat::eye(4,4,CV_32F);
        Rwc_lcm.rowRange(0,3).colRange(0,3).copyTo(Twc_lcm.rowRange(0,3).colRange(0,3));
        Ow_lcm.copyTo(Twc_lcm.rowRange(0,3).col(3));  //#Get Twc

        float camAg = 0.5629;
        Twc_lcm.at<float>(0,3) = Twc_lcm.at<float>(0,3) * sx;
        Twc_lcm.at<float>(1,3) = cos(camAg) * Twc_lcm.at<float>(1,3) + sin(camAg) * Twc_lcm.at<float>(2,3);
        Twc_lcm.at<float>(2,3) = (-sin(camAg) * Twc_lcm.at<float>(1,3) + cos(camAg) * Twc_lcm.at<float>(2,3)) * sy;

        SLAMDATA slamdata;
        unsigned int k=0;
        for(unsigned int row = 0; row<3; row++)
        {
            for(unsigned int col = 0; col<4; col++)
            {
                slamdata.slam_data[k++] = Twc_lcm.at<float>(row,col);
            }

        }

        if(mpTracker->mState == mpTracker->OK && !Tcw.empty())
            slamdata.slam_data[k] = 1;
        else
            slamdata.slam_data[k] = 0;

        if(lcm_slam.good())
        {
            lcm_slam.publish("SLAMDATA", &slamdata);
            std::cout<<"Send SLam Data..."<<std::endl;
        }
        else
        {
            std::cout<<"lcm error occur!"<<std::endl;
        }
    }

    //---------------lcm


    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    cout << "system will be shutdown" << endl;
    mpLocalMapper->RequestFinish();
    cout << "request localmapper finish" << endl;
    mpLoopCloser->RequestFinish();
    cout << "request loopcloser finish" << endl;
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        cout << "request mpviewer finish" << endl;
        
        while(!mpViewer->isFinished())
        {
            usleep(5000);
            cout << "mpviewer not finish yet" << endl;
        }
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished())
    {
        usleep(5000);
        cout << "localMapper not finish yet" << endl;
    }

    while(!mpLoopCloser->isFinished())
    {
        usleep(5000);
        cout << "LoopCloser not finish yet" << endl;
    }
/*
    while(!mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
        cout << "RunningGBA not finish yet" << endl;
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
*/
    cout << "shundown finish!" << endl;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

// ============ RCIR
void System::retrieve_pose_history() 
{
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    // Frame pose is stored relative to its reference keyframe 
    // (which is optimized by BA and pose graph). We need to get first 
    // keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.
    
    // For each frame we have a reference keyframe (lRit), 
    // the timestamp (lT) and a flag (lbL) which is true when tracking failed.
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    
    for (list<cv::Mat>::iterator lit= mpTracker->mlRelativeFramePoses.begin(),
        lend = mpTracker->mlRelativeFramePoses.end();
        lit!=lend; lit++, lRit++, lT++, lbL++) 
    {
        if (*lbL) 
        {
            continue;
        }

        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
       
        while (pKF->isBad()) 
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;
        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        cv::Mat pose = cv::Mat::eye(4, 4, CV_32F);

        Rwc.rowRange(0,3).colRange(0,3).copyTo(pose.rowRange(0,3).colRange(0,3));
        twc.copyTo(pose.rowRange(0,3).col(3));

        pose_history_.push_back(std::make_pair(pose, *lT));
    }
}

void System::save_slam_map_and_poses() 
{
    std::cout << "=======================================================" << std::endl;
    std::cout << "Saving SLAM map! " << std::endl;

    // Wait until all threads effectively stop
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || !mpLoopCloser->isFinishedGBA()) 
    {
        usleep(10000);
    }
    std::cout << "All the other threads finished their jobs!" << std::endl;
    
    {
        std::ofstream ofs(path_to_map_);
        boost::archive::binary_oarchive oar(ofs);
        oar << mpMap;
    }
    std::cout << "Finish saveing SLAM map!" << std::endl;
    
    std::cout << "Saving SLAM poses!" << std::endl;
    retrieve_pose_history();
    {
        boost::filesystem::path boost_path = path_to_map_;
        boost::filesystem::path basepath = boost_path.parent_path();
        boost::filesystem::path filename = boost_path.stem();
        boost::filesystem::path fullpath;

        fullpath = basepath
                / boost::filesystem::path(filename.string() + "_SLAM_poses.txt");
        std::string path_to_save = fullpath.string();
    
        std::ofstream ofs(path_to_save);
        ofs << fixed; 
        for (auto p: pose_history_) 
        {
            cv::Mat pose = p.first;
            double time = p.second;
            
            ofs << setprecision(6) << time << " " << setprecision(9)
                << pose.at<float>(0,0) << " " << pose.at<float>(0,1) << " "
                << pose.at<float>(0,2) << " " << pose.at<float>(0,3) << " "
                << pose.at<float>(1,0) << " " << pose.at<float>(1,1) << " "
                << pose.at<float>(1,2) << " " << pose.at<float>(1,3) << " "
                << pose.at<float>(2,0) << " " << pose.at<float>(2,1) << " "
                << pose.at<float>(2,2) << " " << pose.at<float>(2,3) << std::endl;
        }
        ofs.close();
    }
    std::cout << "Saved SLAM poses!" << std::endl;     

    set_save_state(1);
}

void System::save_localization_poses() 
{
    std::cout << "=======================================================" << std::endl;
    std::cout << "Saving localization poses!" << std::endl;

    // Wait until all threads effectively stop
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || !mpLoopCloser->isFinishedGBA()) 
    {
        usleep(10000);
    }
    std::cout << "All the other threads finished their jobs!" << std::endl;

    retrieve_pose_history();
    {
        boost::filesystem::path boost_path = path_to_map_;
        boost::filesystem::path basepath = boost_path.parent_path();
        boost::filesystem::path filename = boost_path.stem();
        boost::filesystem::path fullpath;

        fullpath = basepath 
                / boost::filesystem::path(filename.string() + "_localization_poses.txt");
        std::string path_to_save = fullpath.string();
    
        std::ofstream ofs(path_to_save);
        ofs << fixed; 
        for (auto p: pose_history_) 
        {
            cv::Mat pose = p.first;
            double time = p.second;
            
            ofs << setprecision(6) << time << " " << setprecision(9)
                << pose.at<float>(0,0) << " " << pose.at<float>(0,1) << " "
                << pose.at<float>(0,2) << " " << pose.at<float>(0,3) << " "
                << pose.at<float>(1,0) << " " << pose.at<float>(1,1) << " "
                << pose.at<float>(1,2) << " " << pose.at<float>(1,3) << " "
                << pose.at<float>(2,0) << " " << pose.at<float>(2,1) << " "
                << pose.at<float>(2,2) << " " << pose.at<float>(2,3) << std::endl;
        }
        ofs.close();
    }
    std::cout << "Saved localization poses!" << std::endl;    
    
    set_save_state(1);
}

void System::set_prev_map_state(int state) 
{
    std::unique_lock<std::mutex> state_mutex_;
    prev_map_state_ = state;
}

void System::set_save_state(int state) 
{
    std::unique_lock<std::mutex> state_mutex_;
    save_state_ = state;
}

int System::get_prev_map_state() 
{
  std::unique_lock<std::mutex> state_mutex_;
  return prev_map_state_;
}

} //namespace ORB_SLAM
