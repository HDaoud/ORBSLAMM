/**
* This file is part of iORB-SLAM.
*
* Copyright (C) 2016-2017 Hayyan Daoud <hayyan dot d at gmail dot com> (University of Malaya)
* For more information see <https://github.com/hdaoud/ORBSLAMM>
*
* iORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* iORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORBSLAMM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MultiMapper.h"
#include"ORBmatcher.h"
#include"PnPsolver.h"
#include "Sim3Solver.h"
#include"Optimizer.h"

#include <mutex>

namespace iORB_SLAM
{

    MultiMapper::MultiMapper():mnFinishRequests(0), mnSLAMSystems(0), mnMaxMapId(0)
    {
    }

    //The first map is the main map (passed in System)
    MultiMapper::MultiMapper(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc): 
    mpMap(pMap), mnMaxMapId(0), mpKeyFrameDB(pDB), mpORBVocabulary(pVoc)//,mpiORBSystem(iORBSystem)
    {       
        //AddMap(pMap,pDB);
    }
    
    void MultiMapper::Run()
    {
        cout<<"MultiMapping Thread is running!\n";
        cout<<"Initial Number of Maps is: "<<mvMapAndKFDB.size()<<endl;
        mbFinished =false;
        mbUpdatingMapPoses = false;
        while(1)
        {
            // Check if there are Maps in the queue
            if(CheckNewMap())
            {
                mbClosingLoop = true;
                // Detect loop candidates and check covisibility consistency
                if(DetectLoop())
                {
//                    cout<<"MM Loop Detected!\n";
                    //SaveTrajectory("MMaps.txt");
                }
            }       
            
            mbClosingLoop = false;
            ResetIfRequested();

            if(CheckFinish())
                break;

            usleep(5000);
        }
                

        SetFinish();
    }
    
    bool MultiMapper::CheckNewMap()
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        return(!mvMapAndKFDB.empty());
    }
    
    bool MultiMapper::DetectLoop()
    {
        //unique_lock<mutex> lock(mMutexMultiMapper);
        //unique_lock<mutex> lock(mMutexLoopQueue);
        for(size_t iM=0, iendM=mvMapAndKFDB.size(); iM<iendM; iM++)
        {            
            Map* pMapBase = mvMapAndKFDB[iM].first;
            KeyFrameDatabase* pKFDB = mvMapAndKFDB[iM].second;
            
            
            for(size_t iiM=iM+1, iiendM=mvMapAndKFDB.size(); iiM<iiendM; iiM++)
            {    
                Map* pMap;
                std::vector<KeyFrame*> vpKeyFrames;
                
                {
                    unique_lock<mutex> lock(mMutexLoopQueue);
                    pMap = mvMapAndKFDB.at(iiM).first;
                    vpKeyFrames = pMap->GetAllKeyFrames();
                }
                
                
                mbMatchedBefore = false;
                mbSwapped = false;
                if(pMap->isAttachedToMap(pMapBase))
                {
                    continue;
                    //If less than 30 Keyframes passed since last matching
                    long Incr = pMap->mpLastKF->mnId - pMap->mpMatchingKF->mnId;
                    long diff = Incr - pMapBase->GetMaxKFid();
                    if(diff >= 30 || (diff < 0 && Incr >= 30))
                        mbMatchedBefore = true;
                    else
                    continue;
                }
                
                //Don't match a map with less than 10 Keyframes
                if(vpKeyFrames.size() <= 10)
                    continue;
                
                if(!vpKeyFrames.empty() && !pKFDB->empty())
                {
                    for(std::vector<KeyFrame*>::iterator Kit = vpKeyFrames.end()-1, Kend=vpKeyFrames.begin(); Kit >= Kend; Kit--)
                    {
                        KeyFrame* pKeyFrame = *Kit;
                                                
                        
                        if(pKeyFrame->isBad())// || pKeyFrame->TrackedMapPoints(1)<100)
                            continue;
                        
                        // avoid that local mapping erase it while it is being processed in this thread
                        pKeyFrame->SetNotErase();
                        
//                        Frame* pCurrentFrame = pKeyFrame->getSourceFrame();
//                        
//                        if(!pCurrentFrame)
//                            continue;
//                        
//                        // Compute Bag of Words Vector
//                        //pCurrentFrame->ComputeBoW(); //This generated errors when using PnP to merge maps
//                        
//                        vector<KeyFrame*> vpCandidateKFs = pKFDB->DetectRelocalizationCandidates(pCurrentFrame);
                        
                        // Compute reference BoW similarity score
                        // This is the lowest score to a connected keyframe in the covisibility graph
                        // We will impose loop candidates to have a higher similarity than this
                        const vector<KeyFrame*> vpConnectedKeyFrames = pKeyFrame->GetVectorCovisibleKeyFrames();
                        const DBoW2::BowVector &CurrentBowVec = pKeyFrame->mBowVec;
                        float minScore = 1;
                        for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
                        {
                            KeyFrame* pKFi = vpConnectedKeyFrames[i];
                            if(pKFi->isBad())
                                continue;
                            const DBoW2::BowVector &BowVec = pKFi->mBowVec;

                            float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

                            if(score<minScore)
                                minScore = score;
                        }

                        // Query the database imposing the minimum score
                        vector<KeyFrame*> vpCandidateKFs = pKFDB->DetectLoopCandidates(pKeyFrame, minScore);
                        
                        

                        if(vpCandidateKFs.empty())
                            continue;                     


                        const int nKFs = vpCandidateKFs.size();

                        // We perform first an ORB matching with each candidate
                        // If enough matches are found we setup a PnP solver
                        ORBmatcher matcher(0.75,true);

//                        vector<PnPsolver*> vpPnPsolvers;
//                        vpPnPsolvers.resize(nKFs);
                        
                        vector<Sim3Solver*> vpSim3Solvers;
                        vpSim3Solvers.resize(nKFs);

                        vector< vector<MapPoint*> > vvpMapPointMatches;
                        vvpMapPointMatches.resize(nKFs);

                        vector<bool> vbDiscarded;
                        vbDiscarded.resize(nKFs);

                        int nCandidates=0;

                        for(int i=0; i<nKFs; i++)
                        {
                            
                            KeyFrame* pKF = vpCandidateKFs[i];
                            
                            if(pKF->isBad())
                            {
                                vbDiscarded[i] = true;
                                continue;
                            }
                            
                            // avoid that local mapping erase it while it is being processed in this thread
                            pKF->SetNotErase();
                            
                            
                                
                            int nmatches = matcher.SearchByBoW(pKeyFrame,pKF,vvpMapPointMatches[i]);
                            //pCurrentFrame->ComputeBoW();
                            //usleep(1000);

                            //int nmatches = matcher.SearchByBoW(pKF, *pCurrentFrame, vvpMapPointMatches[i]);
                            if(nmatches<15)
                            {
                                vbDiscarded[i] = true;
                                pKF->SetErase();
                                continue;
                            }
                            else
                            {
                                //pCurrentFrame->ComputeBoW();
                                
                                //PnPsolver* pSolver = new PnPsolver(*pCurrentFrame,vvpMapPointMatches[i]);
                                //pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                                //vpPnPsolvers[i] = pSolver;

                                Sim3Solver* pSolver = new Sim3Solver(pKeyFrame,pKF,vvpMapPointMatches[i],false);
                                pSolver->SetRansacParameters(0.99,10,300);
                                vpSim3Solvers[i] = pSolver;
                                
                                nCandidates++;

                            }
                            
                        }

                        // Alternatively perform some iterations of P4P RANSAC
                        // Until we find a camera pose supported by enough inliers
                        bool bMatch = false;
                        //ORBmatcher matcher2(0.9,true);

                        while(nCandidates>0 && !bMatch)
                        {
                            for(int i=0; i<nKFs; i++)
                            {
                                if(vbDiscarded[i])
                                    continue;

                                KeyFrame* pKF = vpCandidateKFs[i];

                                // Perform 5 Ransac Iterations
                                vector<bool> vbInliers;
                                int nInliers;
                                bool bNoMore;

//                                PnPsolver* pSolver = vpPnPsolvers[i];
//                                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);
                                
                                Sim3Solver* pSolver = vpSim3Solvers[i];
                                cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                                // If Ransac reachs max. iterations discard keyframe
                                if(bNoMore)
                                {
                                    //cout<<"\tMM RANSAC Reaches Max\n";
                                    vbDiscarded[i]=true;
                                    pKF->SetErase();
                                    nCandidates--;
                                }
                                
                                // If a Camera Pose is computed, optimize
                                if(!Scm.empty())
                                {
                                    //Tcw.copyTo(pCurrentFrame->mTcw);
                                    //pKF->SetPose(Tcw);
                                   // set<MapPoint*> sFound;
                
                                    vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                                    for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                                    {
                                        if(vbInliers[j])
                                        {
                                           vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                                           //pCurrentFrame->mvpMapPoints[j]=vvpMapPointMatches[i][j];
                                           //sFound.insert(vvpMapPointMatches[i][j]);
                                        }
//                                        else
//                                            pCurrentFrame->mvpMapPoints[j]=NULL;
                                    }
                                                                        

                                    
                                    cv::Mat R = pSolver->GetEstimatedRotation();
                                    cv::Mat t = pSolver->GetEstimatedTranslation();
                                    const float s = pSolver->GetEstimatedScale();
                                    
                                    matcher.SearchBySim3(pKeyFrame,pKF,vpMapPointMatches,s,R,t,7.5);

                                    //Transformation between pKF of MapBase and pKeyframe of the new map.
                                    g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                                    
                                    const int nInliers = Optimizer::OptimizeSim3(pKeyFrame, pKF, vpMapPointMatches, gScm, 10, false);                                    
                                    //int nInliers = Optimizer::PoseOptimization(pCurrentFrame);
                                    
                                    if(nInliers>=20)
                                    {
                                        bMatch = true;
                                        mpMatchedKF = pKF;
                                        g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                                        mg2oScw = gScm*gSmw;
                                        mScw = Converter::toCvMat(mg2oScw);

                                        mvpCurrentMatchedPoints = vpMapPointMatches;
                                        break;
                                    }
                                   
                                }
                                
                            }
                        }
                                    
                        if(!bMatch)
                        {
                            pKeyFrame->SetErase();
                            continue;
                        }
                        
                        // Retrieve MapPoints seen in Loop Keyframe and neighbors
                        vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
                        vpLoopConnectedKFs.push_back(mpMatchedKF);
                        mvpLoopMapPoints.clear();
                        for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
                        {
                            KeyFrame* pKF = *vit;
                            vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
                            for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
                            {
                                MapPoint* pMP = vpMapPoints[i];
                                if(pMP)
                                {
                                    if(!pMP->isBad() && pMP->mnLoopPointForKF!=pKeyFrame->mnId)
                                    {
                                        mvpLoopMapPoints.push_back(pMP);
                                        pMP->mnLoopPointForKF=pKeyFrame->mnId;
                                    }
                                }
                            }
                        }

                        // Find more matches projecting with the computed Sim3
                        matcher.SearchByProjection(pKeyFrame, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

                        // If enough matches accept Loop
                        int nTotalMatches = 0;
                        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
                        {
                            if(mvpCurrentMatchedPoints[i])
                                nTotalMatches++;
                        }
                        //cout<<"Total Matches = "<<nTotalMatches<<endl;
                        if(nTotalMatches>=40)
                        {
                            //To prevent creating a new map or shutting down while MM is working on merging
                            mbUpdatingMapPoses = true;
                            
                            for(int i=0; i<nKFs; i++)
                                if(vpCandidateKFs[i]!=mpMatchedKF)
                                    vpCandidateKFs[i]->SetErase();
                            
                            //If this map has been matched before and its pose is updated, update the unmatched map to the "global" pose
                            if(pMap->isAttached() && !pMapBase->isAttached())
                            {
                                cout<<"Swapping maps for merging..\n";
                                
                                //Swap Maps
                                Map* t = pMap;
                                pMap = pMapBase;
                                pMapBase = t;
                                
                                //Inverse SIM3 transformation
                                g2o::Sim3 gSmw(Converter::toMatrix3d(mpMatchedKF->GetRotation()),Converter::toVector3d(mpMatchedKF->GetTranslation()),1.0);
                                g2o::Sim3 gSmw2(Converter::toMatrix3d(pKeyFrame->GetRotation()),Converter::toVector3d(pKeyFrame->GetTranslation()),1.0);
                                g2o::Sim3 gScm = mg2oScw*gSmw.inverse();
                                mg2oScw = gScm.inverse()*gSmw2;
                                        
                                //Swap Keyframes
                                KeyFrame* kt = pKeyFrame;
                                pKeyFrame = mpMatchedKF;
                                mpMatchedKF = kt;
                                                                                               
                                mbSwapped = true;
                            }
//                            else
//                                if(pMap->isAttached() && pMapBase->isAttached())
//                                {
//                                    //we should update all the attached maps to the new pose
//                                    
//                                    for(std::vector<Map*>::iterator it = pMap->getAttachedMaps().begin(), itend=pMap->getAttachedMaps().end(); it != itend; it++)
//                                    {
//                                        Map* pmMap = *it;
//                                        
//                                        //In Multi Robot Scenario:
//                                        //pmMap->StopLocalMapping();
//                                        
//                                        //attach to the baseMap
//                                        g2o::Sim3 g2oScw = pmMap->relativePoseToAttachedMap(pMap)*mg2oScw;
//                                        pmMap->attachToMap(pMapBase, g2oScw);
//                                        pMapBase->attachToMap(pmMap, g2oScw.inverse());
//                                        
//                                        
//                                        //UpdatePosesAndAdd(pmMap, pMapBase); //of pmMap keyframes and MapPoints
//                                        
//                                        //pmMap->ReleaseLocalMapping();
//                                    }
//                                }
                            
                            pMap->attachToMap(pMapBase, mg2oScw);
                            pMapBase->attachToMap(pMap, mg2oScw.inverse());
                            
                            

                            UpdatePosesAndAdd(pMap, pMapBase, mg2oScw, pKeyFrame);
                            
                            mbUpdatingMapPoses = false;
                            
                            //pMap->mnId = -1 - pMapBase->mnId;
                            //cout<<"Map"<<iiM<<" was removed!\n";
                            
                            return true;
                        }
                        else
                        {
                            for(int i=0; i<nKFs; i++)
                                vpCandidateKFs[i]->SetErase();
                            pKeyFrame->SetErase();
                            continue;
                        }
                                    
                    }
                }
            }
            
            
        }
        
        return false;
        
    }
    
    void MultiMapper::UpdatePosesAndAdd(Map* pMap, Map* pMapBase, const g2o::Sim3 g2oScw, KeyFrame* pKeyFrame)
    {
        cout << "\n\nMM Loop detected!" << endl;
        cout<<"Update Map"<<pMap->mnId<<"'s Keyframes and Map Points poses to be merged with Map"<<pMapBase->mnId<<endl;
        
        //retrieve the local mapper and loop closer of pMap
        LocalMapping* localMapper = mMapsLocalMappers[pMap];
        LoopClosing* loopCloser = mMapsLoopClosers[pMap];
        
//        LocalMapping* localMapperBase = mMapsLocalMappers[pMapBase];
        
        //To be run after pMap finishes GBA. using the RunGlobalBundleAdjustmentonMap(pMapBase, mpMatchedKF)
        //Optimizer::GlobalBundleAdjustemnt(pMapBase,20,NULL,pKeyFrame->mnId); 
        
        std::vector<KeyFrame*> mvpCurrentMapKFs;
        // Retrieve keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        if(mbSwapped || !mbMatchedBefore)
             mvpCurrentMapKFs = pMap->GetAllKeyFrames();
        else
        if(mbMatchedBefore)
        {
                mvpCurrentMapKFs = pKeyFrame->GetVectorCovisibleKeyFrames();
    //            mvpCurrentMapKFs = pMap->GetAllKeyFramesAfter(pMap->mpMatchingKF);
                mvpCurrentMapKFs.push_back(pKeyFrame);
        }
        
        //SaveMapTrajectory(pMap, "MapBeforeUpdate.txt");

        // Send a stop signal to Local Mapping
        // Avoid new keyframes are inserted while correcting the loop
        localMapper->RequestStop();
//        localMapperBase->RequestStop();
        
        // If a Global Bundle Adjustment is running, abort it
        if(loopCloser->isRunningGBA())
        {
            loopCloser->SetStopGBA(true);

            while(!loopCloser->isFinishedGBA())
                usleep(5000);

            loopCloser->GetGBAThread()->join();
            delete loopCloser->GetGBAThread();
        }

        // Wait until Local Mapping has effectively stopped
        while(!localMapper->isStopped() /*|| !localMapperBase->isStopped()*/)
        {
            usleep(1000);
        }
        

        // Ensure current keyframe is updated
        pKeyFrame->UpdateConnections();
        
        //This to solve the addVertex error (Id is already registered)
        long MaxKFID2 = pMap->GetMaxKFid();
        long MaxKFID = pMapBase->GetMaxKFid();
        pMap->setMaxKFid(MaxKFID + MaxKFID2 + 1);
        
        
        KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        CorrectedSim3[pKeyFrame]=g2oScw;
        cv::Mat Twc = pKeyFrame->GetPoseInverse();        


        {
            
            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            unique_lock<mutex> lock1(pMapBase->mMutexMapUpdate);

            for(vector<KeyFrame*>::iterator vit=mvpCurrentMapKFs.begin(), vend=mvpCurrentMapKFs.end(); vit!=vend; vit++)
            {
                KeyFrame* pKFi = *vit;
                
                //so only the baseMap's first keyframe is fixed in GBA
                if(pKFi->mnId == 0)
                    pKFi->SetNotFixed();

                cv::Mat Tiw = pKFi->GetPose();

                if(pKFi!=pKeyFrame)
                {
                    cv::Mat Tic = Tiw*Twc;
                    cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                    cv::Mat tic = Tic.rowRange(0,3).col(3);
                    g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                    g2o::Sim3 g2oCorrectedSiw = g2oSic*g2oScw;
                    //Pose corrected with the Sim3 of the loop closure
                    CorrectedSim3[pKFi]=g2oCorrectedSiw;
                }

                cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
                cv::Mat tiw = Tiw.rowRange(0,3).col(3);
                g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
                //Pose without correction
                NonCorrectedSim3[pKFi]=g2oSiw;
            }
            

            // Correct all MapPoints observed by current keyframe and neighbors, so that they align with the other side of the loop
            for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
            {
                KeyFrame* pKFi = mit->first;
                g2o::Sim3 g2oCorrectedSiw = mit->second;
                g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

                vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
                for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
                {
                    MapPoint* pMPi = vpMPsi[iMP];
                    if(!pMPi)
                        continue;
                    if(pMPi->isBad() || pMPi->mnCorrectedByKF==pKeyFrame->mnId)
                        continue;

                    // Project with non-corrected pose and project back with corrected pose
                    cv::Mat P3Dw = pMPi->GetWorldPos();
                    Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                    Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                    cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                    pMPi->SetWorldPos(cvCorrectedP3Dw);
                    pMPi->mnCorrectedByKF = pKeyFrame->mnId;
                    pMPi->mnCorrectedReference = pKFi->mnId;
                    pMPi->UpdateNormalAndDepth();
                }

                // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
                Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
                double s = g2oCorrectedSiw.scale();
                
                eigt *=(1./s); //[R t/s;0 1]

                cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

                pKFi->SetPose(correctedTiw);

                // Make sure connections are updated
                pKFi->UpdateConnections();
            }

            // Start Loop Fusion
            // Update matched map points and replace if duplicated
            for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
            {
                if(mvpCurrentMatchedPoints[i])
                {
                    MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                    MapPoint* pCurMP = pKeyFrame->GetMapPoint(i);
                    if(pCurMP)
                        pCurMP->Replace(pLoopMP);
                    else
                    {
                        pKeyFrame->AddMapPoint(pLoopMP,i);
                        pLoopMP->AddObservation(pKeyFrame,i);
                        pLoopMP->ComputeDistinctiveDescriptors();
                    }
                }
            }

        }
        
        // Project MapPoints observed in the neighborhood of the loop keyframe
        // into the current keyframe and neighbors using corrected poses.
        // Fuse duplications.
        SearchAndFuse(CorrectedSim3, pMap);
        
        // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        map<KeyFrame*, set<KeyFrame*> > LoopConnections;

        for(vector<KeyFrame*>::iterator vit=mvpCurrentMapKFs.begin(), vend=mvpCurrentMapKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

            // Update connections. Detect new links.
            pKFi->UpdateConnections();
            LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
            for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
            {
                LoopConnections[pKFi].erase(*vit_prev);
            }
            for(vector<KeyFrame*>::iterator vit2=mvpCurrentMapKFs.begin(), vend2=mvpCurrentMapKFs.end(); vit2!=vend2; vit2++)
            {
                LoopConnections[pKFi].erase(*vit2);
            }
        }

        // Optimize graph
        //Optimizer::OptimizeEssentialGraph(pMap, mpMatchedKF, pKeyFrame, NonCorrectedSim3, CorrectedSim3, LoopConnections, false);
        Optimizer::MMOptimizeEssentialGraph(pMap, mpMatchedKF, pKeyFrame, NonCorrectedSim3, CorrectedSim3, LoopConnections, false);

        pMap->mpMatchingKF = pKeyFrame;
        pMap->mpMatchedKF = mpMatchedKF;
        pMapBase->mpMatchingKF = mpMatchedKF;
        pMapBase->mpMatchedKF = pKeyFrame;

        // Add loop edge
        mpMatchedKF->AddLoopEdge(pKeyFrame);
        pKeyFrame->AddLoopEdge(mpMatchedKF);
        
        // Launch a new thread to perform Global Bundle Adjustment
        loopCloser->RequestRunGBA(pKeyFrame->mnId, pMap);
        
        // Loop closed. Release Local Mapping.
        localMapper->Release();
//        localMapperBase->Release();
        //SaveMapTrajectory(pMapBase,"Map1.txt");
        //SaveMapTrajectory(pMap,"Map2.txt");
    }
    
    
    void MultiMapper::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, Map* pMap)
    {
        ORBmatcher matcher(0.8);

        for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
        {
            KeyFrame* pKF = mit->first;

            g2o::Sim3 g2oScw = mit->second;
            cv::Mat cvScw = Converter::toCvMat(g2oScw);

            vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
            matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

            // Get Map Mutex
            unique_lock<mutex> lock(pMap->mMutexMapUpdate);
            const int nLP = mvpLoopMapPoints.size();
            for(int i=0; i<nLP;i++)
            {
                MapPoint* pRep = vpReplacePoints[i];
                if(pRep)
                {
                    pRep->Replace(mvpLoopMapPoints[i]);
                }
            }
        }
    }
    
    void MultiMapper::RequestReset()
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
            }
            usleep(5000);
        }
    }
    
    void MultiMapper::ResetIfRequested()
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            mvpMaps.clear();
            mvpKFDB.clear();
            
            mbResetRequested=false;
        }
    }

    
    bool MultiMapper::InitFromFile(const string& dirName)
    {
        //Load all Map xml files in the specified directory and then run the ORB matcher thread to look for merge
        
        return true;
    }
    
    //This to be used when pMap is finite and not being updated to reduce map numbers
    void MultiMapper::MergeMaps(Map* pMap, Map* pMapBase) 
    {
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);
        
        LocalMapping* localMapper = mMapsLocalMappers[pMap];
//        int i=0;
//        for(std::vector<MapAndLocalMapper>::iterator it = mvMapsLocalMappers.begin(), itend = mvMapsLocalMappers.end(); it != itend; it++)
//        {
//            if(it->first == pMap)
//            {
//                localMapper = it->second;
//                break;
//            }
//            i++;
//        }
        
        
        
        vector<KeyFrame*> vpKeyFrames = pMapBase->GetAllKeyFrames();
        //vector<MapPoint*> vpMapPoints = pMapBase->GetAllMapPoints();
        cout<<"Merging Maps...\n";
        int maxKFId = pMap->GetMaxKFid();
        //int maxMPid = pMap->GetMaxMPid();
        int id = 1;
        
        //Add KeyFrames
        for(vector<KeyFrame*>::iterator vit=vpKeyFrames.begin(), vend=vpKeyFrames.end(); vit != vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            if(pKFi->isBad())
                continue;
//            vector<MapPoint*> vpMapPoints = pKFi->GetMapPointMatches();
//            for(vector<MapPoint*>::iterator mpIt=vpMapPoints.begin(), mpEnd=vpMapPoints.end(); mpIt != mpEnd; mpIt++)
//            {
//                MapPoint* pMP = *mpIt;
//                if(pMP->isBad())
//                    continue;
//                pMP->mnId = maxMPid + id;
//                pMP->SwitchMap(pMap);
//                
//                if(pMP->mnFirstKFid == pKFi->mnId)
//                    pMP->mnFirstKFid = maxKFId + id;
//                pMap->AddMapPoint(pMP);
//            }
            pKFi->mnId = maxKFId + id;
            id++;
            pMap->AddKeyFrame(pKFi);
            localMapper->InsertKeyFrame(pKFi);
            
        }
        
        //Add MapPoints
//        for(vector<MapPoint*>::const_iterator vit=vpMapPoints.begin(), vend=vpMapPoints.end(); vit != vend; vit++)
//        {
//            MapPoint* pMP = *vit;
//            if(pMP->isBad())
//                continue;
//            pMapBase->AddMapPoint(pMP);
//        }
        
        //TODO: Erase pMapBase after being merged with pMap

    }

    
    //Maybe I will need to remove the pointer and pass by value - solved by switchMap approach
    void MultiMapper::AddMap(Map* pMap, KeyFrameDatabase* pKeyFrameDB)
    {
        
        unique_lock<mutex> lock(mMutexLoopQueue);
//        mpMap = pMap;
//        mpKeyFrameDB = pKeyFrameDB;
        
        if(pMap->mnId == 0)
            mnSLAMSystems++;
        
        if(pMap->mnId < mnMaxMapId)
        {
            pMap->mnId = mnMaxMapId;
            pMap->mnNxtId = mnMaxMapId + 1;
            
            //This to solve the addVertex error (Id is already registered)
//            long MaxKFID = mvpMaps.at(mnMaxMapId -1)->GetMaxKFid();
//            pMap->setMaxKFid(MaxKFID + pMap->GetMaxKFid() + 1);
        }
        
        MapAndKFDB mapKfDB = make_pair(pMap, pKeyFrameDB);
        mvMapAndKFDB.push_back(mapKfDB);
        mvpMaps.push_back(pMap);
        mvpKFDB.push_back(pKeyFrameDB);
        
        mnMaxMapId++;
        
        cout<<"\n\t--> Map"<<mnMaxMapId - 1<<" was added!\n";
        
        //SaveTrajectory("MapsSeq.txt");//Check that every newly added map is correct in the final read of all maps
        
    }
    
    void MultiMapper::EraseMap(Map* pMap)
    {
        unique_lock<mutex> lock(mMutexMultiMapper);
        mvpMaps.erase(mvpMaps.begin() + pMap->mnId);
        mvpKFDB.erase(mvpKFDB.begin() + pMap->mnId);
    }
    
    vector<Map*> MultiMapper::GetAllMaps()
    {
        unique_lock<mutex> lock(mMutexMultiMapper);
        return mvpMaps;
    }
    
    void MultiMapper::SaveTrajectory(const string& filename)
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        cout << "\nSaving "<< mnMaxMapId <<" maps by their keyframes trajectories to " << filename << " ..." << endl;
        
        ofstream f;
        f.open(filename.c_str(), ios::app);
        f << fixed;
        
        for(std::vector<MapAndKFDB>::iterator it = mvMapAndKFDB.begin(), itend = mvMapAndKFDB.end(); it != itend; it++)
        //for(size_t k = 0, kend = mvpMaps.size(); k < kend; k++)
        {
            Map* pMap = it->first;
            //mvMapAndKFDB.pop_back();
            f<<"#Map #"<<pMap->mnId<<":\n";
           

            pMap->printAttachedMaps();
            vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();            
            sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);


            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];

                if(pKF->isBad())
                    continue;

                cv::Mat R = pKF->GetRotation().t();
                vector<float> q = Converter::toQuaternion(R);
                cv::Mat t = pKF->GetCameraCenter();
                f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

            }

            f<<"\n#----------------\n\n";
        }
        
        f.close();
        cout << endl << "trajectories saved!" << endl;
    }
    
    void MultiMapper::SaveMapTrajectory(Map* pMap, const string& filename)
    {
        cout << "\nSaving Map"<< pMap->mnId <<" by its keyframe's trajectory to " << filename << " ..." << endl;
        
        ofstream f;
        f.open(filename.c_str(), ios::app);
        f << fixed;
        
        vector<KeyFrame*> vpKFs = pMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        f<<"Map "<<pMap->mnId<<" :\n";
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
                continue;

            cv::Mat R = pKF->GetRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }

        f<<"\n----------------\n\n";
        
        
        f.close();
        cout << endl << "trajectories saved!" << endl;
    }
    
    void MultiMapper::SetLoopCloser(Map* pMap, LoopClosing* pLoopCloser)
    {
//        mpLoopCloser = pLoopCloser;
//        MapAndLoopCloser mapCloser = make_pair(pMap, pLoopCloser);
//        mvMapsLoopClosers.push_back(mapCloser);
        mMapsLoopClosers[pMap] = pLoopCloser;
    }
    
    void MultiMapper::SetTracker(Map* pMap, Tracking *pTracker)
    {        
//        mpTracker=pTracker;
        MapAndTracker mapTracker = make_pair(pMap, pTracker);
        mvMapsTrackers.push_back(mapTracker);
    }

    void MultiMapper::SetLocalMapper(Map* pMap, LocalMapping *pLocalMapper)
    {
//        mpLocalMapper=pLocalMapper;
//        MapAndLocalMapper mapMapper = make_pair(pMap, pLocalMapper);
//        mvMapsLocalMappers.push_back(mapMapper);
        mMapsLocalMappers[pMap] = pLocalMapper;
    }
    
    void MultiMapper::SetVocabulary(ORBVocabulary* pVoc)
    {
        mpORBVocabulary = pVoc;
    }

    
    void MultiMapper::RequestFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mnFinishRequests++;
        cout<<"Finish Requests = "<<mnFinishRequests<<" - SLAM Systems = "<<mnSLAMSystems<<endl;
        //mbFinishRequested = true;
    }

    bool MultiMapper::CheckFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        if(mnFinishRequests >= mnSLAMSystems)
        {
            SaveTrajectory("MMaps.txt");
            return true;
        }
        return false;
    }
    
    void MultiMapper::SetFinish()
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool MultiMapper::isFinished()
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }
    
    bool MultiMapper::GetmbUpdatingMapPoses()
    {
        return mbUpdatingMapPoses;
    }
    
    
}