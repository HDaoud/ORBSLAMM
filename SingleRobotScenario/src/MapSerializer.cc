/**
* This file is part of iORB-SLAM.
*
* Copyright (C) 2016 Hayyan Daoud <hayyan dot d at gmail dot com> (University of Malaya)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
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
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#include "../include/MapSerializer.h"
#include <mutex>

#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <stdio.h>

#ifdef WIN32
#include "direct.h"
#endif


namespace iORB_SLAM
{
    using namespace std;
    
    MapSerializer::MapSerializer() 
    {
    }
    
    MapSerializer::MapSerializer(Map* map): mpMap(map) 
    {
    }

    MapSerializer::MapSerializer(std::vector<Map*> maps): mpMap(NULL),
    mvpMaps(maps) 
    {
    }
    
    /**
    * Destructor
    */
   MapSerializer::~MapSerializer()
   {
   }
   
   MapSerializer::MapStatus MapSerializer::SaveMaps(std::vector<Map*> mvpMaps, std::string folderName)
   {
       MapStatus ms = MAP_OK;
       MapStatus total_ms = MAP_OK;
       cout<<"Saving Maps to: " << folderName << endl;
      
       for(std::vector<Map*>::iterator it = mvpMaps.begin(), end = mvpMaps.end(); it != end; it++)
        {

            ostringstream os;
            os << folderName << "/map" << setfill( '0' ) << setw( 6 ) << (*it)->mnId;
            folderName = os.str();


            ms = SaveMap((*it), folderName);
            if(ms != MAP_OK)
                total_ms = MAP_FAILED;
        }
       
       return total_ms;
   }
   
   MapSerializer::MapStatus MapSerializer::SaveMap(Map* pMap, std::string folderName)
   {
       //MapStatus ms = MAP_OK;
       
       char mapId[10];
       sprintf(mapId, "%d", pMap->mnId);
       
       
       cout << "Saving Map " << pMap->mnId << " to " << folderName << endl;
       
       if(pMap == NULL)
       {
           cout << "SaveMap: NULL map pointer. Abort!" << endl;
           return MAP_FAILED;
       }
       
       //Register the map
       mpMap = pMap;
       
       //does the dir exists
       /*
        struct stat st;
        if( stat( folderName.c_str(), &st ) != 0 )
        {
            #ifdef WIN32
                int err = _mkdir( sDirName.c_str());
            #else
                int err = mkdir( folderName.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
            #endif
                if( err != 0 )
                {
                  cout << "Failed to make dir " << folderName << "! Aborting." << endl;
                  return MAP_FAILED;
                }
        }
        
        */
       
       // Get Map Mutex -> Map cannot be changed
        //unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        
        TiXmlDocument xmlDoc;     //XML file

        string sMapFileName = folderName + "/map" + mapId + ".xml";

        TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
        xmlDoc.LinkEndChild(decl);

        TiXmlElement* rootNode = new TiXmlElement(MAP_XML_ID);
        xmlDoc.LinkEndChild( rootNode );
        rootNode->SetAttribute("version", MAP_VERSION);

        ////////////  save keyframes and map points  ////////////
        bool bOK = false;
        //_CreateSaveLUTs();                      // create lookup tables for the mappoints and keyframes

        bOK = _SaveKeyFrames(rootNode);   // recursively save each keyframe
        if( !bOK )  {
            //mutex.unlock(mpMap->mMutexMapUpdate);
          return MAP_FAILED;
        }
        bOK = _SaveMapPoints(rootNode);   // recursively save each map point
        if( !bOK )  {
          //mutex.unlock(mpMap->mMutexMapUpdate);
          return MAP_FAILED;
        }

        ////////////  save the uids for the keyframes and map points in the failure queue  ////////////
//        {
//          TiXmlElement * failElem = new TiXmlElement( "FailureQueue" );
//          rootNode->LinkEndChild( failElem );
//          failElem->SetAttribute( "size", mpMap->vFailureQueue.size() );
//
//          int k = -1, m = -1;
//
//          vector<std::pair<KeyFrame*, MapPoint*> >::iterator fq;
//          for( fq = mpMap->vFailureQueue.begin(); fq != mpMap->vFailureQueue.end(); fq++ )
//          {
//            k = _LookupKeyFrame( (*fq).first );
//            m = _LookupMapPoint( (*fq).second );
//            if( m != -1 && k != -1 )  {
//              TiXmlElement * kmElem = new TiXmlElement( "Failure" );
//              failElem->LinkEndChild( kmElem );
//              kmElem->SetAttribute( "kf", k );
//              kmElem->SetAttribute( "mp", m );
//            }
//          }
//        }  

        xmlDoc.SaveFile(sMapFileName);
        return MAP_OK;
       
   }
   
   bool MapSerializer::_SaveKeyFrames(TiXmlElement* rootNode)
   {
       /*
       std::string sKeyFramePath = sPath + "/KeyFrames";

       //create the dir if not there
       struct stat st;
       if(stat( sKeyFramePath.c_str(),&st ) != 0 )
       {
     #ifdef WIN32
            int err = _mkdir( sKeyFramePath.c_str());
     #else
            int err = mkdir( sKeyFramePath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH );
     #endif
            if(err != 0)
            {
              cerr << "Failed to make dir " << sKeyFramePath << endl;
              return false;
            }
        }
       */
       
        TiXmlElement* keyFramesNode = new TiXmlElement( "KeyFrames" );
        rootNode->LinkEndChild(keyFramesNode);
        
        std::vector<KeyFrame*> KeyFrames = mpMap->GetAllKeyFrames();
        std::sort(KeyFrames.begin(), KeyFrames.end(), KeyFrame::lId);
        
        keyFramesNode->SetAttribute( "size", KeyFrames.size());

//        int k = -1;

        for(std::vector<KeyFrame *>::iterator KFit = KeyFrames.begin(), KFend = KeyFrames.end() ; KFit !=KFend ; KFit++)
        {
            if(_SaveAKeyFrame ((*KFit), keyFramesNode))
            {
    //            //k = _LookupKeyFrame( (*KFit) );
    //            if( k == -1 ) {
    //              return false;
    //            }
            }
            else 
            {
              return false;
            }
        }
        
        /*
        TiXmlElement* keyFrameONode = new TiXmlElement( "KeyFrameOrigins" );
        rootNode->LinkEndChild( keyFrameONode );
        keyFrameONode->SetAttribute( "size", mpMap->mvpKeyFrameOrigins.size() );

        for(std::vector<KeyFrame *>::iterator KFit = mpMap->mvpKeyFrameOrigins.begin(), KFend = mpMap->mvpKeyFrameOrigins.end(); KFit != KFend; KFit++)
        {
          if(!_SaveAKeyFrame ((*KFit), sPath, keyFrameONode))
          {
//            k = _LookupKeyFrame( (*kf) );
//            if( k == -1 ) {
//              return false;
//            }
          }
          else 
          {
            return false;
          }
        }

         */
        return true;
   }
   
   bool MapSerializer::_SaveAKeyFrame(KeyFrame* pKF, TiXmlElement* keyFramesNode)
   {
        TiXmlElement* kfe = new TiXmlElement("KeyFrame");
        keyFramesNode->LinkEndChild(kfe);
        
        ostringstream os;
        std::string s;
            
        //save keyframe attributes
        {            
            kfe->SetAttribute("id", pKF->mnId);
            os << pKF->GetPose();
            s = os.str();
            //PruneWhiteSpace(s);
            kfe->SetAttribute("pose", s);

            os.str("");
            os << pKF->mTimeStamp;
            s = os.str();
            kfe->SetAttribute("TimeStamp", s);

            os.str("");
            os << pKF->GetCameraCenter();
            s = os.str();
            kfe->SetAttribute("CamCenter", s);
            
            //Number of Keypoints
            os.str("");
            os << pKF->N;
            s = os.str();
            kfe->SetAttribute("N", s);
            
            os.str("");
            os << pKF->mnScaleLevels;
            s = os.str();
            kfe->SetAttribute("ScaleLevels", s);
            
            os.str("");
            os << pKF->mfScaleFactor;
            s = os.str();
            kfe->SetAttribute("ScaleFactor", s);
            
            os.str("");
            os << pKF->mfLogScaleFactor;
            s = os.str();
            kfe->SetAttribute("LogScaleFactor", s);
        
        }
        
        //Camera Parameters
        {
            TiXmlElement* camElem = new TiXmlElement("Camera");
            kfe->LinkEndChild(camElem);

            os.str("");
            os << pKF->cx;
            s = os.str();
            camElem->SetAttribute("cx", s);

            os.str("");
            os << pKF->cy;
            s = os.str();
            camElem->SetAttribute("cy", s);

            os.str("");
            os << pKF->fx;
            s = os.str();
            camElem->SetAttribute("fx", s);

            os.str("");
            os << pKF->fy;
            s = os.str();
            camElem->SetAttribute("fy", s);
            /*
            os.str("");
            os << pKF->mbf;
            s = os.str();
            camElem->SetAttribute("mbf", s);
             */
            
        }
        
        //Vectors
        {
            TiXmlElement* vecElem = new TiXmlElement("Vectors");
            kfe->LinkEndChild(vecElem);
            
//            os.str("");
//            os << pKF->mvuRight;
//            s = os.str();
//            vecElem->SetAttribute("mvuRight", s);
//            
//            os.str("");
//            os << pKF->mvKeysUn;
//            s = os.str();
//            vecElem->SetAttribute("mvKeysUn", s);
            
            os.str("");
            os << pKF->mDescriptors;
            s = os.str();
            vecElem->SetAttribute("Descriptors", s);
            
//            os.str("");
//            os << pKF->mFeatVec;
//            s = os.str();
//            vecElem->SetAttribute("FeatVec", s);
            
            os.str("");
            os << pKF->mBowVec;
            s = os.str();
            vecElem->SetAttribute("BowVec", s);
            
            
        }
          
        return true;

    }
   
   bool MapSerializer::_SaveMapPoints(TiXmlElement* rootNode)
   {
        TiXmlElement * mapPointsNode = new TiXmlElement( "MapPoints" );
        std::vector<MapPoint *> mvpAllMapPoints = mpMap->GetAllMapPoints();
        
        rootNode->LinkEndChild( mapPointsNode );
        mapPointsNode->SetAttribute( "size",mvpAllMapPoints.size());
        
        
                
        for(std::vector<MapPoint *>::iterator mp = mvpAllMapPoints.begin(), mpEnd = mvpAllMapPoints.end(); mp != mpEnd; mp++)
        {
            if(_SaveAMapPoint((*mp), mapPointsNode))
            {
                
            }
            else
            {
                cerr << " could not save map point " << endl;
                return false;
            }
        }
        
        return true;
       
   }
   
   bool MapSerializer::_SaveAMapPoint(MapPoint* pMP, TiXmlElement* mapPointsNode)
   {
       TiXmlElement* mpe = new TiXmlElement("MapPoint");
        mapPointsNode->LinkEndChild(mpe);
        
        mpe->SetAttribute("id", pMP->mnId);
        
        std::string s;
        ostringstream os;
        
        os << pMP->GetWorldPos();
        s = os.str();
        mpe->SetAttribute("WorldPos", s);
        
        os.str("");
        os << pMP->mnFirstKFid;
        s = os.str();
        mpe->SetAttribute("FirstKF", s);
        
        //Reference KF
        os.str("");
        os << pMP->GetReferenceKeyFrame()->mnId;
        s = os.str();
        mpe->SetAttribute("refKFid", s);
        
        os.str("");
        os << pMP->nObs;
        s = os.str();
        mpe->SetAttribute("nObs", s);
        
        os.str("");
        os << pMP->isBad();
        s = os.str();
        mpe->SetAttribute("isBad", s);
        
        os.str("");
        os << pMP->GetDescriptor();
        s = os.str();
        mpe->SetAttribute("Descriptor", s);
        
        os.str("");
        os << pMP->GetMaxDistanceInvariance();
        s = os.str();
        mpe->SetAttribute("MaxDistanceInvariance", s);
        
        os.str("");
        os << pMP->GetMinDistanceInvariance();
        s = os.str();
        mpe->SetAttribute("MinDistanceInvariance", s);
        
        os.str("");
        os << pMP->GetNormal();
        s = os.str();
        mpe->SetAttribute("Normal", s);
        
        os.str("");
        os << pMP->mbTrackInView;
        s = os.str();
        mpe->SetAttribute("TrackInView", s);
        
        os.str("");
        os << pMP->mnTrackScaleLevel;
        s = os.str();
        mpe->SetAttribute("TrackScaleLevel", s);
        
        os.str("");
        os << pMP->mTrackViewCos;
        s = os.str();
        mpe->SetAttribute("TrackViewCos", s);
        
        os.str("");
        os << pMP->mTrackProjX;
        s = os.str();
        mpe->SetAttribute("TrackProjX", s);
        
        os.str("");
        os << pMP->mTrackProjY;
        s = os.str();
        mpe->SetAttribute("TrackProjY", s);
        
                
        
        return true;
    }
        
        
   MapSerializer::MapStatus MapSerializer::LoadMaps(std::string folderName)
   {
       MapStatus ms = MAP_OK;
       MapStatus total_ms = MAP_OK;
       cout<<"Loading Maps from: " << folderName << endl;
       
       //Finding how many maps and their ids
       std::vector<int> IDs;
       IDs.resize(3);
       IDs.push_back(0);
       IDs.push_back(1);
       IDs.push_back(2);
       //Initializing mvpMaps vector to the number of maps
       mvpMaps.resize(IDs.size());
      
       for(std::vector<Map*>::iterator it = mvpMaps.begin(), end = mvpMaps.end(); it != end; it++)
        {

            ostringstream os;
            string sMapFileName;
            
            os << folderName << "/map" << setfill( '0' ) << setw( 6 ) << IDs.back(); 
            IDs.pop_back();
            sMapFileName = os.str() + ".xml";
            cout << "Loading Map " << sMapFileName << endl;

            ms = LoadMap((*it), sMapFileName);
            if(ms != MAP_OK)
                total_ms = MAP_FAILED;
        }
       
       return total_ms; 
   }
   
   MapSerializer::MapStatus MapSerializer::LoadMap(Map* pMap, std::string sMapFileName)
   {
        TiXmlDocument mXMLDoc;
        //load the XML file
        if(!mXMLDoc.LoadFile(sMapFileName))
        {
          cerr << "Failed to load " << sMapFileName << ". Aborting." << endl;
          return MAP_FAILED;
        }
       
        TiXmlHandle hDoc(&mXMLDoc);
        TiXmlElement* pElem;
        TiXmlHandle hRoot(0);
        
        pElem = hDoc.FirstChildElement().Element();
        // should always have a valid root but handle gracefully if it does not
        if (!pElem)
        {
          cerr << "No root handle in XML file " << sMapFileName << endl;
          return MAP_FAILED;
        }

        string sID(MAP_XML_ID);
        string sVersion(MAP_VERSION);
        string sFileVersion = pElem->Attribute("version");  

        if( (sID.compare(pElem->Value()) != 0 ) && (sVersion.compare(sFileVersion) != 0 ) )
        {
            cerr << "Invalid XML file. Need a version " << sVersion << " " << sID
               << " XML file. Not a version " << sFileVersion << " " << pElem->Value() << " file." << endl;
          return MAP_FAILED;
        }
        
        hRoot = TiXmlHandle(pElem);
        
        mpMap = pMap;
        
        // load the keyframes
        bool bOK = _LoadKeyFrames(hRoot);
        if( !bOK )
        {
            //mpMap->Reset();
            
            return MAP_FAILED;
        }
        
        // load map points
        bOK = _LoadMapPoints(hRoot);
        if( !bOK ) {
//          mpMap->Reset();
          return MAP_FAILED;
        }
        
        return MAP_OK;
       
   }
   
   bool MapSerializer::_LoadKeyFrames(TiXmlHandle &hRoot)
   {
        int nSize = -1;
        TiXmlHandle pNode = hRoot.FirstChild("KeyFrames");
        pNode.ToElement()->QueryIntAttribute("size", &nSize);

        for(TiXmlElement* pElem = pNode.FirstChild().Element(); pElem != NULL; pElem = pElem->NextSiblingElement())
        {
          ///@TODO should do empty entry checking
          TiXmlHandle hkf(pElem);
          if(!_LoadAKeyFrame(hkf))
          {
            cerr << "Failed to Load keyframe " <<  pElem->Attribute("id")  << ". Abort." << endl;
            return false;
          }
        }

        int size = (int)mpMap->GetAllKeyFrames().size();
        if( size != nSize) 
        {
          cerr << "Loaded the wrong number of keyframes. " << size
              << " instead of " << nSize << ". Aborting" << endl;
          return false;
        }
        
        return true;
   }
   
   bool MapSerializer::_LoadAKeyFrame(TiXmlHandle &hKF)
   {
       int mnId = -1;
       TiXmlElement* kfe = hKF.ToElement();
       
       kfe->QueryIntAttribute("id", &mnId);
       
       if(mnId == -1)
       {
           cerr << "Error Loading Keyframe ID \n";
           return false;
       }
       
//       KeyFrame* pKF = new KeyFrame();
//       
//       pKF->mnId = mnId;
       
       
       
       
       return true;
       
   }

   bool MapSerializer::_LoadMapPoints(TiXmlHandle& hRoot)
   {
       return true;
   }
   
   bool MapSerializer::_LoadAMapPoint(TiXmlHandle& hMP)
   {
       return true;
   }
}//namespace