#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/xfeatures2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"

/*
 *  seed::cv is a wrapper for opencv
 */

//THIS IS NOT A STABLE LIBRARY!!

namespace seed {
namespace vis {

inline cv::Mat sift(cv::Mat sample_image, cv::Mat image){
    //
    // now, you can no more create an instance on the 'stack', like in the tutorial
    // (yea, noticed for a fix/pr).
    // you will have to use cv::Ptr all the way down:
    //
    //cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
    //cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
    //cv::Ptr<Feature2D> f2d = ORB::create();
    // you get the picture, i hope..
    

    //-- Step 1: Detect the keypoints:
    std::vector<cv::KeyPoint> sample_keypoints, keypoints;
    f2d->detect( sample_image, sample_keypoints);
    f2d->detect( image, keypoints);

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::Mat sample_descriptors, descriptors;
    f2d->compute( sample_image, sample_keypoints, sample_descriptors);
    f2d->compute( image, keypoints, descriptors);

//    //-- Step 3: Matching descriptor vectors using BFMatcher :
//    cv::BFMatcher matcher;
//    //cv::FlannBasedMatcher matcher;
//    std::vector< cv::DMatch > matches;
//    //std::vector< std::vector< cv::DMatch > > matches;
//    //matcher.knnMatch( sample_descriptors, descriptors, matches, 10 ); //find N match for each feature!
//    matcher.match( sample_descriptors, descriptors, matches );

    //-- Step 3: Matching descriptor vectors with a FLANN based matcher
    const int n_best_matches = 5;
    cv::FlannBasedMatcher matcher;
    std::vector< std::vector<cv::DMatch> > knn_matches;
    matcher.knnMatch( sample_descriptors, descriptors, knn_matches, n_best_matches );


    std::default_random_engine re;
    std::uniform_real_distribution<double> unif(0,1);

    std::vector<cv::Point2f> scene_corners(4);
    bool good = false;

    //init m_index to -1 (all are non selected)
    std::vector<int> m_index, index;
    for(int i=0; i<knn_matches.size(); i++){
        m_index.push_back(-1);
    }

    int number_of_trials = 30;

    do{
        number_of_trials--;

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = 0.7f;
        std::vector<cv::DMatch> matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            //Lowe's ratio test
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                matches.push_back(knn_matches[i][0]);
                m_index[i] = 0;
            }
            //[RIC] montecarlo step
            else{
                //do nothing for now...
                double coin = unif(re);

                //change the selection
                if( coin >= 0.4 ){
                    if( coin >= 0.7 ){ //increment
                        m_index[i] = m_index[i]+1 >= n_best_matches ? m_index[i] : m_index[i]+1;
                        matches.push_back(knn_matches[i][m_index[i]]);
                    }
                    else{ //decrement
                        m_index[i] = m_index[i]-1 < -1 ? m_index[i] : m_index[i]-1;

                        if(m_index[i]>-1)
                            matches.push_back(knn_matches[i][m_index[i]]);
                    }
                }
            }
        }

        // plot sample
        std::cout<<"[";
        for(int i=0; i<m_index.size(); i++)
            if(m_index[i]!= -1)
                std::cout<<" "<< m_index[i];
            else
                std::cout<<" -";
        std::cout<<" ]"<<std::endl;



    //    //-- Step 3.1: Find good matches :
    //    double max_dist = 0; double min_dist = 100;
    //    //-- Quick calculation of max and min distances between keypoints
    //    for( int i = 0; i < sample_descriptors.rows; i++ )
    //    {
    //        double dist = matches[i].distance;
    //        if( dist < min_dist ) min_dist = dist;
    //        if( dist > max_dist ) max_dist = dist;
    //    }
    //
    //    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //    //-- small)
    //    //-- PS.- radiusMatch can also be used here.
    //    std::vector< cv::DMatch > good_matches;
    //    for( int i = 0; i < sample_descriptors.rows; i++ )
    //    {
    //        if( matches[i].distance <= std::max(1.5*min_dist, 0.02) )
    //        {
    //            good_matches.push_back( matches[i]);
    //        }
    //    }
    //    matches = good_matches;


        std::cout<<"got matches: "<<matches.size()<<std::endl;

        cv::Mat sample_out_image, out_image;
        cv::Scalar keypointColor = cv::Scalar(255, 0, 0);     // Blue keypoints.
        cv::drawKeypoints(sample_image, sample_keypoints, sample_out_image, keypointColor, cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        cv::imshow("sift_result", sample_out_image);
        cv::waitKey(0);

        cv::drawMatches( sample_image, sample_keypoints, image, keypoints,
             matches, out_image, cv::Scalar::all(-1), cv::Scalar::all(-1),
             std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        cv::imshow("sift_result", out_image);
        cv::waitKey(0);

//        //plot metches one by one
//        for(int i=0; i<matches.size(); i++){
//
//            std::vector< cv::DMatch > matches_sup;
//            matches_sup.push_back(matches[i]);
//
//            cv::drawMatches( sample_image, sample_keypoints, image, keypoints,
//                     matches_sup, out_image, cv::Scalar::all(-1), cv::Scalar::all(-1),
//                     std::vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//
//            std::cout<<"match: "<<i<<"/"<<matches.size()<<" dist: "<<matches[i].distance<<" pos: "<<m_index[i]<<std::endl;
//            cv::imshow("sift_result", out_image);
//            cv::waitKey(0);
//        }


        //-- Step 4: Localize the object
        std::vector<cv::Point2f> obj;
        std::vector<cv::Point2f> scene;
        for( size_t i = 0; i < matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( sample_keypoints[ matches[i].queryIdx ].pt );
            scene.push_back( keypoints[ matches[i].trainIdx ].pt );
        }
        cv::Mat H = cv::findHomography( obj, scene, cv::RANSAC );
        //-- Get the corners from the image_1 ( the object to be "detected" )
        std::vector<cv::Point2f> obj_corners(4);
        obj_corners[0] = cv::Point2f(0, 0);
        obj_corners[1] = cv::Point2f( (float)sample_image.cols, 0 );
        obj_corners[2] = cv::Point2f( (float)sample_image.cols, (float)sample_image.rows );
        obj_corners[3] = cv::Point2f( 0, (float)sample_image.rows );
        //std::vector<cv::Point2f> scene_corners(4);
        cv::perspectiveTransform( obj_corners, scene_corners, H);
        //-- Draw lines between the corners (the mapped object in the scene - image_2 )
        cv::line( out_image, scene_corners[0] + cv::Point2f((float)sample_image.cols, 0),
              scene_corners[1] + cv::Point2f((float)sample_image.cols, 0), cv::Scalar(0, 100, 0), 4 );
        cv::line( out_image, scene_corners[1] + cv::Point2f((float)sample_image.cols, 0),
              scene_corners[2] + cv::Point2f((float)sample_image.cols, 0), cv::Scalar( 0, 155, 0), 4 );
        cv::line( out_image, scene_corners[2] + cv::Point2f((float)sample_image.cols, 0),
              scene_corners[3] + cv::Point2f((float)sample_image.cols, 0), cv::Scalar( 0, 200, 0), 4 );
        cv::line( out_image, scene_corners[3] + cv::Point2f((float)sample_image.cols, 0),
              scene_corners[0] + cv::Point2f((float)sample_image.cols, 0), cv::Scalar( 0, 255, 0), 4 );

        //-- Step 5: check the recognition
        double r1 =  cv::norm(scene_corners[0] - scene_corners[1]) / cv::norm(obj_corners[0] - obj_corners[1]);
        double r2 =  cv::norm(scene_corners[1] - scene_corners[2]) / cv::norm(obj_corners[1] - obj_corners[2]);
        double r3 =  cv::norm(scene_corners[2] - scene_corners[3]) / cv::norm(obj_corners[2] - obj_corners[3]);
        double r4 =  cv::norm(scene_corners[3] - scene_corners[0]) / cv::norm(obj_corners[3] - obj_corners[0]);

        double rd1 = cv::norm(scene_corners[0] - scene_corners[2]) / cv::norm(obj_corners[0] - obj_corners[2]);
        double rd2 = cv::norm(scene_corners[1] - scene_corners[3]) / cv::norm(obj_corners[1] - obj_corners[3]);

        //check if the ratio between sides is preserved (ie. omogenean transform)
        const double omo_th = 0.05;
        const double square_th = 0.05;

        if(rd1 < 0.6 || rd1 > 1.6){
            std::cout<<"scale-factor is too different: "<< rd1 <<std::endl;
            continue;
        }

        if( std::abs(rd1-rd2) < square_th ){
            std::cout<<"IS A SQUARE, square-rate: "<< std::abs(rd1-rd2) <<std::endl;
            if( std::abs(r1-r2) < omo_th && std::abs(r2-r3) < omo_th && std::abs(r3-r4) < omo_th ){
                std::cout<<"THE MATCH IS GOOD, omo-rate: "<< std::abs(r1-r2) <<" "<< std::abs(r2-r3) <<" "<< std::abs(r3-r4) <<std::endl;
                good = true;
            }
            else {
                std::cout<<"THE MATCH IS BAD, omo-rate: "<< std::abs(r1-r2) <<" "<< std::abs(r2-r3) <<" "<< std::abs(r3-r4) <<std::endl;
            }
        }
        else
            std::cout<<"IS NOT A SQUARE, square-rate: "<< std::abs(rd1-rd2) <<std::endl;


//        //check if the new sample (m_index) is good enough to be selected
//        double coin2 = unif(re);
//        if(coin2 > )

        //-- Show detected matches
        cv::imshow("sift_result", out_image );
        cv::waitKey(0);

    }while(!good || number_of_trials<=0);

    //erase the object
    cv::Point pts[4] = { scene_corners[0],
                         scene_corners[1],
                         scene_corners[2],
                         scene_corners[3] };

    cv::Mat final_image = image;

    cv::fillConvexPoly( final_image, pts, 4, cv::Scalar(1) );
    cv::imshow("sift_result", final_image );
    cv::waitKey(0);

    return final_image;
}

inline void sift(std::string path_image_to_find, std::string path_image)
{
    std::cout<<"path to image: "<<path_image_to_find<<std::endl;
    const cv::Mat sample_image = cv::imread(path_image_to_find, 0); //Load as grayscale
    const cv::Mat image = cv::imread(path_image, 0); //Load as grayscale
    cv::imshow("sift_result", sample_image);
    cv::waitKey(0);

    cv::Mat current_image = image;

    while(true){
        current_image = sift(sample_image,current_image);
    }

    return;

}


class Face{
public:
    Face(){
        count = 0;
        center = cv::Point(0,0);
        ray = 0;
        dist = 0;
    }
    Face(cv::Point center_in, double ray_in, double dist_in=0.0, int count_in=0){
        count = count_in;
        center = center_in;
        ray = ray_in;
        dist = dist_in;
    }

    int count;
    cv::Point center;
    double ray;
    double dist;
};


/** @function detectAndDisplay */
inline std::vector< Face > face_detect(cv::CascadeClassifier cascade, cv::Mat frame ) {
    std::vector<Face> face_position;
    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;

    cv::cvtColor( frame, frame_gray, CV_BGR2GRAY );
    cv::equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    for( size_t i = 0; i < faces.size(); i++ )
    {
        cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );

        face_position.push_back( Face(center, (faces[i].width+faces[i].height)/4 ) );

        cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );


    }

//    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//    image_pub.publish(msg);

    return face_position;
}

inline void haar(std::string path_image_to_find, std::string path_image)
{
    std::cout<<"path to image: "<<path_image_to_find<<std::endl;
    const cv::Mat sample_image = cv::imread(path_image_to_find, 0); //Load as grayscale
    const cv::Mat image = cv::imread(path_image, 0); //Load as grayscale
    cv::imshow("sift_result", sample_image);
    cv::waitKey(0);

    cv::String face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;

    if( !face_cascade.load( face_cascade_name ) ){ std::cout<<"--(!)Error loading"<<std::endl; return ; };

    std::vector< Face > new_faces, faces;

     //-- 3. Apply the classifier to the frame
    new_faces = face_detect( face_cascade, image );

    bool found;
    int j;

    for(u_int i=0; i<new_faces.size(); i++){
        j = 0;
        found = false;
        while(!found && j<faces.size()){
            std::cout<<cv::norm(new_faces[i].center - faces[j].center)<<std::endl;
            if(cv::norm(new_faces[i].center - faces[j].center) < 30 ){ //pixels
                found = true;
                //faces.push_back( Face(faces[j].center, faces[j].ray, 0.0, faces[j].count+1) );
                faces[j].ray = new_faces[i].ray;
                faces[j].center = new_faces[i].center;
                faces[j].count += 2;
            }
            j++;
        }
        if(!found)
            faces.push_back( Face(new_faces[i].center, new_faces[i].ray, 0.0, 2) );
        }

    cv::Mat out_image;

    cv::imshow("sift_result", out_image);
    cv::waitKey(0);

    return;

}


}
}
