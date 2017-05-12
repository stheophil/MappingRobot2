#include "error_handling.h"
#include "rover.h"
#include "robot_configuration.h"
#include "fast_particle_slam.h"

#include <stdio.h>
#include <chrono>
#include <fstream>

#include <boost/optional.hpp>

#include <opencv2/imgcodecs/imgcodecs.hpp>     // cv::imread()
#include <opencv2/opencv.hpp>

int ParseLogFile(std::ifstream& ifs, bool bVideo, boost::optional<std::string> const& ostrOutput) {
    cv::VideoWriter vid;
    
    if(bVideo && ostrOutput) {
        vid.open(
            ostrOutput.get() + ".mov",
            cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 
            5, 
            cv::Size(c_nMapExtent, c_nMapExtent)
        );
    }
    
    auto const tpStart = std::chrono::system_clock::now();

    CFastParticleSlamBase rbt;
    SScanLine scanline;

    for( std::string strLine; std::getline( ifs, strLine ); ) {
    // TODO: Read odometry / lidar packages into scanline
    //     if(rbt.receivedSensorData(data) && vid.isOpened()) {
	// 			cv::Mat matTemp;
	// 			cv::cvtColor(rbt.getMapWithPoses(), matTemp, cv::COLOR_GRAY2RGB);
	// 			vid << matTemp;
    //     }
    }

    if(!bVideo && ostrOutput) {
        try {
            cv::imwrite(ostrOutput.get(), rbt.getMap());
        } catch (cv::Exception& ex) {
            std::cerr << "Exception while writing to " << ostrOutput.get() << ": " << ex.what();
            return 1;
        }
    }

    // We are ignoring the rest of the last scan line
    auto const tpEnd = std::chrono::system_clock::now();
    std::chrono::duration<double> durDiff = tpEnd-tpStart;
    auto const poseFinal = rbt.Poses().back();
    std::cout << durDiff.count() << " s\n"
        << " Final pose: ("<< poseFinal.m_pt.x  <<";"<< poseFinal.m_pt.y <<";" << poseFinal.m_fYaw << ")\n";
    
    return 0;
}
