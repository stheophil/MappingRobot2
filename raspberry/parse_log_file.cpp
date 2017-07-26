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

    CFastParticleSlamBase pfslam;
    SScanLine scanline;
    
    for( std::string strLine; std::getline( ifs, strLine ); ) {
        if(!strLine.empty()) {
            switch(strLine[0]) {
                case 'o':
                    SOdometryData odom;
                    if(4==sscanf(strLine.data(), "o;%*lf;%hd;%hd;%hd;%hd", 
                        &odom.m_nFrontLeft, 
                        &odom.m_nFrontRight,
						&odom.m_nBackLeft,
						&odom.m_nBackRight)) 
                    {
                        scanline.add(odom);
                    } else {
                        std::cerr << "Invalid odometry data: " << strLine << std::endl;
                    }
                    break;
                case 'l':
                {
                    std::vector<SScanLine::SScan> vecscan;
                    for(auto i = strLine.find(';', strLine.find(';') + 1);;) {
                        auto iEnd = strLine.find(';', i+1);
                        if(iEnd==std::string::npos) break;

                        int nAngle;
                        int nDistance;
                        if(2==sscanf(&strLine[i+1], "%d/%d", &nAngle, &nDistance)) {
                            vecscan.emplace_back(nAngle, nDistance);
                        } else {
                            std::cerr << "Invalid lidar data: " << &strLine[i] << std::endl;
                        }
                        i = iEnd;
                    }
                    scanline.m_vecscan = std::move(vecscan);

                    if(scanline.translation()!=rbt::size<double>::zero() || scanline.rotation()!=0.0) {
                        pfslam.receivedSensorData(scanline);
                        if(vid.isOpened()) {
                            cv::Mat matTemp;
                            cv::cvtColor(pfslam.getMapWithPoses(), matTemp, cv::COLOR_GRAY2RGB);
                            vid << matTemp;
                        }
                    }
                    scanline.clear();
                    break;
                }
                default:
                    std::cerr << "Invalid line in log file: " << strLine << std::endl;
                    return 1;
            }
        }
    }

    if(!bVideo && ostrOutput) {
        try {
            cv::imwrite(ostrOutput.get(), pfslam.getMap());
        } catch (cv::Exception& ex) {
            std::cerr << "Exception while writing to " << ostrOutput.get() << ": " << ex.what();
            return 1;
        }
    }

    // We are ignoring the rest of the last scan line
    auto const tpEnd = std::chrono::system_clock::now();
    std::chrono::duration<double> durDiff = tpEnd-tpStart;
    auto const poseFinal = pfslam.Poses().back();
    std::cout << durDiff.count() << " s\n"
        << " Final pose: ("<< poseFinal.m_pt.x  <<";"<< poseFinal.m_pt.y <<";" << poseFinal.m_fYaw << ")\n";
    
    return 0;
}
