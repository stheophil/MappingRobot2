#include "error_handling.h"
#include "rover.h"
#include "robot_configuration.h"
#include "fast_particle_slam.h"
#include "path_finding.h"

#include <stdio.h>
#include <chrono>
#include <fstream>
#include  <clocale>

#include <boost/range/adaptor/transformed.hpp>
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
    
    SOdometryData odomPrev = {0};
    double fSecondsPrev = 0;
    double fSpeedPrevLeft = 0;
    double fSpeedPrevRight = 0;

    auto Speed = [&](SOdometryData const& odom, double fSeconds, bool bLeft) { // in m/s
        auto const nTicks = (bLeft 
            ? (odom.m_nFrontLeft + odom.m_nBackLeft)
            : (odom.m_nFrontRight + odom.m_nBackRight)) / 2;
        return encoderTicksToCm(nTicks) / 100.0 / (fSeconds - fSecondsPrev); 
    };

    auto Acceleration = [&](SOdometryData const& odom, double fSeconds, bool bLeft) {
        return (Speed(odom, fSeconds, bLeft) - (bLeft ? fSpeedPrevLeft : fSpeedPrevRight)) / (fSeconds - fSecondsPrev);
    };

    rbt::interval<double> intvlfAcceleration = rbt::interval<double>::empty();
    rbt::interval<double> intvlfSpeed = rbt::interval<double>::empty();

    std::setlocale(LC_ALL, "en_US.utf8");
    for( std::string strLine; std::getline( ifs, strLine ); ) {
        if(!strLine.empty()) {
            switch(strLine[0]) {
                case 'o':
                    SOdometryData odom;
                    double fSeconds;
                    if(5==sscanf(strLine.data(), "o;%lf;%hd;%hd;%hd;%hd", 
                        &fSeconds,
                        &odom.m_nFrontLeft, 
                        &odom.m_nFrontRight,
						&odom.m_nBackLeft,
						&odom.m_nBackRight)) 
                    {
                        scanline.add(odom);

                        intvlfAcceleration |= Acceleration(odom, fSeconds, /*bLeft*/ true);
                        intvlfAcceleration |= Acceleration(odom, fSeconds, /*bLeft*/ false);

                        intvlfSpeed |= Speed(odom, fSeconds, /*bLeft*/ true);
                        intvlfSpeed |= Speed(odom, fSeconds, /*bLeft*/ false);

                        odomPrev = odom;
                        fSpeedPrevLeft = Speed(odom, fSeconds, /*bLeft*/ true);
                        fSpeedPrevRight = Speed(odom, fSeconds, /*bLeft*/ false);
                        fSecondsPrev = fSeconds;
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
                            vecscan.emplace_back(nAngle < 180 ? nAngle + 180 : nAngle - 180, nDistance);
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
            cv::imwrite(ostrOutput.get() + ".png", pfslam.getMap());
        } catch (cv::Exception& ex) {
            std::cerr << "Exception while writing to " << ostrOutput.get() << ": " << ex.what();
            return 1;
        }
    }

    // We are ignoring the rest of the last scan line
    auto const tpEnd = std::chrono::system_clock::now();
    std::chrono::duration<double> const durDiff = tpEnd-tpStart;
    auto const poseFinal = pfslam.Poses().back();
    std::cout << durDiff.count() << " s\n"
        << " Final pose: ("<< poseFinal.m_pt.x  <<";"<< poseFinal.m_pt.y <<";" << poseFinal.m_fYaw << ")\n";
    
    std::cout << " Min/Max Speed = ( " << intvlfSpeed.begin << " m/s, " << intvlfSpeed.end << " m/s)" << std::endl;
    std::cout << " Min/Max Accel = ( " << intvlfAcceleration.begin << " m/s^2, " << intvlfAcceleration.end << " m/s^2)" << std::endl;

    std::cout << " Finding path back to (0;0) \n";

    {
        auto const tpStart = std::chrono::system_clock::now();
        auto const vecptf = FindPath(pfslam.getMap(), poseFinal.m_pt, rbt::point<double>::zero());
        auto const tpEnd = std::chrono::system_clock::now();
    
        std::chrono::duration<double> const durDiff = tpEnd-tpStart;
        std::cout << " Path finding took " << durDiff.count() << "s\n";
    
        if(ostrOutput) {
            cv::imwrite(
                ostrOutput.get() + "_astar.png", 
                ObstacleMapWithPoses(
                    pfslam.getMap(), 
                    boost::copy_range<std::vector<rbt::pose<double>>>(
                        boost::adaptors::transform(vecptf, [](rbt::point<double> const& ptf) { return rbt::pose<double>(ptf, 0); })
                    )
                )
            );
        }
    }
    return 0;
}
