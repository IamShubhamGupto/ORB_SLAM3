#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp> // VideoCapture

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_webcam path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Create a VideoCapture object to capture video from the default camera
    cv::VideoCapture cap(0);

    // Check if the camera opened successfully
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera" << std::endl;
        return -1;
    }

    cv::Mat frame;
    double timestamp;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true, 0, file_name);
    
    float imageScale = SLAM.GetImageScale();
    int main_error = 0;
    std::thread runthread([&](){
         while (!SLAM.isShutDown()) {
            // Capture a new frame
            cap >> frame;

            // Check if the frame is empty
            if (frame.empty()) {
                std::cerr << "Error: Empty frame" << std::endl;
                main_error=1;
                break;
            }

            timestamp = (double)cv::getTickCount() / cv::getTickFrequency();

            if(imageScale != 1.f) {
                int width = frame.cols * imageScale;
                int height = frame.rows * imageScale;
                cv::resize(frame, frame, cv::Size(width, height));
            }

            // Track the frame
            SLAM.TrackMonocular(frame, timestamp);
        }

    });

    cout << "System shutdown!\n";
    SLAM.StartViewer();
    cout << "Viewer started, waiting for thread." << endl;
    runthread.join();
    if (main_error != 0)
        return main_error;
    cout << "Tracking thread joined..." << endl;

    // Release the VideoCapture object
    cap.release();
    SLAM.Shutdown();

    return 0;
}
