#ifndef MSCKF_UTILS_RECORDER_H_
#define MSCKF_UTILS_RECORDER_H_

#include <fstream>
#include <iostream>
#include <vector>

#include "feature/Feature.h"

namespace msckf_dvio {

class Recorder {
public:
    Recorder(std::string filename) {
        // check if the file exist

        // check the file
        outfile.open(filename,std::ios_base::app);
        if (outfile.fail()) {
          printf("Unable to open output file!!");
          printf("Path: %s", filename.c_str());
          std::exit(EXIT_FAILURE);
        }

        init("id, timestamp");
    }

    void init(std::string header) {
        // wirte header

        // outfile << "id timestamps" << std::endl;
        outfile << header << std::endl;
    }

    void writeFeature(Feature &feat) {
        outfile << feat.featid << "," << std::fixed << std::setprecision(9);

        for(const auto& t :feat.timestamps[0])
          outfile<< t << ",";

        outfile <<"\n";
    }

    void writeString(std::string str) {
        outfile << str;
    }

protected:
    // Output stream file
    std::ofstream outfile;
};

}


#endif  //MSCKF_UTILS_RECORDER_H_