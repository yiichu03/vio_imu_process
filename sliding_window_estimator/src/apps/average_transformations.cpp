#include <fstream>
#include <iomanip>
#include <iostream>

#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <swift_vio/YamlHelpers.h>

#include <sophus/se3.hpp>
#include <swift_vio/memory.h>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;

using std::string, std::cout, std::endl;

const size_t N_elem = 16;

Sophus::SE3d getCameraExtrinsic(const std::string &yamlfile, int cameraid) {
  Sophus::SE3d T_cam_imu;
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  // read lines from yaml
  // locate the lines for T_cam_imu
  // and put in T_cam_imu
  string line;
  std::ifstream file{yamlfile};

  if (!file) {
    std::cout << "Cannot read file " << yamlfile << "\n";
  }

  size_t count = 0, mark = 0;
  while (getline(file, line)) {
    if (std::strstr(line.c_str(),
                    ("cam" + std::to_string(cameraid) + ":").c_str())) {
      mark = 1;
    }
    if (mark == 1) {
      count++;
      if (count > 2 && count <= 2 + N_elem) {
        for (int i = line.size() - 1; i >= 0; i--) {
          if (line[i] == '-' && line[i + 1] == ' ') {
            std::string stemp = line.substr(i + 2);
            T((count - 3) / 4, (count - 3) % 4) = std::atof(stemp.c_str());
            break;
          }
        }
      }
      if (count == 18) {
        mark = 0;
        break;
      }
    }
  }
  T_cam_imu = Sophus::SE3d(T.topLeftCorner<3, 3>(), T.topRightCorner<3, 1>());
  file.close();
  return T_cam_imu;
}

Sophus::SE3d
averageTransforms(const Eigen::AlignedVector<Sophus::SE3d> &transformList) {
  Eigen::Matrix<double, 6, 1> sumtangents = Eigen::Matrix<double, 6, 1>::Zero();
  Sophus::SE3d invfirst = transformList.front().inverse();
  for (size_t j = 1u; j < transformList.size(); ++j) {
    Eigen::Matrix<double, 6, 1> tangent =
        Sophus::SE3d::log(transformList[j] * invfirst);
    sumtangents += tangent;
  }
  return Sophus::SE3d::exp(sumtangents / transformList.size()) *
         transformList[0];
}

void copyfile(const std::string &infile, const std::string &outfile) {
  string line;
  std::ifstream instream{infile};
  std::ofstream outstream{outfile};

  if (instream && outstream) {
    while (getline(instream, line)) {
      outstream << line << "\n";
    }
  } else {
    std::cerr << "Cannot read file " << infile << "\n";
  }
  instream.close();
  outstream.close();
}

std::vector<std::string> splitstring(std::string text,
                                     std::string delimiter = " ") {

  std::vector<string> words{};

  size_t pos = 0;
  while ((pos = text.find(delimiter)) != string::npos) {
    words.push_back(text.substr(0, pos));
    text.erase(0, pos + delimiter.length());
  }
  if (text.size()) {
    words.push_back(text);
  }
  return words;
}

void saveCameraExtrinsic(const Sophus::SE3d &T_cam_imu, int cameraid,
                         const std::string &yamlfile) {
  // read lines from yamlfile,
  // locate the T_cam_imu lines,
  // and replace them with lines created from T_cam_imu
  std::ifstream file{yamlfile};
  if (!file) {
    std::cerr << "Unable to open " << yamlfile << ". Abort saving camera extrinsics!\n";
    return;
  }
  std::vector<std::string> linestosave;
  linestosave.reserve(100);

  string line;
  size_t count = 0, mark = 0;

  while (getline(file, line)) {
    // judge the line just read is the line to process
    if (strstr(line.c_str(),
               ("cam" + std::to_string(cameraid) + ":").c_str())) {
      mark = 1;
    }
    if (mark == 1) {
      count++;
      if (count > 2 && count <= 2 + N_elem) {
        for (int i = line.size() - 1; i >= 0; i--) {
          if (line[i] == '-' && line[i + 1] == ' ') {
            std::string stemp = line.substr(0, i + 1);
            std::ostringstream out;
            out << std::setprecision(18)
                << T_cam_imu.matrix()((count - 3) / 4, (count - 3) % 4);
            line = stemp + " " + out.str();
            break;
          }
        }
      }
      if (count == 18) {
        mark = 0;
      }
    }
    linestosave.emplace_back(line);
  }
  file.close();

  std::ofstream stream(yamlfile, std::ios::out | std::ios::trunc);
  for (const auto& line : linestosave) {
    stream << line << std::endl;
  }
  stream.close();
  std::cout << "Saved camera extrinsics to " << yamlfile << "\n";
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::cout << "Given a directory containing many camchain-imucam**.yaml "
                 "files resulting from kalibr_calibrate_imu_camera, "
                 "read from these yamls T_cam_imu, take their average, and "
                 "save to a yaml in the same format under the directory."
              << std::endl;
    std::cout << "Usage: " << argv[0]
              << " <folder> <name-signatures, e.g., "
                 "calibrated,camchain-imucam> <camera-index, e.g., 0> <outputyaml>\n"
                 "name-signature should be in the path of the "
                 "camchain-imucam**.yaml, is used to filter results. \n"
                 "Camera-index is which camera extrinsics to work with, "
                 "usually should be 0."
              << std::endl;
    exit(1);
  }

  std::string folder = argv[1];
  std::string signaturestrings = argv[2];
  std::vector<std::string> signaturelist = splitstring(signaturestrings, ",");
  std::string outputyaml = argv[4];

  cout << "Signature list:\n";
  int j = 0;
  for (auto sig : signaturelist) {
    cout << j << " " << sig << "\n";
    ++j;
  }

  int cameraid = std::atoi(argv[3]);

  std::vector<std::string> yamllist;
  yamllist.reserve(16);

  recursive_directory_iterator rdi(folder);
  recursive_directory_iterator end_rdi;

  string ext_str0(".yaml");
  for (; rdi != end_rdi; rdi++) {
    if (ext_str0.compare((*rdi).path().extension().string()) == 0) {
      bool allfound = true;
      for (auto sig : signaturelist) {
        allfound =
            allfound && (*rdi).path().string().find(sig) != std::string::npos;
      }
      if (allfound)
        yamllist.push_back((*rdi).path().string());
    }
  }
  std::sort(yamllist.begin(), yamllist.end());
  cout << "Found yaml files:\n";
  int i = 0;
  for (auto yaml : yamllist) {
    cout << i << " " << yaml << "\n";
    ++i;
  }
  if (yamllist.size() == 0) {
    cout << "Unable to locate any yaml files under " << folder
         << ". Double check folder path and search keys!\n";
    exit(2);
  }

  Eigen::AlignedVector<Sophus::SE3d> T_cam_imu_list;
  for (auto fn : yamllist) {
    T_cam_imu_list.emplace_back(getCameraExtrinsic(fn, cameraid));
  }

  Sophus::SE3d mean_T_cam_imu = averageTransforms(T_cam_imu_list);

  // copy one file as template
  copyfile(yamllist[0], outputyaml);

  saveCameraExtrinsic(mean_T_cam_imu, cameraid, outputyaml);

  return 0;
}
