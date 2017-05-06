#include "calibio.h"

#include <sstream>
using namespace std;

CalibIO::CalibIO () {
  c_dist = 0;
  roi    = 0;
  K1     = 0;
  D1     = 0;
  R1     = 0;
  K2     = 0;
  D2     = 0;
  R2     = 0;
  R      = 0;
  T      = 0;
  P1     = 0;
  P1_roi = 0;
  P2     = 0;
  P2_roi = 0;
}

CalibIO::~CalibIO () {
  clear();
}

void CalibIO::clear() {
  if (c_dist!=0) cvReleaseMat(&c_dist);
  if (roi!=0)    cvReleaseMat(&roi);
  if (K1!=0)     cvReleaseMat(&K1);
  if (D1!=0)     cvReleaseMat(&D1);
  if (R1!=0)     cvReleaseMat(&R1);
  if (K2!=0)     cvReleaseMat(&K2);
  if (D2!=0)     cvReleaseMat(&D2);
  if (R2!=0)     cvReleaseMat(&R2);
  if (R!=0)      cvReleaseMat(&R);
  if (T!=0)      cvReleaseMat(&T);
  if (P1!=0)     cvReleaseMat(&P1);
  if (P1_roi!=0) cvReleaseMat(&P1_roi);
  if (P2!=0)     cvReleaseMat(&P2);
  if (P2_roi!=0) cvReleaseMat(&P2_roi);
  c_dist = 0;
  roi    = 0;
  K1     = 0;
  D1     = 0;
  R1     = 0;
  K2     = 0;
  D2     = 0;
  R2     = 0;
  R      = 0;
  T      = 0;
  P1     = 0;
  P1_roi = 0;
  P2     = 0;
  P2_roi = 0;
}

// read calibration text file
bool CalibIO::readCalibFromFile (string calib_file_name) {

  // open calibration file
  FILE *calib_file = fopen (calib_file_name.c_str(),"r");
  if (calib_file==NULL)
    return false;

  bool success = true;
  time   = readCalibFileString(calib_file,"calib_time:",success);
  c_dist = readCalibFileMatrix(calib_file,"corner_dist:",1,1,success);
  roi    = readCalibFileMatrix(calib_file,"roi:",1,4,success);
  K1     = readCalibFileMatrix(calib_file,"K1:",3,3,success);
  D1     = readCalibFileMatrix(calib_file,"D1:",1,5,success);
  R1     = readCalibFileMatrix(calib_file,"R1:",3,3,success);
  K2     = readCalibFileMatrix(calib_file,"K2:",3,3,success);
  D2     = readCalibFileMatrix(calib_file,"D2:",1,5,success);
  R2     = readCalibFileMatrix(calib_file,"R2:",3,3,success);
  R      = readCalibFileMatrix(calib_file,"R:",3,3,success);
  T      = readCalibFileMatrix(calib_file,"T:",3,1,success);
  P1     = readCalibFileMatrix(calib_file,"P1:",3,4,success);
  P1_roi = readCalibFileMatrix(calib_file,"P1_roi:",3,4,success);
  P2     = readCalibFileMatrix(calib_file,"P2:",3,4,success);
  P2_roi = readCalibFileMatrix(calib_file,"P2_roi:",3,4,success);

  // close file and return success
  fclose (calib_file);
  return success;
}

// displays all calibration matrices
void CalibIO::showCalibrationParameters() {
  cout << endl << "========================" << endl;
  cout << "Calibration parameters:";
  cout << endl << "========================" << endl << endl;
  cout << "Calibration time: ";
  for (uint32_t i=0; i<time.size(); i++)
    cout << time[i] << " ";
  cout << endl << endl;
  showCvMat(c_dist,"c_dist");
  showCvMat(roi,"roi");
  showCvMat(K1,"K1");
  showCvMat(D1,"D1");
  showCvMat(R1,"R1");
  showCvMat(K2,"K2");
  showCvMat(D2,"D2");
  showCvMat(R2,"R2");
  showCvMat(R,"R");
  showCvMat(T,"T");
  showCvMat(P1,"P1");
  showCvMat(P1_roi,"P1_roi");
  showCvMat(P2,"P2");
  showCvMat(P2_roi,"P2_roi");
}

// displays an OpenCV matrix
void CalibIO::showCvMat(CvMat *m,string desc) {
  if (desc.compare(""))
    cout << "Matrix \"" << desc << "\":" << endl;
  if (m!=0) {
    CvSize s = cvGetSize(m);
    for (int32_t i=0; i<s.height; i++) {
      for (int32_t j=0; j<s.width; j++) {
         cout << setw(10) << cvmGet(m,i,j) << " ";
      }
      cout << endl;
    }
  } else {
    cout << " --- undefined --- " << endl;
  }
  cout << endl;
}

uint32_t CalibIO::width () {
  if (roi!=0)
    return (uint32_t)(cvmGet(roi,0,1)-cvmGet(roi,0,0)+1);
  else
    return -1;
}

uint32_t CalibIO::height () {
  if (roi!=0)
    return (uint32_t)(cvmGet(roi,0,3)-cvmGet(roi,0,2)+1);
  else
    return -1;
}

/////////////////////////////////////////////////////////////////////////////////
///////////////////////////// PRIVATE FUNCTIONS /////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

vector<string> CalibIO::splitLine (string line) {
  vector<string> line_vector;
  if (line[0]=='\0' || line[1]=='\0')
    return line_vector;
  uint32_t k1=0,k2=0;
  do {
    k2++;
    if (k2>k1 && (line[k2]==' ' || line[k2]=='\n' || line[k2]=='\0' || line[k2]=='\t' || line[k2]==',' || line[k2]==';') ){
      char line_vector_curr[10000];
      for (uint32_t i=k1; i<k2; i++)
        line_vector_curr[i-k1] = line[i];
      line_vector_curr[k2-k1] = '\0';
      //cout << line_vector_curr << "<-" << endl;
      line_vector.push_back(line_vector_curr);
      k1=k2+1;
    }
  } while (line[k2]!= '\0' && line[k2]!='\n' && k2<line.size());
  return line_vector;
}

vector<string> CalibIO::readCalibFileString (FILE *calib_file,const char *string_name,bool &success) {

  // init result
  vector<string> words;

  // go to beginning of file
  rewind(calib_file);

  // buffer for lines of file
  char line[20000];

  // for all lines of calib file do
  while(calib_file!=NULL && fgets(line,sizeof(line),calib_file)!=NULL) {

    // split current line into elements
    vector<string> line_vector =  splitLine(line);

    // if matrix_name is first passage of this line
    if (!line_vector[0].compare(string_name)) {
      for (uint32_t i=1; i<line_vector.size(); i++)
        words.push_back(line_vector[i]);
      return words;
    }
  }

  success = false;
  return words;
}

CvMat* CalibIO::readCalibFileMatrix (FILE *calib_file,const char *matrix_name,uint32_t m,uint32_t n,bool &success) {

  // go to beginning of file
  rewind(calib_file);

  // buffer for lines of file
  char line[20000];

  // for all lines of calib file do
  while(calib_file!=NULL && fgets(line,sizeof(line),calib_file)!=NULL) {

    // split current line into elements
    vector<string> line_vector =  splitLine(line);

    // if matrix_name is first passage of this line
    if (!line_vector[0].compare(matrix_name)) {

      // check for right numer of matrix elements
      if (line_vector.size()-1!=m*n) {
        cout << "ERROR Number of elements in " << matrix_name << ": " << line_vector.size()-1 << "!=" << m*n << endl;
        success = false;
        return NULL;
      }

      // create and fill opencv matrix
      CvMat* M = cvCreateMat(m,n,CV_32FC1);
      float val;

      uint32_t k=1;
      for (uint32_t i=0; i<m; i++) {
        for (uint32_t j=0; j<n; j++) {
          stringstream sst;
          sst<<line_vector[k++];
          sst>>val;
          cvmSet(M,i,j,val);
        }
      }
      return M;
    }
  }

  success = false;
  return NULL;
}



