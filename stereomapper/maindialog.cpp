#include <iostream>
#include <unistd.h>

#include "maindialog.h"
#include "ui_maindialog.h"
#include "selectcamerasdialog.h"
#include "../libviso2/src/timer.h"
using namespace std;

MainDialog::MainDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MainDialog) {

  setWindowFlags(Qt::WindowMinMaxButtonsHint);
  ui->setupUi(this);
  stereo_image = new StereoImage();
  time_of_last_frame.tv_sec  = 0;
  time_of_last_frame.tv_usec = 0;
  calib = new CalibIO();
  capture_mutex = new QMutex();
  cam_left      = new FrameCaptureThread(stereo_image,calib,true,capture_mutex);
  cam_right     = new FrameCaptureThread(stereo_image,calib,false,capture_mutex);
  vo_thread     = new VisualOdometryThread(calib);
  stereo_thread = new StereoThread(calib,ui->modelView);
  read_thread   = new ReadFromFilesThread(stereo_image,calib);
  visualize_thread = new VisualizeThread(ui->disparityView,ui->modelView);
  QObject::connect(stereo_image,SIGNAL(newStereoImageArrived()),this,SLOT(newStereoImageArrived()));
  QObject::connect(vo_thread,SIGNAL(newHomographyArrived()),this,SLOT(newHomographyArrived()));
  QObject::connect(stereo_thread,SIGNAL(newDisparityMapArrived()),this,SLOT(newDisparityMapArrived()));
  frame_number = 0;
  gain_total   = 1;
  stereo_scan  = false;
  settings = new QSettings("KIT", "stereomapper");
  QPalette p(palette());
  p.setColor(QPalette::Background, Qt::white);
  setPalette(p);

  ui->shutterSpinBox->setValue(settings->value("shutter_value","100").toInt());
  ui->modelView->setBackgroundWallFlag(ui->backgroundWallCheckBox->isChecked());
  ui->modelView->setShowCamerasFlag(ui->showCamerasCheckBox->isChecked());
  ui->modelView->setGridFlag(ui->gridCheckBox->isChecked());
  ui->modelView->setWhiteFlag(ui->whiteCheckBox->isChecked());
}

MainDialog::~MainDialog()
{
  delete ui;
  delete cam_left;
  delete cam_right;
  delete capture_mutex;
  delete stereo_image;
  delete calib;
  delete vo_thread;
  delete stereo_thread;
  delete read_thread;
  delete visualize_thread;
  delete settings;
}

void MainDialog::on_captureFromFirewireButton_clicked()
{
  SelectCamerasDialog dlg(cam_left,cam_right,ui->shutterSpinBox->value(),calib);
  dlg.exec();
  if (cam_left->isRunning()||cam_right->isRunning()) {
    ui->captureFromFirewireButton->setEnabled(false);
    ui->stopCapturingButton->setEnabled(true);
  }
}

void MainDialog::on_stopCapturingButton_clicked()
{
  cam_left->stopRecording();
  cam_right->stopRecording();
  ui->captureFromFirewireButton->setEnabled(true);
  ui->stopCapturingButton->setEnabled(false);
}

void MainDialog::on_exitButton_clicked() {
  exit(0);
}

string MainDialog::createNewOutputDirectory() {
  char buffer[1024];
  for (int32_t i=0; i<9999; i++) {
    sprintf(buffer,"output_%04d/I1_000000.png",i);
    FILE *file = fopen (buffer,"r");
    if (file!=NULL) {
      fclose (file);
      continue;
    }
    sprintf(buffer,"output_%04d/",i);
    break;
  }
  cout << "Creating output directory: " << buffer << endl;
  char cmd[1024];
  sprintf(cmd,"mkdir %s",buffer);
  system(cmd);
  return buffer;
}

void MainDialog::on_stereoScanButton_clicked() {

  // stopping ... (wait for termination of visual odometry and stereo thread)
  if (stereo_scan) {

    // set button text
    ui->stereoScanButton->setText("Scan!");

    // stop stereo scanning
    stereo_scan = false;

    // terminate all processes
    vo_thread->terminate();
    stereo_thread->terminate();
    read_thread->terminate();
    while (vo_thread->isRunning() || stereo_thread->isRunning() || read_thread->isRunning());

  // starting ...
  } else {

    // set button text
    ui->stereoScanButton->setText("Stop!");

    // reset everything
    vo_thread->resetHomographyTotal();
    frame_number = 0;
    gain_total   = 1;
    ui->modelView->clearAll();
    stereo_thread->clearReconstruction();

    // create output dir
    if (ui->saveToFilesCheckBox->isChecked())
      output_dir = createNewOutputDirectory();

    // start reading from files
    if (ui->readFromFilesCheckBox->isChecked()) {
      if (read_thread->isRunning())
        read_thread->terminate();
      QString input_dir = QFileDialog::getExistingDirectory (this,tr("Open Directory"),settings->value("input_dir_name","/home/geiger").toString(),QFileDialog::ShowDirsOnly);
      settings->setValue("input_dir_name",input_dir);
      read_thread->setInputDir(input_dir);
      read_thread->start();
    }

    // start stereo scanning
    stereo_scan = true;
  }
}

void MainDialog::keyPressEvent(QKeyEvent * event) {
  if (event->key()==Qt::Key_S) {
    save_single_frame = false;
    on_stereoScanButton_clicked();
  }
  if (event->key()==Qt::Key_F) {
    save_single_frame = true;
    on_stereoScanButton_clicked();
  }
}

void MainDialog::on_readFromFilesCheckBox_clicked() {
  if (ui->readFromFilesCheckBox->isChecked()) {
    frame_number = 0;
  }
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

// this is the main loop!
void MainDialog::newStereoImageArrived(){

  // get stereo image deepcopy
  StereoImage::simage simg(*stereo_image->getStereoImage());
  stereo_image->pickedUp();

  // save images
  if (stereo_scan && (ui->saveToFilesCheckBox->isChecked())) {
    if (frame_number!=0)
      cout << stereo_image->timeDiff(simg.time,last_frame_time) << endl;
    last_frame_time = simg.time;
    SaveStereoImageThread* save_thread = new SaveStereoImageThread(simg,output_dir,frame_number++);
    save_thread->start();
    save_stereo_threads.push_back(save_thread);

    // stop capturing if we requested only a single frame!
    if (save_single_frame)
      on_stereoScanButton_clicked();
  }

  // check if any of the save threads have finished and must be deleted
  vector<SaveStereoImageThread*> running_save_stereo_threads;
  for (vector<SaveStereoImageThread*>::iterator it=save_stereo_threads.begin(); it!=save_stereo_threads.end(); it++) {
    if ((*it)->isRunning())
      running_save_stereo_threads.push_back(*it);
    else
      delete *it;
  }
  save_stereo_threads = running_save_stereo_threads;

  // reconstruction mode
  if (ui->tabWidget->currentIndex()==0) {

    // show image
    if (!stereo_scan) {
      ui->leftImageView->setImage(simg.I1,simg.width,simg.height);
      ui->rightImageView->setImage(simg.I2,simg.width,simg.height);
    }

    // start quad matching if idle
    if (stereo_scan && simg.rectified && !vo_thread->isRunning()) {
      QPalette p(palette());
      if (ui->saveToFilesCheckBox->isChecked()) p.setColor(QPalette::Background, Qt::red);
      else                                      p.setColor(QPalette::Background, Qt::green);
      setPalette(p);
      vo_thread->pushBack(simg,ui->recordRawOdometryCheckBox->isChecked());
      vo_thread->start();
    }

  // elas mode
  } else if (ui->tabWidget->currentIndex()==1) {
    ui->elasImageView->setImage(simg.I1,simg.width,simg.height);
    if (stereo_scan && !stereo_thread->isRunning() && !ui->saveToFilesCheckBox->isChecked()) {
      stereo_thread->pushBack(simg,ui->subsamplingCheckBox->isChecked());
      stereo_thread->start();
    }

  // left full image
  } else if (ui->tabWidget->currentIndex()==2) {
    ui->leftFullImageView->setImage(simg.I1,simg.width,simg.height);

  // right full image
  } else if (ui->tabWidget->currentIndex()==3) {
    ui->rightFullImageView->setImage(simg.I2,simg.width,simg.height);
  }
}

void MainDialog::newHomographyArrived() {

  // get stereo image deepcopy
  StereoImage::simage simg(*vo_thread->getStereoImage());
  Matrix H_total = vo_thread->getHomographyTotal();
  gain_total *= vo_thread->getGain();
  vo_thread->pickedUp();

  // show quad match
  ui->leftImageView->setImage(simg.I1,simg.width,simg.height);
  ui->rightImageView->setImage(simg.I2,simg.width,simg.height);

  // show images
  ui->leftImageView->setMatches(vo_thread->getMatches(),vo_thread->getInliers(),true);
  ui->rightImageView->setMatches(vo_thread->getMatches(),vo_thread->getInliers(),false);  

  // start dense stereo matching if idle
  if (stereo_scan && !stereo_thread->isRunning() && !ui->saveToFilesCheckBox->isChecked()) {
    stereo_thread->pushBack(simg,H_total,gain_total);
    stereo_thread->start();
    gain_total = 1;
    //ui->modelView->addCamera(H_total,0.08,true);
  } else {
    //ui->modelView->addCamera(H_total,0.05,false);
  }
}

void MainDialog::newDisparityMapArrived() {

  // get stereo image deepcopy
  StereoImage::simage simg(*stereo_thread->getStereoImage());

  if (ui->tabWidget->currentIndex()==0)
    ui->disparityView->setColorImage(stereo_thread->getColorDisparityMap(),simg.width,simg.height);
  else {
    if (!ui->subsamplingCheckBox->isChecked())
      ui->elasDisparityView->setColorImage(stereo_thread->getColorDisparityMap(),simg.width,simg.height);
    else
      ui->elasDisparityView->setColorImage(stereo_thread->getColorDisparityMap(),simg.width/2,simg.height/2);
  }

  if (stereo_scan && ui->tabWidget->currentIndex()==0) {
    ui->modelView->addCamera(stereo_thread->getHomographyTotal(),0.1,true);
    ui->modelView->addPoints(stereo_thread->getPoints());
  }

  stereo_thread->pickedUp();
}


void MainDialog::on_resetBusButton_clicked() {

  cam_left->resetBus();
  cam_right->resetBus();

  // show successfull reset message:
  cout << endl;
  cout << "=============================================" << endl;
  cout << "The firewire bus has been reset successfully!" << endl;
  cout << "=============================================" << endll;
}

void MainDialog::on_backgroundWallSlider_sliderMoved(int position) {
  ui->modelView->setBackgroundWallPosition((float)position/100.0);
}

void MainDialog::on_backgroundWallCheckBox_clicked() {
  ui->modelView->setBackgroundWallFlag(ui->backgroundWallCheckBox->isChecked());
}

void MainDialog::on_showCamerasCheckBox_clicked(){
  ui->modelView->setShowCamerasFlag(ui->showCamerasCheckBox->isChecked());
}

void MainDialog::on_gridCheckBox_clicked() {
  ui->modelView->setGridFlag(ui->gridCheckBox->isChecked());
}

void MainDialog::on_whiteCheckBox_clicked() {
  ui->modelView->setWhiteFlag(ui->whiteCheckBox->isChecked());
}

void MainDialog::on_addPosePushButton_clicked() {
  ui->modelView->addPose();
}

void MainDialog::on_deletePosePushButton_clicked() {
  ui->modelView->delPose();
}

void MainDialog::on_playPosesPushButton_clicked() {
  ui->modelView->playPoses(false);
}

void MainDialog::on_recordPosesPushButton_clicked() {
  ui->modelView->playPoses(true);
}

void MainDialog::on_resizePushButton_clicked() {
  ui->modelView->setMinimumWidth(800);
  ui->modelView->setMinimumHeight(450);
}


void MainDialog::on_resizeSmallPushButton_clicked() {

  int minw = ui->modelView->minimumWidth();
  int minh = ui->modelView->minimumHeight();
  int maxw = ui->modelView->maximumWidth();
  int maxh = ui->modelView->maximumHeight();

  ui->modelView->setMinimumWidth(320);
  ui->modelView->setMinimumHeight(480);
  ui->modelView->setMaximumWidth(320);
  ui->modelView->setMaximumHeight(480);
  ui->modelView->setShowCamerasFlag(0);
  ui->modelView->setGridFlag(0);

  ui->modelView->recordHuman();

  ui->modelView->setMinimumWidth (minw);
  ui->modelView->setMinimumHeight(minh);
  ui->modelView->setMaximumWidth (maxw);
  ui->modelView->setMaximumHeight(maxh);
  ui->modelView->setShowCamerasFlag(ui->showCamerasCheckBox->isChecked());
  ui->modelView->setGridFlag(ui->gridCheckBox->isChecked());
}

void MainDialog::on_shutterSpinBox_valueChanged(int ) {
  settings->setValue("shutter_value",ui->shutterSpinBox->value());
}
