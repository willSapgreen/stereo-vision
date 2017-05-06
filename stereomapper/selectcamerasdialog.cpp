#include "selectcamerasdialog.h"
#include "ui_selectcamerasdialog.h"
#include <iostream>

#include <QFileDialog>

using namespace std;

SelectCamerasDialog::SelectCamerasDialog(FrameCaptureThread *cam_left,FrameCaptureThread *cam_right,int shutter,CalibIO *calib,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SelectCamerasDialog),
    cam_left(cam_left),
    cam_right(cam_right),
    shutter(shutter),
    calib(calib)
{
  ui->setupUi(this);

  // query left camera
  vector<string> devices_left  = cam_left->queryDevices();
  for (unsigned int i=0; i<devices_left.size(); i++)
    ui->leftCameraComboBox->addItem(QString::fromStdString(devices_left[i]));

  // query right camera
  vector<string> devices_right = cam_right->queryDevices();
  for (unsigned int i=0; i<devices_right.size(); i++)
    ui->rightCameraComboBox->addItem(QString::fromStdString(devices_right[i]));

  if (devices_left.size()==2 && devices_right.size()==2) {
    if (!devices_left[0].compare("b09d01009e410e") || !devices_left[0].compare("b09d01007fb88f"))
      ui->rightCameraComboBox->setCurrentIndex(min(1,(int)devices_right.size()-1));
    else
      ui->leftCameraComboBox->setCurrentIndex(min(1,(int)devices_left.size()-1));
  }

  settings = new QSettings("KIT", "stereomapper");
  ui->calibFileNameEdit->setText(settings->value("calib_file_name","/home/geiger/calib.txt").toString());
}

SelectCamerasDialog::~SelectCamerasDialog()
{
  delete ui;
  delete settings;
}

void SelectCamerasDialog::on_buttonBox_accepted()
{
  // calibration file
  if (ui->rectifyCheckBox->isChecked()) {
    if (calib->readCalibFromFile(ui->calibFileNameEdit->text().toStdString()))
      calib->showCalibrationParameters();
    else {
      calib->clear();
      cout << "Calibration file not found." << endl;
    }
  } else {
    calib->clear();
    cout << "No rectification! Using raw input images ..." << endl;
  }

  // camera indices
  int cam_ind_left  = ui->leftCameraComboBox->currentIndex();
  int cam_ind_right = ui->rightCameraComboBox->currentIndex();

  // return if cameras have not be assigned
  if (cam_ind_left<0 || cam_ind_right<0) {
    cout << "At least one camera has not been assigned!" << endl;
    return;
  }

  // return if same camera for left and right image has been selected
  if (cam_ind_left==cam_ind_right) {
    cout << "Same camera ids have been selected for left and right camera!" << endl;
    return;
  }

  // set camera indices
  cam_left->setCamInd(cam_ind_left,shutter);
  cam_right->setCamInd(cam_ind_right,shutter);

  // start capturing process
  cam_left->start();
  cam_right->start();
}

void SelectCamerasDialog::on_selectFileButton_clicked() {
  QFileInfo file_info(ui->calibFileNameEdit->text());
  QString file_name = QFileDialog::getOpenFileName(this, tr("Open File"),file_info.absoluteDir().absolutePath(),tr("Calib files (*.txt)"));
  if (file_name!=0) {
    ui->calibFileNameEdit->setText(file_name);
    settings->setValue("calib_file_name",file_name);
  }
}
