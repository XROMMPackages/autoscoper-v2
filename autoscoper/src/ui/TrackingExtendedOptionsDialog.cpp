#include "ui/TrackingExtendedOptionsDialog.h"
#include "ui_TrackingExtendedOptionsDialog.h"
#include "ui/AutoscoperMainWindow.h"

#include "Tracker.hpp"
#include "Trial.hpp"
#include <QTimer>
TrackingExtendedOptionsDialog::TrackingExtendedOptionsDialog(QWidget *parent) :
												QDialog(parent),
												diag(new Ui::TrackingExtendedOptionsDialog){
	diag->setupUi(this);
}

TrackingExtendedOptionsDialog::~TrackingExtendedOptionsDialog(){
	delete diag;
}

void TrackingExtendedOptionsDialog::on_pushButtonApply_clicked(bool checked){	
	AutoscoperMainWindow *mainwindow  = dynamic_cast <AutoscoperMainWindow *> ( parent());
	Trial * tr = NULL;
	if(mainwindow){
		
		// Rotation Represenation
		if(diag->radioButtonYPR->isChecked()){
			mainwindow->getTracker()->trial()->setRotationMode(EULER_ANGLE);
		}else if(diag->radioButtonQuat->isChecked()){
			fprintf(stderr, "quaternion is selected");
			mainwindow->getTracker()->trial()->setRotationMode(QUATERNION);
		}else if(diag->radioButtonAxisAngle->isChecked()){
			mainwindow->getTracker()->trial()->setRotationMode(AXIS_ANGLE);
		}

		// Optimization Algorithm
		if (diag->downHillSimplex->isChecked()){
			mainwindow->getTracker()->trial()->setOptimizationAlgorithm(DOWNHILL_SIMPLEX);
		} else if (diag->levenbergMarquardt->isChecked()){
			mainwindow->getTracker()->trial()->setOptimizationAlgorithm(LEVENBERG_MARQUARDT);
		}

		// Compute Correlations
		if (diag->addCorrelations->isChecked()){
			mainwindow->getTracker()->trial()->setComp_Correlations(ADD);
		} else if (diag->multiplyCorrelations->isChecked()){
			mainwindow->getTracker()->trial()->setComp_Correlations(MULTIPLY);
		}

		if (diag->renderOptionA->isChecked()){
			mainwindow->getTracker()->setRenderMode(A);
		} else if (diag->renderOptionB->isChecked()){
			mainwindow->getTracker()->setRenderMode(B);
		} else if (diag->renderOptionC->isChecked()){
			mainwindow->getTracker()->setRenderMode(C);
		}
		// Render Modes

	}
	this->accept();
}

