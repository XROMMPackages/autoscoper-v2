#include "ui/TrackingExtendedOptionsDialog.h"
#include "ui_TrackingExtendedOptionsDialog.h"
#include "ui/AutoscoperMainWindow.h"

#include "Tracker.hpp"
#include "Trial.hpp"
#include <QTimer>
TrackingExtendedOptionsDialog::TrackingExtendedOptionsDialog(QWidget *parent):
												QDialog(parent),
												diag(new Ui::TrackingExtendedOptionsDialog){
	diag->setupUi(this);
	AutoscoperMainWindow *mainwindow  = dynamic_cast <AutoscoperMainWindow *> ( parent);

	diag->renderOptionC->hide();
	diag->frame_3->hide();
	diag->frame_7->hide();
	diag->frame_8->hide();
	diag->frame_9->hide();
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
			mainwindow->getTracker()->trial()->setRotationMode(QUATERNION);
		}else if(diag->radioButtonAxisAngle->isChecked()){
			mainwindow->getTracker()->trial()->setRotationMode(AXIS_ANGLE);
		}

		// Optimization Algorithm
		/*if (diag->downHillSimplex->isChecked()){
			mainwindow->getTracker()->trial()->setOptimizationAlgorithm(DOWNHILL_SIMPLEX);
		} else if (diag->levenbergMarquardt->isChecked()){
			mainwindow->getTracker()->trial()->setOptimizationAlgorithm(LEVENBERG_MARQUARDT);
		}*/

		// Compute Correlations
		if (diag->addCorrelations->isChecked()){
			mainwindow->getTracker()->trial()->setComp_Correlations(ADD);
		} else if (diag->multiplyCorrelations->isChecked()){
			mainwindow->getTracker()->trial()->setComp_Correlations(MULTIPLY);
		}

		if (diag->renderOptionA->isChecked()){
			mainwindow->getTracker()->setRenderMode(SEP);
		} else if (diag->renderOptionB->isChecked()){
			mainwindow->getTracker()->setRenderMode(INDV);
		} 
		/*else if (diag->renderOptionC->isChecked()){
			mainwindow->getTracker()->setRenderMode(COMB);
		}*/
		
		// set the levenberg marquardt multipliers
		/*if (diag->levmarRotateSpinBox->text().toDouble() != 0.0){
			mainwindow->getTracker()->lRotateMultiplier = diag->levmarRotateSpinBox->text().toDouble();
		}
		if (diag->levmarTransSpinBox->text().toDouble() != 0.0){
			mainwindow->getTracker()->lTransMultiplier = diag->levmarTransSpinBox->text().toDouble();
		}

		if (diag->downhillSpinBox->text().toDouble() != 0.0) {
			mainwindow->getTracker()->FTOL = diag->downhillSpinBox->text().toDouble();
		}*/

		if (diag->compute_refined_viewport->isChecked()){
			mainwindow->getTracker()->compute_refined_viewport = true;
		} else {
			mainwindow->getTracker()->compute_refined_viewport = false;
		}
		
#ifndef WITH_CMINPACK
		diag->frame_8->hide();
		diag->levenbergMarquardt->hide();
#endif

		//// set the default correlation value
		//mainwindow->getTracker()->defaultCorrelationValue = diag->correlationDefaultSpinBox->value();

		//// set bounding box division factor
		//mainwindow->getTracker()->box_division_factor = diag->bbDivisionFactor->text().toInt();

	}
	this->accept();
}

