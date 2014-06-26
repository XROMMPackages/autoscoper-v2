#ifndef TRACKINGEXTENDEDOPTIONSDIALOG_H_
#define TRACKINGEXTENDEDOPTIONSDIALOG_H_

#include <QDialog>

namespace Ui {
	class TrackingExtendedOptionsDialog;
}

class TrackingExtendedOptionsDialog : public QDialog{

	Q_OBJECT

	private:
		

	public:
		explicit TrackingExtendedOptionsDialog(QWidget *parent = 0);
		~TrackingExtendedOptionsDialog();

		Ui::TrackingExtendedOptionsDialog *diag;

	public slots:
		void on_pushButtonApply_clicked(bool checked);
		
};

#endif /* TRACKINGEXTENDEDOPTIONSDIALOG_H_ */
