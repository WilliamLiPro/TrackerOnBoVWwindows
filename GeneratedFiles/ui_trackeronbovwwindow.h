/********************************************************************************
** Form generated from reading UI file 'trackeronbovwwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.3.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TRACKERONBOVWWINDOW_H
#define UI_TRACKERONBOVWWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QWidget>
#include "widgetpainter.h"

QT_BEGIN_NAMESPACE

class Ui_TrackerOnBoVWwindowClass
{
public:
    QWidget *centralWidget;
    QTextBrowser *show_test;
    QGroupBox *groupBox_3;
    QToolButton *select_dataset_path;
    QLabel *load_dataset;
    QLabel *label_4;
    QComboBox *open_camera;
    QLabel *label;
    QGroupBox *groupBox;
    QToolButton *button_run;
    QToolButton *button_pause;
    QToolButton *button_stop;
    QGroupBox *groupBox_4;
    QLineEdit *text_BoVWpath;
    QToolButton *button_BoVWpath;
    QLabel *label_6;
    QComboBox *selet_feature;
    WidgetPainter *show_image;
    QGroupBox *groupBox_5;
    QLineEdit *text_savePath;
    QLabel *label_7;
    QSpinBox *sample_interval;
    QToolButton *button_savePath;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *TrackerOnBoVWwindowClass)
    {
        if (TrackerOnBoVWwindowClass->objectName().isEmpty())
            TrackerOnBoVWwindowClass->setObjectName(QStringLiteral("TrackerOnBoVWwindowClass"));
        TrackerOnBoVWwindowClass->resize(723, 533);
        QIcon icon;
        icon.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489848105.ico"), QSize(), QIcon::Normal, QIcon::Off);
        TrackerOnBoVWwindowClass->setWindowIcon(icon);
        TrackerOnBoVWwindowClass->setLayoutDirection(Qt::LeftToRight);
        TrackerOnBoVWwindowClass->setStyleSheet(QStringLiteral("background-color: qlineargradient(spread:pad, x1:0.550773, y1:1, x2:0.551136, y2:0.023, stop:0 rgba(80, 120, 120, 120), stop:1 rgba(240, 240, 255, 200));"));
        centralWidget = new QWidget(TrackerOnBoVWwindowClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        show_test = new QTextBrowser(centralWidget);
        show_test->setObjectName(QStringLiteral("show_test"));
        show_test->setGeometry(QRect(450, 10, 236, 341));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(show_test->sizePolicy().hasHeightForWidth());
        show_test->setSizePolicy(sizePolicy);
        show_test->setMinimumSize(QSize(200, 320));
        show_test->setSizeIncrement(QSize(0, 0));
        show_test->setLayoutDirection(Qt::LeftToRight);
        show_test->setStyleSheet(QStringLiteral("background-color: rgb(250, 255, 255);"));
        show_test->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        show_test->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        show_test->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        groupBox_3 = new QGroupBox(centralWidget);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setGeometry(QRect(30, 270, 200, 91));
        groupBox_3->setMinimumSize(QSize(150, 50));
        QFont font;
        font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font.setPointSize(10);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        groupBox_3->setFont(font);
        groupBox_3->setAcceptDrops(false);
        groupBox_3->setToolTipDuration(-1);
        groupBox_3->setStyleSheet(QStringLiteral("background-color: qlineargradient(spread:reflect, x1:0.517, y1:1, x2:0.511, y2:0.171, stop:0 rgba(100, 100, 100, 255), stop:0.829545 rgba(255, 255, 255, 255));"));
        groupBox_3->setFlat(false);
        groupBox_3->setCheckable(false);
        select_dataset_path = new QToolButton(groupBox_3);
        select_dataset_path->setObjectName(QStringLiteral("select_dataset_path"));
        select_dataset_path->setGeometry(QRect(105, 50, 41, 31));
        select_dataset_path->setAcceptDrops(false);
        select_dataset_path->setAutoFillBackground(false);
        select_dataset_path->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
"border-radius:5px;border-width:0px;\\nbackground-color: qlineargradient(spread:pad, x1:0.550773, y1:1, x2:0.551136, y2:0.023, stop:0 rgba(150, 180, 238, 255), stop:1 rgba(255, 255, 255, 255));\n"
""));
        select_dataset_path->setInputMethodHints(Qt::ImhNone);
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489847844.ico"), QSize(), QIcon::Normal, QIcon::Off);
        select_dataset_path->setIcon(icon1);
        select_dataset_path->setIconSize(QSize(48, 48));
        select_dataset_path->setArrowType(Qt::NoArrow);
        load_dataset = new QLabel(groupBox_3);
        load_dataset->setObjectName(QStringLiteral("load_dataset"));
        load_dataset->setGeometry(QRect(15, 55, 71, 21));
        load_dataset->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255,0);\n"
"color: rgb(0, 0, 0);"));
        label_4 = new QLabel(groupBox_3);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(15, 15, 56, 21));
        label_4->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255,0);\n"
"color: rgb(0, 0, 0);"));
        open_camera = new QComboBox(groupBox_3);
        open_camera->setObjectName(QStringLiteral("open_camera"));
        open_camera->setGeometry(QRect(70, 15, 111, 21));
        open_camera->setFont(font);
        open_camera->setAutoFillBackground(false);
        open_camera->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        open_camera->setEditable(false);
        open_camera->setMaxVisibleItems(8);
        open_camera->setInsertPolicy(QComboBox::InsertAtCurrent);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(530, 400, 146, 31));
        label->setStyleSheet(QLatin1String("background-color: qradialgradient(spread:pad, cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5, stop:0.335227 rgba(250, 250, 120, 255), stop:1 rgba(255, 240, 200, 0));\n"
"color: rgb(180, 100, 20);"));
        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(230, 270, 186, 91));
        groupBox->setMinimumSize(QSize(0, 0));
        groupBox->setStyleSheet(QStringLiteral("background-color: qlineargradient(spread:reflect, x1:0.517, y1:1, x2:0.511, y2:0.171, stop:0 rgba(100, 100, 100, 255), stop:0.829545 rgba(255, 255, 255, 255));"));
        button_run = new QToolButton(groupBox);
        button_run->setObjectName(QStringLiteral("button_run"));
        button_run->setGeometry(QRect(20, 25, 36, 36));
        button_run->setAcceptDrops(false);
        button_run->setAutoFillBackground(false);
        button_run->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
""));
        button_run->setInputMethodHints(Qt::ImhNone);
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489847909.ico"), QSize(), QIcon::Normal, QIcon::Off);
        button_run->setIcon(icon2);
        button_run->setIconSize(QSize(48, 48));
        button_pause = new QToolButton(groupBox);
        button_pause->setObjectName(QStringLiteral("button_pause"));
        button_pause->setGeometry(QRect(70, 25, 36, 36));
        button_pause->setAcceptDrops(false);
        button_pause->setAutoFillBackground(false);
        button_pause->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
""));
        button_pause->setInputMethodHints(Qt::ImhNone);
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489847914.ico"), QSize(), QIcon::Normal, QIcon::Off);
        button_pause->setIcon(icon3);
        button_pause->setIconSize(QSize(48, 48));
        button_stop = new QToolButton(groupBox);
        button_stop->setObjectName(QStringLiteral("button_stop"));
        button_stop->setGeometry(QRect(120, 25, 36, 36));
        button_stop->setAcceptDrops(false);
        button_stop->setAutoFillBackground(false);
        button_stop->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
""));
        button_stop->setInputMethodHints(Qt::ImhNone);
        QIcon icon4;
        icon4.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489847941.ico"), QSize(), QIcon::Normal, QIcon::Off);
        button_stop->setIcon(icon4);
        button_stop->setIconSize(QSize(48, 48));
        groupBox_4 = new QGroupBox(centralWidget);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        groupBox_4->setGeometry(QRect(30, 370, 280, 91));
        groupBox_4->setMinimumSize(QSize(280, 50));
        QFont font1;
        font1.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font1.setPointSize(10);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(9);
        groupBox_4->setFont(font1);
        groupBox_4->setAcceptDrops(false);
        groupBox_4->setToolTipDuration(-1);
        groupBox_4->setLayoutDirection(Qt::LeftToRight);
        groupBox_4->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:reflect, x1:0.517, y1:1, x2:0.511, y2:0.171, stop:0 rgba(100, 100, 100, 255), stop:0.829545 rgba(255, 255, 255, 255));\n"
"font: 75 10pt \"\345\256\213\344\275\223\";\n"
"color: rgb(0, 0, 0);"));
        groupBox_4->setFlat(false);
        groupBox_4->setCheckable(false);
        text_BoVWpath = new QLineEdit(groupBox_4);
        text_BoVWpath->setObjectName(QStringLiteral("text_BoVWpath"));
        text_BoVWpath->setGeometry(QRect(15, 55, 186, 20));
        text_BoVWpath->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255);\n"
"alternate-background-color: rgb(255, 255, 255);"));
        button_BoVWpath = new QToolButton(groupBox_4);
        button_BoVWpath->setObjectName(QStringLiteral("button_BoVWpath"));
        button_BoVWpath->setGeometry(QRect(220, 50, 41, 31));
        button_BoVWpath->setAcceptDrops(false);
        button_BoVWpath->setAutoFillBackground(false);
        button_BoVWpath->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
"border-radius:5px;border-width:0px;\\nbackground-color: qlineargradient(spread:pad, x1:0.550773, y1:1, x2:0.551136, y2:0.023, stop:0 rgba(78, 114, 238, 150), stop:1 rgba(255, 255, 255, 255));\n"
""));
        button_BoVWpath->setInputMethodHints(Qt::ImhNone);
        QIcon icon5;
        icon5.addFile(QStringLiteral(":/TrackerOnBoVWwindow/Resources/ooopic_1489847735.ico"), QSize(), QIcon::Normal, QIcon::Off);
        button_BoVWpath->setIcon(icon5);
        button_BoVWpath->setIconSize(QSize(48, 48));
        button_BoVWpath->setArrowType(Qt::NoArrow);
        label_6 = new QLabel(groupBox_4);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(45, 20, 76, 21));
        label_6->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255,0);\n"
"color: rgb(0, 0, 0);"));
        selet_feature = new QComboBox(groupBox_4);
        selet_feature->setObjectName(QStringLiteral("selet_feature"));
        selet_feature->setGeometry(QRect(125, 20, 76, 21));
        selet_feature->setFont(font1);
        selet_feature->setAutoFillBackground(false);
        selet_feature->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        selet_feature->setMaxVisibleItems(8);
        selet_feature->setInsertPolicy(QComboBox::InsertAtCurrent);
        show_image = new WidgetPainter(centralWidget);
        show_image->setObjectName(QStringLiteral("show_image"));
        show_image->setGeometry(QRect(30, 15, 320, 240));
        show_image->setAutoFillBackground(false);
        show_image->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        groupBox_5 = new QGroupBox(centralWidget);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        groupBox_5->setGeometry(QRect(310, 370, 201, 91));
        groupBox_5->setMinimumSize(QSize(100, 50));
        groupBox_5->setFont(font1);
        groupBox_5->setAcceptDrops(false);
        groupBox_5->setToolTipDuration(-1);
        groupBox_5->setLayoutDirection(Qt::LeftToRight);
        groupBox_5->setStyleSheet(QString::fromUtf8("background-color: qlineargradient(spread:reflect, x1:0.517, y1:1, x2:0.511, y2:0.171, stop:0 rgba(100, 100, 100, 255), stop:0.829545 rgba(255, 255, 255, 255));\n"
"font: 75 10pt \"\345\256\213\344\275\223\";\n"
"color: rgb(0, 0, 0);"));
        groupBox_5->setFlat(false);
        groupBox_5->setCheckable(false);
        text_savePath = new QLineEdit(groupBox_5);
        text_savePath->setObjectName(QStringLiteral("text_savePath"));
        text_savePath->setGeometry(QRect(15, 55, 101, 20));
        text_savePath->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255);\n"
"alternate-background-color: rgb(255, 255, 255);"));
        label_7 = new QLabel(groupBox_5);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(20, 20, 76, 21));
        label_7->setStyleSheet(QLatin1String("background-color: rgb(255, 255, 255,0);\n"
"color: rgb(0, 0, 0);"));
        sample_interval = new QSpinBox(groupBox_5);
        sample_interval->setObjectName(QStringLiteral("sample_interval"));
        sample_interval->setGeometry(QRect(90, 20, 51, 22));
        sample_interval->setStyleSheet(QStringLiteral("background-color: rgb(255, 255, 255);"));
        button_savePath = new QToolButton(groupBox_5);
        button_savePath->setObjectName(QStringLiteral("button_savePath"));
        button_savePath->setGeometry(QRect(130, 50, 41, 31));
        button_savePath->setAcceptDrops(false);
        button_savePath->setAutoFillBackground(false);
        button_savePath->setStyleSheet(QLatin1String("border-radius:5px;border-width:0px;\n"
"border-radius:5px;border-width:0px;\\nbackground-color: qlineargradient(spread:pad, x1:0.550773, y1:1, x2:0.551136, y2:0.023, stop:0 rgba(78, 114, 238, 150), stop:1 rgba(255, 255, 255, 255));\n"
""));
        button_savePath->setInputMethodHints(Qt::ImhNone);
        button_savePath->setIcon(icon5);
        button_savePath->setIconSize(QSize(48, 48));
        button_savePath->setArrowType(Qt::NoArrow);
        TrackerOnBoVWwindowClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(TrackerOnBoVWwindowClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 723, 23));
        TrackerOnBoVWwindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(TrackerOnBoVWwindowClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        TrackerOnBoVWwindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(TrackerOnBoVWwindowClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        TrackerOnBoVWwindowClass->setStatusBar(statusBar);
        QWidget::setTabOrder(show_test, selet_feature);
        QWidget::setTabOrder(selet_feature, text_BoVWpath);
        QWidget::setTabOrder(text_BoVWpath, button_BoVWpath);
        QWidget::setTabOrder(button_BoVWpath, open_camera);
        QWidget::setTabOrder(open_camera, select_dataset_path);
        QWidget::setTabOrder(select_dataset_path, button_run);
        QWidget::setTabOrder(button_run, button_pause);
        QWidget::setTabOrder(button_pause, button_stop);

        retranslateUi(TrackerOnBoVWwindowClass);
        QObject::connect(select_dataset_path, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(setDatasetPath()));
        QObject::connect(open_camera, SIGNAL(currentTextChanged(QString)), TrackerOnBoVWwindowClass, SLOT(setCamera()));
        QObject::connect(button_run, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(runTracking()));
        QObject::connect(button_pause, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(pauseTracking()));
        QObject::connect(button_stop, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(stopTracking()));
        QObject::connect(selet_feature, SIGNAL(currentTextChanged(QString)), TrackerOnBoVWwindowClass, SLOT(setFeatureType()));
        QObject::connect(text_BoVWpath, SIGNAL(textChanged(QString)), TrackerOnBoVWwindowClass, SLOT(setBoVWpath()));
        QObject::connect(button_BoVWpath, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(changeBoVWpath()));
        QObject::connect(open_camera, SIGNAL(highlighted(QString)), TrackerOnBoVWwindowClass, SLOT(updateCamera()));
        QObject::connect(text_savePath, SIGNAL(textChanged(QString)), TrackerOnBoVWwindowClass, SLOT(setSavePath()));
        QObject::connect(button_savePath, SIGNAL(released()), TrackerOnBoVWwindowClass, SLOT(changeSavePath()));
        QObject::connect(sample_interval, SIGNAL(valueChanged(int)), TrackerOnBoVWwindowClass, SLOT(setSampleInterval()));

        QMetaObject::connectSlotsByName(TrackerOnBoVWwindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *TrackerOnBoVWwindowClass)
    {
        TrackerOnBoVWwindowClass->setWindowTitle(QApplication::translate("TrackerOnBoVWwindowClass", "\347\233\256\346\240\207\350\267\237\350\270\252\347\250\213\345\272\217V0.4", 0));
        show_test->setHtml(QApplication::translate("TrackerOnBoVWwindowClass", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'SimSun'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">  \346\254\242\350\277\216\344\275\277\347\224\250\347\233\256\346\240\207\350\267\237\350\270\252\346\265\213\350\257\225\347\250\213\345\272\217</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">* \346\234\254\347\256\227\346\263\225\345\237\272\344\272\216BoVW\347\211\271\345\276\201\350\267\237\350\270\252\345\217\212\347\211"
                        "\271\345\276\201\345\212\250\346\200\201\347\255\233\351\200\211\347\256\227\346\263\225\357\274\214\346\224\257\346\214\201\345\244\232\347\247\215\347\211\271\345\276\201\347\232\204\350\267\237\350\270\252,\345\214\205\346\213\254\357\274\232</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">  SIFT,SURF,ORB,BRISK,BRIEF,FREAK</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">* \346\224\257\346\214\201\346\221\204\345\203\217\345\244\264\344\270\216\346\225\260\346\215\256\351\233\206\350\267\237\350\270\252</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top"
                        ":0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">* \345\274\200\345\220\257\347\233\256\346\240\207\350\267\237\350\270\252\345\211\215\357\274\214\350\257\267\345\205\210\346\217\220\344\276\233\350\256\255\347\273\203\345\245\275\347\232\204BoVW\350\267\257\345\276\204\344\273\245\345\217\212BoVW\347\211\271\345\276\201\347\261\273\345\236\213</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">* \346\211\223\345\274\200\346\221\204\345\203\217\345\244\264\346\210\226\346\225\260\346\215\256\351\233\206\345\220\216\357\274\214\350\257\267\345\205\210\347\224\250\351\274\240\346\240\207\346\241\206\351\200\211\350\242\253\350\267\237\350\270\252\347\233\256\346\240\207</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0p"
                        "x; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", 0));
        show_test->setPlaceholderText(QString());
        groupBox_3->setTitle(QString());
        select_dataset_path->setText(QString());
        load_dataset->setText(QApplication::translate("TrackerOnBoVWwindowClass", "<html><head/><body><p>\350\257\273\345\217\226\346\225\260\346\215\256\351\233\206</p></body></html>", 0));
        label_4->setText(QApplication::translate("TrackerOnBoVWwindowClass", "<html><head/><body><p>\346\221\204\345\203\217\345\244\264</p></body></html>", 0));
        open_camera->clear();
        open_camera->insertItems(0, QStringList()
         << QApplication::translate("TrackerOnBoVWwindowClass", "\344\270\215\345\274\200\345\220\257", 0)
        );
        label->setText(QApplication::translate("TrackerOnBoVWwindowClass", "<html><head/><body><p align=\"center\"><span style=\" font-size:12pt;\">Version 0.9</span></p></body></html>", 0));
        groupBox->setTitle(QString());
        button_run->setText(QString());
        button_pause->setText(QApplication::translate("TrackerOnBoVWwindowClass", "\345\201\234\346\255\242", 0));
        button_stop->setText(QApplication::translate("TrackerOnBoVWwindowClass", "\345\201\234\346\255\242", 0));
        groupBox_4->setTitle(QApplication::translate("TrackerOnBoVWwindowClass", "BoVW \350\267\257\345\276\204", 0));
        text_BoVWpath->setText(QApplication::translate("TrackerOnBoVWwindowClass", "BoVW/SURF BoVW.dat", 0));
        button_BoVWpath->setText(QString());
        label_6->setText(QApplication::translate("TrackerOnBoVWwindowClass", "<html><head/><body><p>\347\211\271\345\276\201\347\261\273\345\236\213</p></body></html>", 0));
        selet_feature->clear();
        selet_feature->insertItems(0, QStringList()
         << QApplication::translate("TrackerOnBoVWwindowClass", "SURF", 0)
         << QApplication::translate("TrackerOnBoVWwindowClass", "SIFT", 0)
         << QApplication::translate("TrackerOnBoVWwindowClass", "ORB", 0)
         << QApplication::translate("TrackerOnBoVWwindowClass", "BRISK", 0)
         << QApplication::translate("TrackerOnBoVWwindowClass", "BRIEF", 0)
         << QApplication::translate("TrackerOnBoVWwindowClass", "FREAK", 0)
        );
        groupBox_5->setTitle(QApplication::translate("TrackerOnBoVWwindowClass", "\344\277\235\345\255\230\347\273\223\346\236\234", 0));
        text_savePath->setText(QApplication::translate("TrackerOnBoVWwindowClass", "Result/", 0));
        label_7->setText(QApplication::translate("TrackerOnBoVWwindowClass", "<html><head/><body><p>\351\207\207\346\240\267\351\227\264\351\232\224</p></body></html>", 0));
        button_savePath->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class TrackerOnBoVWwindowClass: public Ui_TrackerOnBoVWwindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TRACKERONBOVWWINDOW_H
