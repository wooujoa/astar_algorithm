/**
 * @file /include/astar_algorithm/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef astar_algorithm_MAIN_WINDOW_H
#define astar_algorithm_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QIcon>
#include <QGridLayout>
#include <QMainWindow>
#include <QMessageBox>
#include <QPoint>
#include <QPushButton>
#include <QVector>
#include <QSpinBox>       // 추가
#include <QLabel>         // 추가
#include <QVBoxLayout>    // 추가
#include <QHBoxLayout>    // 추가
#include <QWidget>        // 추가
#include "ui_mainwindow.h"
#include "AStarPathfinder.h"
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

protected:
  void closeEvent(QCloseEvent *event) override;

private slots:
  void set_mapsetting();
  void set_start();
  void set_end();
  void set_astar();
  void set_reset();
  void cellClicked();

private:
  Ui::MainWindowDesign *ui;
  // 맵 관련 변수
  std::vector<std::vector<bool>> walls;       // true: 벽, false: 빈 공간
  QVector<QVector<QPushButton *>> mapButtons; // 맵의 버튼들
  int rows{0}, cols{0};                             // 맵 크기

  // 시작점과 도착점
  QPoint startPoint;
  QPoint endPoint;
  bool hasStart;
  bool hasEnd;
  bool settingStart; // 시작점 설정 모드
  bool settingEnd;   // 도착점 설정 모드

  // A* 알고리즘 객체
  AStar *astar;

  // 유틸리티 함수
  void createMap();
  void clearPath();
  void updateCellColor(int x, int y, const QString &color);
  void clearMapResources();
};

#endif // astar_algorithm_MAIN_WINDOW_H
