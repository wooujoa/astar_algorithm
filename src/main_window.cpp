#include "../include/astar_algorithm/main_window.hpp"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindowDesign),
      rows(0),
      cols(0),
      startPoint(-1, -1), // 시작점 초기화 추가
      endPoint(-1, -1),   // 끝점 초기화 추가
      hasStart(false),
      hasEnd(false),
      settingStart(false),
      settingEnd(false),
      astar(nullptr)
{
  ui->setupUi(this);

  // 버튼 연결
  connect(ui->mapsetting, &QPushButton::clicked, this, &MainWindow::set_mapsetting);
  connect(ui->start, &QPushButton::clicked, this, &MainWindow::set_start);
  connect(ui->end, &QPushButton::clicked, this, &MainWindow::set_end);
  connect(ui->astar, &QPushButton::clicked, this, &MainWindow::set_astar);
  connect(ui->reset, &QPushButton::clicked, this, &MainWindow::set_reset);

  // SpinBox 범위 설정
  ui->row->setRange(1, 100);
  ui->col->setRange(1, 100);

  // 기본값 설정
  ui->row->setValue(10); // 기본값 10으로 변경
  ui->col->setValue(10); // 기본값 10으로 변경

  // 버튼 초기 상태 설정
  ui->start->setEnabled(false);
  ui->end->setEnabled(false);
  ui->astar->setEnabled(false);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
  delete astar;
  // 맵 버튼들 정리
  for (auto &row : mapButtons)
  {
    for (auto &btn : row)
    {
      delete btn;
    }
  }
}
// 새로운 함수 추가: 맵 리소스 정리
void MainWindow::clearMapResources()
{
  // 기존 버튼들 정리
  for (int i = 0; i < mapButtons.size(); ++i)
  {
    for (int j = 0; j < mapButtons[i].size(); ++j)
    {
      if (mapButtons[i][j])
      {
        ui->gridLayout->removeWidget(mapButtons[i][j]);
        delete mapButtons[i][j];
        mapButtons[i][j] = nullptr;
      }
    }
  }
  mapButtons.clear();

  // A* 객체 정리
  if (astar)
  {
    delete astar;
    astar = nullptr;
  }

  // 상태 초기화
  hasStart = false;
  hasEnd = false;
  settingStart = false;
  settingEnd = false;
  startPoint = QPoint(-1, -1);
  endPoint = QPoint(-1, -1);

  // 벽 정보 초기화
  walls.clear();
}

void MainWindow::set_mapsetting()
{
  // 현재 값 저장
  int newRows = ui->row->value();
  int newCols = ui->col->value();

  // 유효성 검사
  if (newRows <= 0 || newCols <= 0)
  {
    QMessageBox::warning(this, "Warning", "Row and column values must be greater than 0!");
    return;
  }

  // 새로운 크기 설정
  rows = newRows;
  cols = newCols;

  // 맵 생성
  createMap();
}

void MainWindow::createMap()
{
  // 기존 버튼과 리소스 정리
  clearMapResources();

  try
  {
    // 맵 데이터 초기화
    walls = std::vector<std::vector<bool>>(rows, std::vector<bool>(cols, false));
    mapButtons = QVector<QVector<QPushButton *>>(rows, QVector<QPushButton *>(cols, nullptr));

    // UI 설정
    ui->gridLayout->setSpacing(1);
    ui->gridLayout->setContentsMargins(0, 0, 0, 0);

    // UI 버튼 생성
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        QPushButton *btn = new QPushButton(this);
        if (!btn)
        {
          throw std::runtime_error("Failed to create button");
        }

        btn->setFixedSize(30, 30);
        btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        btn->setStyleSheet("background-color: white; border: 0.5px solid black;");

        btn->setProperty("row", i);
        btn->setProperty("col", j);
        connect(btn, &QPushButton::clicked, this, &MainWindow::cellClicked);

        ui->gridLayout->addWidget(btn, i, j);
        mapButtons[i][j] = btn;
      }
    }

    // A* 객체 생성
    if (astar)
    {
      delete astar;
      astar = nullptr;
    }
    astar = new AStar(rows, cols);
    if (!astar)
    {
      throw std::runtime_error("Failed to create AStar object");
    }

    // 버튼 활성화
    ui->start->setEnabled(true);
    ui->end->setEnabled(true);
    ui->astar->setEnabled(false);
  }
  catch (const std::exception &e)
  {
    QMessageBox::critical(this, "Error", QString("Failed to create map: %1").arg(e.what()));
    clearMapResources();
  }
}
void MainWindow::cellClicked()
{
  QPushButton *btn = qobject_cast<QPushButton *>(sender());
  if (!btn)
    return;

  int x = btn->property("row").toInt();
  int y = btn->property("col").toInt();

  if (settingStart)
  {
    if (walls[x][y])
    {
      QMessageBox::warning(this, "Warning", "Cannot set start point on a wall!");
      return;
    }

    if (hasStart)
    {
      updateCellColor(startPoint.x(), startPoint.y(), "white");
    }
    startPoint = QPoint(x, y);
    hasStart = true;
    updateCellColor(x, y, "green");
    settingStart = false;
  }
  else if (settingEnd)
  {
    if (walls[x][y])
    {
      QMessageBox::warning(this, "Warning", "Cannot set end point on a wall!");
      return;
    }

    if (hasEnd)
    {
      updateCellColor(endPoint.x(), endPoint.y(), "white");
    }
    endPoint = QPoint(x, y);
    hasEnd = true;
    updateCellColor(x, y, "red");
    settingEnd = false;
  }
  else
  {
    // 시작점이나 끝점에 벽을 설치하려고 할 때
    if ((hasStart && x == startPoint.x() && y == startPoint.y()) ||
        (hasEnd && x == endPoint.x() && y == endPoint.y()))
    {
      QMessageBox::warning(this, "Warning", "Cannot place wall on start/end point!");
      return;
    }

    walls[x][y] = !walls[x][y];
    updateCellColor(x, y, walls[x][y] ? "black" : "white");
  }

  // A* 버튼 활성화 조건 체크
  ui->astar->setEnabled(hasStart && hasEnd);
}
void MainWindow::set_start()
{
  settingStart = true;
  settingEnd = false;
}

void MainWindow::set_end()
{
  settingStart = false;
  settingEnd = true;
}

void MainWindow::set_astar()
{
  if (rows <= 0 || cols <= 0 || mapButtons.empty())
  {
    QMessageBox::warning(this, "Warning", "Please set up the map first!");
    return;
  }

  if (!hasStart || !hasEnd || !astar)
  {
    QMessageBox::warning(this, "Warning", "Please set start and end points first!");
    return;
  }

  clearPath(); // 이전 경로 지우기

  // A* 알고리즘으로 경로 찾기
  std::vector<QPoint> path = astar->findPath(startPoint, endPoint, walls);

  // 찾은 경로 표시
  for (const QPoint &point : path)
  {
    if (point != startPoint && point != endPoint)
    {
      updateCellColor(point.x(), point.y(), "blue");
    }
  }
}

void MainWindow::set_reset()
{
  // 모든 상태 초기화
  hasStart = false;
  hasEnd = false;
  settingStart = false;
  settingEnd = false;
  startPoint = QPoint(-1, -1);
  endPoint = QPoint(-1, -1);

  // 버튼 상태 설정
  ui->start->setEnabled(true);
  ui->end->setEnabled(true);
  ui->astar->setEnabled(false);

  // 맵이 존재하는 경우에만 초기화
  if (!mapButtons.empty() && !walls.empty())
  {
    for (int i = 0; i < rows && i < mapButtons.size(); ++i)
    {
      for (int j = 0; j < cols && j < mapButtons[i].size(); ++j)
      {
        if (mapButtons[i][j])
        {
          walls[i][j] = false;
          mapButtons[i][j]->setStyleSheet("background-color: white; border: 0.5px solid black;");
        }
      }
    }
  }
}

void MainWindow::updateCellColor(int x, int y, const QString &color)
{
  if (x >= 0 && x < rows && y >= 0 && y < cols && mapButtons[x][y])
  {
    mapButtons[x][y]->setStyleSheet(QString("background-color: %1;").arg(color));
  }
}

void MainWindow::clearPath()
{
  for (int i = 0; i < rows; i++)
  {
    for (int j = 0; j < cols; j++)
    {
      if (!walls[i][j] && QPoint(i, j) != startPoint && QPoint(i, j) != endPoint)
      {
        updateCellColor(i, j, "white");
      }
    }
  }
}