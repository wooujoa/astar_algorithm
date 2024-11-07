// AStarPathfinder.cpp
#include "../include/astar_algorithm/AStarPathfinder.h"
#include <unordered_map>
#include <algorithm>

// 그리드 크기 초기화
AStar::AStar(int rows, int cols) : rows(rows), cols(cols) {}

// 시작점에서 도착점까지 경로 찾기
std::vector<QPoint> AStar::findPath(const QPoint &start, const QPoint &end,
                                    const std::vector<std::vector<bool>> &walls)
{
  std::priority_queue<Node *, std::vector<Node *>, CompareNode> openList;          // 탐색할 노드들
  std::unordered_map<int, std::unordered_map<int, Node *>> nodeMap;                // 생성된 모든 노드들
  std::vector<std::vector<bool>> closedList(rows, std::vector<bool>(cols, false)); // 이미 탐색한 노드들

  // 시작 노드 생성
  Node *startNode = new Node(start.x(), start.y());
  startNode->h = calculateHeuristic(start.x(), start.y(), end.x(), end.y());
  startNode->f = startNode->h;

  openList.push(startNode);
  nodeMap[start.x()][start.y()] = startNode;

  while (!openList.empty())
  {
    // f값이 가장 작은 노드 선택
    Node *current = openList.top();
    openList.pop();

    int x = current->x;
    int y = current->y;

    // 이미 처리된 노드면 스킵
    if (closedList[x][y])
      continue;
    closedList[x][y] = true;

    // 목적지 도달
    if (x == end.x() && y == end.y())
    {
      std::vector<QPoint> path = reconstructPath(current);

      // 메모리 정리
      for (auto &row : nodeMap)
      {
        for (auto &col : row.second)
        {
          delete col.second;
        }
      }

      return path;
    }

    // 8방향 탐색
    for (size_t i = 0; i < directions.size(); i++)
    {
      int newX = x + directions[i].first;
      int newY = y + directions[i].second;

      // 유효하지 않은 위치, 이미 처리된 노드, 또는 이동 불가능한 위치는 건너뛰기
      if (!isValid(newX, newY) || closedList[newX][newY] ||
          !canMove(walls, x, y, newX, newY))
      {
        continue;
      }

      // 이동 비용 계산
      int moveCost = getMoveCost(directions[i].first, directions[i].second);
      int newG = current->g + moveCost;

      // 이웃 노드 처리
      Node *neighbor;
      auto rowIt = nodeMap.find(newX);
      if (rowIt != nodeMap.end() && rowIt->second.find(newY) != rowIt->second.end())
      {
        // 이미 생성된 노드인 경우
        neighbor = rowIt->second[newY];
        if (newG >= neighbor->g)
        {
          continue; // 이미 더 좋은 경로가 있음
        }
      }
      else
      {
        // 새로운 노드 생성
        neighbor = new Node(newX, newY);
        nodeMap[newX][newY] = neighbor;
      }

      // 노드 정보 업데이트
      neighbor->parent = current;
      neighbor->g = newG;
      neighbor->h = calculateHeuristic(newX, newY, end.x(), end.y());
      neighbor->f = neighbor->g + neighbor->h;

      openList.push(neighbor);
    }
  }

  // 경로를 찾지 못한 경우 메모리 정리
  for (auto &row : nodeMap)
  {
    for (auto &col : row.second)
    {
      delete col.second;
    }
  }

  return std::vector<QPoint>();
}

// 좌표가 그리드 범위 내에 있는지 확인하는 함수
bool AStar::isValid(int x, int y) const
{
  return x >= 0 && x < rows && y >= 0 && y < cols;
}

// 현재 위치에서 새로운 위치로 이동 가능한지 확인하는 함수
bool AStar::canMove(const std::vector<std::vector<bool>> &walls, int x, int y,
                    int newX, int newY) const
{
  // 벽 체크
  if (walls[newX][newY])
    return false;

  // 대각선 이동시 추가 체크
  int dx = newX - x;
  int dy = newY - y;
  if (dx != 0 && dy != 0)
  {
    // 대각선 이동시 인접한 두 칸이 모두 벽이면 이동 불가
    if (walls[x][newY] || walls[newX][y])
    {
      return false;
    }
  }

  return true;
}

// 두 점 사이의 휴리스틱 비용을 계산하는 함수 (대각선 거리 사용)
int AStar::calculateHeuristic(int x1, int y1, int x2, int y2) const
{
  int dx = std::abs(x1 - x2);
  int dy = std::abs(y1 - y2);

  // 대각선(14) + 직선(10) 이동을 고려한 거리 계산
  return 10 * (dx + dy) + (14 - 2 * 10) * std::min(dx, dy);
}

// 이동 방향에 따른 비용을 반환하는 함수
int AStar::getMoveCost(int dx, int dy) const
{
  // 대각선 이동이면 14, 직선 이동이면 10
  return (dx != 0 && dy != 0) ? 14 : 10;
}

// 최종 경로를 재구성하는 함수
std::vector<QPoint> AStar::reconstructPath(Node *endNode) const
{
  std::vector<QPoint> path;
  Node *current = endNode;

  // 끝점에서 시작점까지 역추적하여 경로 구성
  while (current != nullptr)
  {
    path.push_back(QPoint(current->x, current->y));
    current = current->parent;
  }

  // 경로를 시작점에서 끝점 순서로 뒤집기
  std::reverse(path.begin(), path.end());
  return path;
}