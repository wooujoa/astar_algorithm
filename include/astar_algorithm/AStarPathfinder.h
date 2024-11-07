#ifndef ASTAR_PATHFINDER_H
#define ASTAR_PATHFINDER_H

#include <vector>
#include <queue> 
#include <unordered_set>
#include <cmath>
#include <QPoint>

class AStar
{
public:
  AStar(int rows, int cols);
  ~AStar() = default;

  std::vector<QPoint> findPath(const QPoint &start, const QPoint &end,
                               const std::vector<std::vector<bool>> &walls);

private:
  struct Node
  {
    int x, y;
    int g; // 시작점부터의 실제 비용
    int h; // 목표점까지의 예상 비용
    int f; // 총 비용 (g + h)
    Node *parent;

    Node(int x, int y) : x(x), y(y), g(0), h(0), f(0), parent(nullptr) {}

    // 노드 비교를 위한 연산자
    bool operator==(const Node &other) const
    {
      return x == other.x && y == other.y;
    }
  };

  // 노드 포인터 비교를 위한 구조체
  struct CompareNode
  {
    bool operator()(const Node *a, const Node *b) const
    {
      if (a->f == b->f)
      {
        return a->h > b->h; // f가 같으면 h가 작은 것을 선호
      }
      return a->f > b->f;
    }
  };

  // 해시 함수를 위한 구조체
  struct NodeHash
  {
    std::size_t operator()(const Node &node) const
    {
      return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
  };

  int rows;
  int cols;

  // 8방향 이동 배열 (상하좌우 + 대각선)
  const std::vector<std::pair<int, int>> directions = {
      {-1, 0},  // 상
      {1, 0},   // 하
      {0, -1},  // 좌
      {0, 1},   // 우
      {-1, -1}, // 좌상
      {-1, 1},  // 우상
      {1, -1},  // 좌하
      {1, 1}    // 우하
  };

  bool isValid(int x, int y) const;
  bool canMove(const std::vector<std::vector<bool>> &walls, int x, int y, int newX, int newY) const;
  int calculateHeuristic(int x1, int y1, int x2, int y2) const;
  int getMoveCost(int dx, int dy) const;
  std::vector<QPoint> reconstructPath(Node *endNode) const;
};

#endif // ASTAR_PATHFINDER_H