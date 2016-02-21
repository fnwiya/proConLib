#!/usr/bin/env python
# -*- coding: utf-8 -*-
import heapq


class Dijkstra(object):
    def dijkstra(self, adj, start, goal=None):
        '''
        ダイクストラアルゴリズムによる最短経路を求めるメソッド
        入力
        adj: adj[i][j]の値が頂点iから頂点jまでの距離(頂点iから頂点jに枝がない場合，
             値はfloat('inf'))となるような2次元リスト(正方行列)
        start: 始点のID
        goal: オプション引数．終点のID
        出力
        goalを引数に持つ場合，startからgoalまでの最短経路を格納したリストを返す
        持たない場合は，startから各頂点までの最短距離を格納したリストを返す
        >>> d = Dijkstra()
        >>> d.dijkstra([[float('inf'), 2, 4, float('inf'), float('inf')],
                        [2, float('inf'), 3, 5, float('inf')],
                        [4, 3, float('inf'), 1, 4],
                        [float('inf'), 5, 1, float('inf'), 3],
                        [float('inf'), float('inf'), 4, 3, float('inf')]], 0)
        [0, 2, 4, 5, 8] # 例えば，始点0から頂点3までの最短距離は5となる
        >>> d.dijkstra([[float('inf'), 2, 4, float('inf'), float('inf')],
                        [2, float('inf'), 3, 5, float('inf')],
                        [4, 3, float('inf'), 1, 4],
                        [float('inf'), 5, 1, float('inf'), 3],
                        [float('inf'), float('inf'), 4, 3, float('inf')]], 0, goal=4)
        [0, 2, 4] # 頂点0から頂点4までの最短経路は0 -> 2 -> 4となる
        '''
        num = len(adj)
        dist = [float('inf') for i in range(num)]
        prev = [float('inf') for i in range(num)]

        dist[start] = 0
        q = []
        heapq.heappush(q, (0, start))
        while len(q) != 0:
            prov_cost, src = heapq.heappop(q)
            if dist[src] < prov_cost:
                continue
            for dest in range(num):
                cost = adj[src][dest]
                if cost != float('inf') and dist[dest] > dist[src] + cost:
                    dist[dest] = dist[src] + cost
                    heapq.heappush(q, (dist[dest], dest))
                    prev[dest] = src

        if goal is not None:
            return self.get_path(goal, prev)
        else:
            return dist

    def get_path(self, goal, prev):
        path = [goal]
        dest = goal

        while prev[dest] != float('inf'):
            path.append(prev[dest])
            dest = prev[dest]

        return list(reversed(path))

if __name__ == '__main__':
    import doctest
    doctest.testmod()
