#include <bits/stdc++.h>
using namespace std;

vector<vector<int>> ug;
vector<vector<pair<int,int>>> wg;
vector<int> vis;

// BFS
void runBFS(int start){
    queue<int> q;
    q.push(start);
    vis[start] = 1;

    while(!q.empty()){
        int x = q.front();
        q.pop();
        cout << x << " ";

        for(int nxt : ug[x]){
            if(!vis[nxt]){
                vis[nxt] = 1;
                q.push(nxt);
            }
        }
    }
}

// DFS
void dfs(int x){
    vis[x] = 1;
    cout << x << " ";
    for(int nxt : ug[x]){
        if(!vis[nxt]) dfs(nxt);
    }
}

void runDFS(int start){
    dfs(start);
}

// DSU for Kruskal
struct DSU{
    vector<int> p, r;
    DSU(int n): p(n), r(n,0){
        for(int i=0;i<n;i++) p[i] = i;
    }
    int find(int x){
        if(p[x] == x) return x;
        return p[x] = find(p[x]);
    }
    bool unite(int a,int b){
        a = find(a);
        b = find(b);
        if(a == b) return false;
        if(r[a] < r[b]) swap(a,b);
        p[b] = a;
        if(r[a] == r[b]) r[a]++;
        return true;
    }
};

// Kruskal
int runKruskal(int n, vector<tuple<int,int,int>> &edges){
    sort(edges.begin(), edges.end());
    DSU d(n);
    int cost = 0;

    for(auto &e : edges){
        int w,u,v;
        tie(w,u,v) = e;
        if(d.unite(u,v)) cost += w;
    }

    return cost;
}

// Prim
int runPrim(int n){
    vector<int> used(n,0);
    int cost = 0;

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    pq.push({0,0});

    while(!pq.empty()){
        auto [w, node] = pq.top();
        pq.pop();
        if(used[node]) continue;

        used[node] = 1;
        cost += w;

        for(auto &nbr : wg[node]){
            pq.push({nbr.second, nbr.first});
        }
    }

    return cost;
}

// Dijkstra
vector<int> runDijkstra(int n, int start){
    vector<int> dist(n, INT_MAX);
    dist[start] = 0;

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    pq.push({0,start});

    while(!pq.empty()){
        auto [d,u] = pq.top();
        pq.pop();
        if(d != dist[u]) continue;

        for(auto &nbr : wg[u]){
            int v = nbr.first;
            int w = nbr.second;

            if(dist[u] + w < dist[v]){
                dist[v] = dist[u] + w;
                pq.push({dist[v], v});
            }
        }
    }

    return dist;
}

// MAIN ONLY CALLS FUNCTIONS
int main(){
    int choice;
    cin >> choice;

    if(choice == 1){             // BFS
        int n, m;
        cin >> n >> m;
        ug.assign(n, {});
        vis.assign(n, 0);

        while(m--){
            int u, v;
            cin >> u >> v;
            ug[u].push_back(v);
            ug[v].push_back(u);
        }

        int start;
        cin >> start;
        runBFS(start);
    }

    else if(choice == 2){        // DFS
        int n, m;
        cin >> n >> m;
        ug.assign(n, {});
        vis.assign(n, 0);

        while(m--){
            int u, v;
            cin >> u >> v;
            ug[u].push_back(v);
            ug[v].push_back(u);
        }

        int start;
        cin >> start;
        runDFS(start);
    }

    else if(choice == 3){        // Kruskal + Prim
        int n, m;
        cin >> n >> m;

        vector<tuple<int,int,int>> edges;
        wg.assign(n, {});

        while(m--){
            int u, v, w;
            cin >> u >> v >> w;
            edges.push_back({w,u,v});
            wg[u].push_back({v,w});
            wg[v].push_back({u,w});
        }

        int k = runKruskal(n, edges);
        int p = runPrim(n);
        cout << k << " " << p;
    }

    else if(choice == 4){        // Dijkstra
        int n, m;
        cin >> n >> m;

        wg.assign(n, {});
        while(m--){
            int u, v, w;
            cin >> u >> v >> w;
            wg[u].push_back({v,w});
        }

        int start;
        cin >> start;

        vector<int> dist = runDijkstra(n, start);

        for(int d : dist) cout << d << " ";
    }

    return 0;
}
