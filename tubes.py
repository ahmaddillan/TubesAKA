import heapq
import time
import random
import sys

# Menetapkan batas rekursi yang lebih tinggi jika ingin menggunakan rekursi
sys.setrecursionlimit(1000000)  # Menambah batas rekursi jika perlu (hati-hati)

random.seed(42)

def heuristic(a, b):
    """Fungsi heuristic menggunakan jarak Manhattan."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search_iterative(graph, start, goal):
    """Implementasi algoritma A* secara iteratif."""
    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}

    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0

    f_score = {node: float('inf') for node in graph}
    f_score[start] = heuristic(start, goal)

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in graph[current]:
            tentative_g_score = g_score[current] + graph[current][neighbor]

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

                if neighbor not in [i[1] for i in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None

def a_star_search_recursive(graph, start, goal, open_set=None, came_from=None, g_score=None, f_score=None, max_depth=10000):
    """Implementasi algoritma A* secara rekursif."""
    if open_set is None:
        open_set = []
        heapq.heappush(open_set, (0, start))

    if came_from is None:
        came_from = {}

    if g_score is None:
        g_score = {node: float('inf') for node in graph}
        g_score[start] = 0

    if f_score is None:
        f_score = {node: float('inf') for node in graph}
        f_score[start] = heuristic(start, goal)

    if not open_set:
        return None

    current = heapq.heappop(open_set)[1]

    if current == goal:
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(start)
        return path[::-1]

    if max_depth <= 0:
        return None

    for neighbor in graph[current]:
        tentative_g_score = g_score[current] + graph[current][neighbor]

        if tentative_g_score < g_score[neighbor]:
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)

            if neighbor not in [i[1] for i in open_set]:
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # Berhenti pada batas maksimal kedalaman rekursi
    return a_star_search_recursive(graph, start, goal, open_set, came_from, g_score, f_score, max_depth - 1)

def build_graph(x, y):
    """Membangun graf berbobot untuk grid berukuran x * y."""
    nodes = [(i, j) for i in range(x) for j in range(y)]
    graph = {node: {} for node in nodes}

    for i, j in nodes:
        if i > 0:  # Koneksi ke atas
            graph[(i, j)][(i - 1, j)] = 1
        if i < x - 1:  # Koneksi ke bawah
            graph[(i, j)][(i + 1, j)] = 1
        if j > 0:  # Koneksi ke kiri
            graph[(i, j)][(i, j - 1)] = 1
        if j < y - 1:  # Koneksi ke kanan
            graph[(i, j)][(i, j + 1)] = 1

    return graph

def main():
    print("\n=== Program A* Search ===")
    print("Program akan menjalankan A* Iteratif dan A* Rekursif dalam satu kali proses.\n")

    try:
        x = int(input("Masukkan jumlah baris grid (x): "))
        y = int(input("Masukkan jumlah kolom grid (y): "))

        graph = build_graph(x, y)
        start = (0, 0)
        goal = (x - 1, y - 1)

        # A* Iteratif
        print("\n=== A* Iteratif ===")
        start_time_iter = time.time()
        path_iter = a_star_search_iterative(graph, start, goal)
        end_time_iter = time.time()
        if path_iter:
            print(f"Jalur terpendek dari {start} ke {goal} (Iteratif):")
            print(" -> ".join(str(point) for point in path_iter))
        else:
            print(f"Tidak ada jalur yang ditemukan dari {start} ke {goal}.")
        print(f"Waktu eksekusi (Iteratif): {end_time_iter - start_time_iter:.6f} detik")

        # A* Rekursif
        print("\n=== A* Rekursif ===")
        start_time_recur = time.time()
        path_recur = a_star_search_recursive(graph, start, goal)
        end_time_recur = time.time()
        if path_recur:
            print(f"Jalur terpendek dari {start} ke {goal} (Rekursif):")
            print(" -> ".join(str(point) for point in path_recur))
        else:
            print(f"Tidak ada jalur yang ditemukan dari {start} ke {goal}.")
        print(f"Waktu eksekusi (Rekursif): {end_time_recur - start_time_recur:.6f} detik")

    except ValueError:
        print("Input tidak valid. Masukkan angka yang benar.")

if __name__ == "__main__":
    main()