import numpy as np
from track import load_track
import matplotlib.pyplot as plt
import pure_viterbi


def line_track_trellis_graph(line_track):
    h, w = line_track.shape[:2]
    r = 5
    n_nodes = h//r
    n_states = w//r

    A = np.zeros((n_states, n_states), dtype=float)
    B = np.zeros((n_states, n_nodes), dtype=float)
    O = np.zeros(n_nodes, dtype=np.int32)

    B = np.zeros((n_nodes, n_states), dtype=int)
    for i in range(B.shape[0]):
        rows = r * i
        for j in range(B.shape[1]):
            cols = r * j
            val = np.sum(line_track[rows:rows+r, cols:cols+r])
            B[i, j] = val
        O[i] = np.argmax(B[i])
    B = B / np.max(B)
    A = B.T @ B
    A = A / np.max(A)
    return A, B, O


if __name__ == "__main__":
    IMG_PATH = "./tracks/line.png"
    track = load_track(IMG_PATH)

    A, B, O = line_track_trellis_graph(track)
    # print(A, B, O)

    B = B
    C = np.zeros_like(A[0])
    S_Opt, D, E = pure_viterbi.viterbi(A, C, B, O.astype(np.int32))

    plt.imshow(B)
    plt.scatter(O, list(range(len(O))), label="Baseline")
    plt.scatter(S_Opt, list(range(len(S_Opt))), label="Pure Viterbi")
    plt.legend()
    plt.show()

    print('Optimal state sequence: S = ', S_Opt)
    np.set_printoptions(formatter={'float': "{: 7.4f}".format})
    print('D =', D, sep='\n')
    np.set_printoptions(formatter={'float': "{: 7.0f}".format})
    print('E =', E, sep='\n')