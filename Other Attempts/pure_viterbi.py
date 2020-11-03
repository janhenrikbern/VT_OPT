import numpy as np

def viterbi(A, C, B, O):
    """Viterbi algorithm for solving the uncovering problem

    Notebook: C5/C5S3_Viterbi.ipynb
    Source: https://www.audiolabs-erlangen.de/resources/MIR/FMP/C5/C5S3_Viterbi.html

    Citation: 
    Lawrence R. Rabiner: A Tutorial on Hidden Markov Models and Selected Applications in 
    Speech Recognition. Proceedings of the IEEE, 77 (1989), pp. 257–286. 

    Args:
        A: State transition probability matrix of dimension I x I
        C: Initial state distribution  of dimension I
        B: Output probability matrix of dimension I x K
        O: Observation sequence of length N

    Returns:
        S_opt: Optimal state sequence of length N
        D: Accumulated probability matrix
        E: Backtracking matrix
    """
    I = A.shape[0]    # Number of states
    N = len(O)  # Length of observation sequence

    # Initialize D and E matrices
    D = np.zeros((I, N))
    E = np.zeros((I, N-1)).astype(np.int32)
    D[:, 0] = np.add(C, B[:, 0])

    # Compute D and E in a nested loop
    for n in range(1, N):
        for i in range(I):
            temp_product = np.add(A[:, i], D[:, n-1])
            D[i, n] = np.max(temp_product) + B[i, O[n]]
            E[i, n-1] = np.argmax(temp_product)

    # Backtracking
    S_opt = np.zeros(N).astype(np.int32)
    S_opt[-1] = np.argmax(D[:, -1])
    for n in range(N-2, 0, -1):
        S_opt[n] = E[int(S_opt[n+1]), n]

    return S_opt, D, E

if __name__ == "__main__":
    # Test:
    # Define model parameters
    A = np.array([[0.6, 0.5, 0.4, 0.0], 
                [0.5, 0.6, 0.5, 0.4], 
                [0.4, 0.5, 0.6, 0.5],
                [0.0, 0.4, 0.5, 0.6]])

    C = np.array([0.6, 0.8, 0.8, 0.6])

    B = np.array([[0.3, 0.6, 0.3], 
                [0.2, 0.4, 0.6], 
                [0.3, 0.5, 0.7],
                [0.3, 0.6, 0.3]])


    O = np.array([2,0,0,0,0,0,0,0]).astype(np.int32)

    # Apply Viterbi algorithm
    S_opt, D, E = viterbi(A, C, B, O)
    #
    print('Observation sequence:   O = ', O)
    print('Optimal state sequence: S = ', S_opt)
    np.set_printoptions(formatter={'float': "{: 7.4f}".format})
    print('D =', D, sep='\n')
    np.set_printoptions(formatter={'float': "{: 7.0f}".format})
    print('E =', E, sep='\n')