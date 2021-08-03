// Author: Keenan Burnett
#include <ros/ros.h>
#include <limits>
#include <iostream>
#include <algorithm>
#include <vector>
#include "utils/association.hpp"

namespace kalman {

std::vector<int> brute_force(Eigen::MatrixXd C, double infeasible_cost) {
    uint M = C.cols();
    uint N = C.rows();
    std::vector<int> assignments;
    if (N == 0 || M == 0)
        return assignments;
    std::vector<int> d(M, 0);
    for (uint j = 0; j < M; j++) {
        d[j] = j;
    }
    // Pad with (-1) if M < N
    if (M < N) {
        for (uint i = 0; i < (N - M); i++) {
            d.insert(d.begin(), -1);
        }
    }
    double min_cost = std::numeric_limits<double>::infinity();
    // Loop over all permutations
    std::vector<int> candidate(N, -1);
    do {
        double cost = 0;
        for (uint i = 0; i < N; i++) {
            if (d[i] < 0) {
                candidate[i] = -1;
                continue;
            }
            cost += C(i, d[i]);
            candidate[i] = d[i];
        }
        if (cost < min_cost) {
            min_cost = cost;
            assignments = candidate;
        }
    } while (std::next_permutation(d.begin(), d.end()));
    // check for infeasible assignments in brute force output:
    for (uint i = 0; i < assignments.size(); i++) {
        if (assignments[i] >= 0) {
            if (C(i, assignments[i]) == infeasible_cost) {
                assignments[i] = -1;
            }
        }
    }
    return assignments;
}

static bool is_independent(uint i, uint j, Eigen::MatrixXd &zeros) {
    for (uint n = 0; n < zeros.rows(); n++) {
        if (n == i)
            continue;
        if (zeros(n, j) == 1)
            return false;
    }
    for (uint m = 0; m < zeros.cols(); m++) {
        if (m == j)
            continue;
        if (zeros(i, m) == 1)
            return false;
    }
    return true;
}

static std::vector<int> assoc_from_zeros(Eigen::MatrixXd &zeros, Eigen::MatrixXd &cost, double infeasible_cost) {
    std::vector<int> assignments(zeros.rows(), -1);
    for (uint i = 0; i < zeros.rows(); i++) {
        for (uint j = 0; j < zeros.cols(); j++) {
            if (zeros(i, j) == 1) {
                if (cost(i, j) == infeasible_cost)
                    assignments[i] = -1;
                else
                    assignments[i] = j;
                break;
            }
        }
    }
    return assignments;
}

static bool find_uncovered_zero(Eigen::MatrixXd &C, std::vector<int> &row_cover, std::vector<int> &col_cover,
    uint &row, uint&col) {
    for (uint i = 0; i < C.rows(); i++) {
        for (uint j = 0; j < C.cols(); j++) {
            if (C(i, j) == 0 && row_cover[i] == 0 && col_cover[j] == 0) {
                row = i;
                col = j;
                return true;
            }
        }
    }
    return false;
}

static double get_smallest_uncovered(Eigen::MatrixXd &C, std::vector<int> &row_cover, std::vector<int> &col_cover) {
    double INF = std::numeric_limits<double>::infinity();
    double smallest = INF;
    for (uint i = 0; i < C.rows(); i++) {
        for (uint j = 0; j < C.cols(); j++) {
            if (C(i, j) < smallest && row_cover[i] == 0 && col_cover[j] == 0)
                smallest = C(i, j);
        }
    }
    if (smallest == INF)
        return 0;
    else
        return smallest;
}

static bool get_starred_zero_in_column(Eigen::MatrixXd &zeros, uint &row, uint col) {
    for (uint i = 0; i < zeros.rows(); i++) {
        if (i == row)
            continue;
        if (zeros(i, col) == 1) {
            row = i;
            return true;
        }
    }
    return false;
}

static bool get_primed_zero_in_row(Eigen::MatrixXd &zeros, uint row, uint &col) {
    for (uint j = 0; j < zeros.cols(); j++) {
        if (j == col)
            continue;
        if (zeros(row, j) == 2) {
            col = j;
            return true;
        }
    }
    return false;
}

static void erase_all_primes(Eigen::MatrixXd &zeros) {
    for (uint i = 0; i < zeros.rows(); i++) {
        for (uint j = 0; j < zeros.cols(); j++) {
            if (zeros(i, j) == 2)
                zeros(i, j) = 0;
        }
    }
}

static void uncover_all_lines(std::vector<int> &row_cover, std::vector<int> &col_cover) {
    for (uint i = 0; i < row_cover.size(); i++) {
        row_cover[i] = 0;
    }
    for (uint j = 0; j < col_cover.size(); j++) {
        col_cover[j] = 0;
    }
}

static bool check_for_nan(Eigen::MatrixXd C) {
    for (uint i = 0; i < C.rows(); i++) {
        for (uint j = 0; j < C.cols(); j++) {
            if (std::isnan(C(i, j)))
                return true;
        }
    }
    return false;
}

std::vector<int> hungarian(Eigen::MatrixXd C, double infeasible_cost) {
    // Preliminaries
    uint N = C.rows();
    uint M = C.cols();
    uint k = std::min(N, M);
    Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(N, M);
    std::vector<int> row_cover(N, 0);
    std::vector<int> col_cover(M, 0);
    std::vector<int> temp(N, -1);
    if (N == 0 || M == 0)
        return temp;
    if (N > M)
        goto STEP1;
    // Subtract the smallest value in each row from each element in that row
    for (uint i = 0; i < N; i++) {
        double minval = infeasible_cost;
        for (uint j = 0; j < M; j++) {
            if (C(i, j) < minval)
                minval = C(i, j);
        }
        if (minval < infeasible_cost) {
            for (uint j = 0; j < M; j++) {
                C(i, j) -= minval;
            }
        }
    }
    STEP1: {
        k = std::min(N, M);
        zeros = Eigen::MatrixXd::Zero(N, M);
        for (uint i = 0; i < N; i++) {
            for (uint j = 0; j < M; j++) {
                if (C(i, j) == 0 && is_independent(i, j, zeros))
                    zeros(i, j) = 1;    // value of starred zero
            }
        }
        row_cover = std::vector<int>(N, 0);
        col_cover = std::vector<int>(M, 0);
    }
    STEP2: {
        uint cover_count = 0;
        for (uint j = 0; j < M; j++) {
            bool cover = false;
            for (uint i = 0; i < N; i++) {
                if (zeros(i, j) == 1) {
                    cover = true;
                    break;
                }
            }
            if (cover) {
                col_cover[j] = 1;
                cover_count++;
            }
        }
        // If k columns are covered, the starred zeros for the desired independent set, exit.
        if (cover_count == k)
            return assoc_from_zeros(zeros, C, infeasible_cost);
    }
    // Step 3: Choose a noncovered zero and prime it. If there is no starred zero in this row, got to Step 4.
    // If there is a starred zero (Z) in this row, cover this row, uncover the column of Z.
    // Repeat until all zeros are covered. Go to Step 5.
    uint row = 0, col = 0;
    STEP3: {
        while (find_uncovered_zero(C, row_cover, col_cover, row, col) == true) {
            zeros(row, col) = 2;    // Prime the uncovered zero
            // Search row for starred zero
            int starred_zero_column = -1;
            for (uint j = 0; j < M; j++) {
                if (zeros(row, j) == 1) {
                    starred_zero_column = j;
                    break;
                }
            }
            if (starred_zero_column < 0)
                goto STEP4;
            col_cover[starred_zero_column] = 0;  // uncover the the starred zero's column
            row_cover[row] = 1;  // cover this row
        }
    goto STEP5;
    }
    // There is a sequence of alternating starred and primed zeros
    // Unstar the starred zeros of the sequence and star the primed zeros of the sequence
    STEP4: {
        bool dostuff = true;
        zeros(row, col) = 1;
        while (dostuff == true) {
            dostuff &= get_starred_zero_in_column(zeros, row, col);
            if (dostuff)
                zeros(row, col) = 0;  // unstar each starred zero of the sequence
            else
                break;
            dostuff &= get_primed_zero_in_row(zeros, row, col);
            if (dostuff)
                zeros(row, col) = 1;  // star each primed zero of the sequence
        }
        erase_all_primes(zeros);
        uncover_all_lines(row_cover, col_cover);
        goto STEP2;
    }
    STEP5: {
        double h = get_smallest_uncovered(C, row_cover, col_cover);
        if (h == 0)
            return assoc_from_zeros(zeros, C, infeasible_cost);
        for (uint i = 0; i < N; i++) {
            if (row_cover[i]) {
                for (uint j = 0; j < M; j++) {
                    C(i, j) += h;  // add h to each covered row
                }
            }
        }
        for (uint j = 0; j < M; j++) {
            if (!col_cover[j]) {
                for (uint i = 0; i < N; i++) {
                    C(i, j) -= h;  // subtract h from each uncovered column
                }
            }
        }
        if (check_for_nan(C)) {
            std::cout << "ERROR: NAN values found in the cost matrix, returning (-1)s for assignment" << std::endl;
            return temp;
        }
        goto STEP3;
    }
    return temp;
}

static bool is_member(std::vector<int> v, int x) {
    return std::find(v.begin(), v.end(), x) != v.end();
}

std::vector<int> greedy(Eigen::MatrixXd C, double infeasible_cost) {
    std::vector<int> assignments(C.rows(), -1);
    for (uint i = 0; i < C.rows(); i++) {
        double min_cost = infeasible_cost;
        for (uint j = 0; j < C.cols(); j++) {
            if (C(i, j) < min_cost && !is_member(assignments, j)) {
                assignments[i] = j;
                min_cost = C(i, j);
            }
        }
    }
    return assignments;
}

std::vector<int> association(Eigen::MatrixXd C, int type, double infeasible_cost) {
    if (type == 0) {
        return hungarian(C, infeasible_cost);
    } else if (type == 1) {
        return greedy(C, infeasible_cost);
    } else {
        std::cout << "WARNING: association type not recognized: " << type << std::endl;
        std::vector<int> temp(C.rows(), -1);
        return temp;
    }
}
}  // namespace kalman
