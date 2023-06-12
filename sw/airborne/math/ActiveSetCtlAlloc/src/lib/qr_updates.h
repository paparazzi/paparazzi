/**
 * Copyright (C) Till Blaha 2022-2023
 * MAVLab -- Faculty of Aerospace Engineering -- Delft University of Techology
 */

/**
 * @file qr_updates.h
 * 
 * @brief QR update function definition
*/

#ifndef QR_UPDATES_H
#define QR_UPDATES_H

#include <solveActiveSet.h>

/** 
 * @brief Update QR due to a partial unit circ-shift of some columns of A = QR
 *
 * Efficiently computes QR due to A[:, i:j] <- circshift(A[:, i:j], 1)
 * 
 * Rightshift if j > i, lefthsift if i > i
 * 
 * Very close to the original algorithm in LINPACK, by Dongarra et. al. 1971
 * Requires explicit Q and R factors, and not the (more common) Householder 
 * factor representation of Q
 * 
 * @param n number of rows of A and R
 * @param p number of columns of A and R
 * @param Q_ptr Pointer to the rows of Q
 * @param R_ptr Pointer to the rows of R (not very space efficient)
 * @param i 0-based index of the first column to be shifted
 * @param j 0-based index of the last column to be shifted
 * 
 */
void qr_shift ( int n, int p, num_t** Q_ptr, num_t** R_ptr, int i, int j);

/**
 * @brief Compute Givens plane rotation
 * 
 * https://en.wikipedia.org/wiki/Givens_rotation
 * Computes scalars G_0, G_1, G_2, G_3 such that:
 * 
 * [ 1 ... 0     0    ... 0 ] [z_1]    [z_1]
 * [ ...   ...   ...    ... ] [.]      [.]
 * [ 0 ... G[0]  G[2] ... 0 ] [a]      [r]
 * [ 0 ... G[1]  G[3] ... 0 ] [b]  =   [0]
 * [ ...   ...   ...    ... ] [.]      [.]
 * [ 0 ... 0     0    ... 1 ] [z_n]    [z_n]
 * 
 *      square matrix        vector    vector
 * 
 * @param a Top entry in the vector
 * @param b Bottom entry in the vector
 * @param G On exit: holds the 4 entries describing the Givens rotation
*/
void givens(num_t a, num_t b, num_t G[4]);

/**
 * @brief Sparse computation of A <- G * A, where G is adjacent Givens
 * 
 * An adjacent Givens rotation has all non-zero/non-unit elements adjacent to 
 * each other:
 * 
 *      [ 1 ... 0     0    ... 0 ] [z_1]    [z_1]
 *      [ ...   ...   ...    ... ] [.]      [.]
 * row1 [ 0 ... G[0]  G[2] ... 0 ] [a]      [r]
 * row2 [ 0 ... G[1]  G[3] ... 0 ] [b]  =   [0]
 *      [ ...   ...   ...    ... ] [.]      [.]
 *      [ 0 ... 0     0    ... 1 ] [z_n]    [z_n]
 * 
 *             square matrix     vector    vector 
 * 
 * @param p Number of columns in A
 * @param A On exit: pointer to rows of result of G*A
 * @param G Representation of Givens rotation
 * @param row1 Row in square matrix containing Givens parameters G[0] and G[2]
 * @param row2 Row in square matrix containing Givens parameters G[1] and G[3]
*/
void givens_left_apply(int p, num_t** A, num_t* G, int row1, int row2);

/**
 * @brief Sparse computation of A <- A * G, where G is adjacent Givens
 * 
 * An adjacent Givens rotation has all non-zero/non-unit elements adjacent to 
 * each other:
 *
 *         col1  col2
 *
 * [ 1 ... 0     0    ... 0 ] [z_1]    [z_1]
 * [ ...   ...   ...    ... ] [.]      [.]
 * [ 0 ... G[0]  G[2] ... 0 ] [a]      [r]
 * [ 0 ... G[1]  G[3] ... 0 ] [b]  =   [0]
 * [ ...   ...   ...    ... ] [.]      [.]
 * [ 0 ... 0     0    ... 1 ] [z_n]    [z_n]
 * 
 *        square matrix     vector    vector 
 * 
 * @param n Number of rows in A
 * @param A On exit: pointer to rows of result of A*G
 * @param G Representation of Givens rotation
 * @param col1 Column in square matrix containing Givens parameters G[0] and G[1]
 * @param col2 Column in square matrix containing Givens parameters G[2] and G[3]
*/
void givens_right_apply(int n, num_t** A, num_t* G, int col1, int col2);

#endif