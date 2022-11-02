/*
 * Copyright 2022 Santiago Previotto.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
/** \file signal.h */
#ifndef SRC_INCLUDE_SIGNAL_H_
#define SRC_INCLUDE_SIGNAL_H_

#define N_COS         17    /**< \brief Samples for cosine. */
#define N_TRIANG      20    /**< \brief Samples for triangle. */

float signal_triangle[N_TRIANG] = { 0.50, 0.55, 0.60, 0.65, 0.70,
                                    0.75, 0.80, 0.85, 0.90, 0.95,
                                    1.00, 0.95, 0.90, 0.85, 0.80,
                                    0.75, 0.70, 0.65, 0.60, 0.55 };

float signal_cos[N_COS] = { 2.000, 1.924, 1.707, 1.383, 1.000, 0.617,
                            0.293, 0.075, 0.000, 0.076, 0.293, 0.617,
                            1.000, 1.383, 1.707, 1.924, 2.000 };

#endif /* SRC_INCLUDE_SIGNAL_H_ */
