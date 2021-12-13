/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package CoopFormationControlSimulation;

import java.util.Arrays;

import org.apache.commons.math3.exception.DimensionMismatchException;
import org.apache.commons.math3.exception.MaxCountExceededException;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;

public class Def_Track implements FirstOrderDifferentialEquations {

	int N;
	int m;
	RealMatrix Ai;
	RealMatrix Bi;
	RealMatrix Pi;
	RealMatrix Fi;
	double epsilon;
	RealMatrix Lg;
	RealMatrix neBiPi;
	RealMatrix nFL;

	public Def_Track(int N, int m, RealMatrix Ai, RealMatrix Bi, RealMatrix Pi,
			RealMatrix Fi, double epsilon, RealMatrix Lg) {
		this.N = N;
		this.m = m;
		this.Ai = Ai;
		this.Bi = Bi;
		this.Pi = Pi;
		this.Fi = Fi;
		this.epsilon = epsilon;
		this.Lg = Lg;
		this.neBiPi = Bi.transpose().scalarMultiply(-epsilon).multiply(Pi);
		RealMatrix F = Kron(MatrixUtils.createRealIdentityMatrix(N), Fi);
		RealMatrix L = Kron(Lg, MatrixUtils.createRealIdentityMatrix(m));
		this.nFL = F.scalarMultiply(-1).multiply(L);
	}

	@Override
	public void computeDerivatives(double t, double[] x, double[] xDot)
			throws MaxCountExceededException, DimensionMismatchException {

		// An expanded system: x(1)~x(8) are errors, x(9)~x(10) are reference
		// trajectory
		double[] x_veh_arr = Arrays.copyOfRange(x, 0, this.getDimension());
		RealMatrix x_veh = MatrixUtils.createRealMatrix(this.getDimension(), 1);
		x_veh.setColumn(0, x_veh_arr);

		// group coordination control law
		double[] u_coop = this.nFL.multiply(x_veh).getColumn(0);

		double[] u = new double[u_coop.length];

		for (int i = 0; i < 2 * N; i += 2) {

			RealMatrix xi = MatrixUtils.createRealMatrix(new double[][] {
					{ x[i] }, { x[i + 1] } });

			double[] ui = neBiPi.multiply(xi).getColumn(0);// stabalizing
															// control law

			// u = stabalizing control law + group coordination control law
			u[i] = u_coop[i] + ui[0];// ui1
			u[i + 1] = u_coop[i + 1] + ui[1];// ui2
			xDot[i] = x[i + 1] + u[i];// xi1 = xi2+ui1
			xDot[i + 1] = u[i + 1];// xi2 = ui2
		}
	}

	@Override
	public int getDimension() {
		return N * m;
	}

	public static RealMatrix Kron(RealMatrix a, RealMatrix b) {
		double[][] kroneckerProduct = kronProduct(a.getData(), b.getData());
		return MatrixUtils.createRealMatrix(kroneckerProduct);
	}

	private static double[][] kronProduct(double[][] A, double[][] B) {
		final int m = A.length;
		final int n = A[0].length;
		final int p = B.length;
		final int q = B[0].length;

		double[][] out = new double[m * p][n * q];
		kronProduct(A, B, out);
		return out;
	}

	private static void kronProduct(double[][] A, double[][] B, double[][] out) {
		final int m = A.length;
		final int n = A[0].length;
		final int p = B.length;
		final int q = B[0].length;

		if (out == null || out.length != m * p || out[0].length != n * q) {
			throw new RuntimeException("Wrong dimensions in Kronecker product");
		}

		for (int i = 0; i < m; i++) {
			final int iOffset = i * p;
			for (int j = 0; j < n; j++) {
				final int jOffset = j * q;
				final double aij = A[i][j];

				for (int k = 0; k < p; k++) {
					for (int l = 0; l < q; l++) {
						out[iOffset + k][jOffset + l] = aij * B[k][l];
					}
				}

			}
		}
	}
}
