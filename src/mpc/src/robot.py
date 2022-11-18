import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are

from pydrake.solvers import mathematicalprogram as mp
from pydrake.solvers.osqp import OsqpSolver
from pydrake.solvers.snopt import SnoptSolver
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve
import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables


class Robot(object):
	def __init__(self, Q, R, Qf):
		self.g = 9.81
		self.m = 1
		self.a = 0.25
		self.I = 0.0625
		self.Q = np.diag([10,10,1]) # Shape based on state
		self.R = np.diag([1,1])
		self.Qf = np.diag([10,10,1])

		# Input limits
		self.umin = np.array([-0.26, ])
		self.umax = np.array([0.26, ])

		self.nx = 3
		self.nu = 2
   
	def x_d(self, t):
		# TODO: Return desired state at time t
		# Required trajectory
		return x_traj(t)

  	def u_d(self, t):
		# TODO: Return desired control input at time t
		# MAY NOT BE NEEDED
		# Based on trajectory
		return u_traj(t)

  	def continuous_time_full_dynamics(self, x, u):
		# Dynamics for the quadrotor
		# TODO: Pranav add dynamics
		g = self.g
		m = self.m
		a = self.a
		I = self.I

		theta = x[2]
		ydot = x[3]
		zdot = x[4]
		thetadot = x[5]
		u0 = u[0]
		u1 = u[1]

		xdot = np.array([ydot,
						zdot,
						thetadot,
						-sin(theta) * (u0 + u1) / m,
						-g + cos(theta) * (u0 + u1) / m,
						a * (u0 - u1) / I])
		return xdot

  	def continuous_time_linearized_dynamics(self):
		# TODO: Pranav add dynamics
		# Dynamics linearized at the fixed point
		# This function returns A and B matrix
		A = np.zeros((6,6))
		A[:3, -3:] = np.identity(3)
		A[3, 2] = -self.g;

		B = np.zeros((6,2))
		B[4,0] = 1/self.m;
		B[4,1] = 1/self.m;
		B[5,0] = self.a/self.I
		B[5,1] = -self.a/self.I

		return A, B

  	def discrete_time_linearized_dynamics(self, T):
		# TODO: Pranav add dynamics
		# CHECK PAPER
		# Discrete time version of the linearized dynamics at the fixed point
		# This function returns A and B matrix of the discrete time dynamics
		A_c, B_c = self.continuous_time_linearized_dynamics()
		A_d = np.identity(6) + A_c * T;
		B_d = B_c * T;

		return A_d, B_d



  ###
	# TODO: MPC 
  ###
  	def add_initial_state_constraint(self, prog, x, x_current):
		# TODO: impose initial state constraint.
		# Use AddBoundingBoxConstraint
		prog.AddBoundingBoxConstraint(x_current, x_current, x[0])

  	def add_input_saturation_constraint(self, prog, x, u, N):
		# TODO: impose input limit constraint.
		# Use AddBoundingBoxConstraint
		# The limits are available through self.umin and self.umax
		for ui in u:
			prog.AddBoundingBoxConstraint(self.umin - self.u_d()[0], self.umax - self.u_d()[0], ui[0])
			prog.AddBoundingBoxConstraint(self.umin - self.u_d()[0], self.umax - self.u_d()[0], ui[1])

  	def add_dynamics_constraint(self, prog, x, u, N, T):
		# TODO: impose dynamics constraint.
		# Use AddLinearEqualityConstraint(expr, value)
		A, B = self.discrete_time_linearized_dynamics(T)
		for i in range(N-1):
			prog.AddLinearEqualityConstraint(A @ x[i,:] + B @ u[i,:] - x[i+1,:], np.zeros(6))

  	def add_cost(self, prog, x, u, N):
		# x = x_e and u = u_e
		# TODO: Might need to change
		for i in range(N-1):
			prog.AddQuadraticCost(x[i,:] @ self.Q @ x[i,:].T)
			prog.AddQuadraticCost(u[i,:] @ self.R @ u[i,:].T)
		prog.AddQuadraticCost(x[i,:] @ self.Qf @ x[i,:])	

  	def compute_mpc_feedback(self, x_current, use_clf=False):
		'''
		This function computes the MPC controller input u
		'''

		# Parameters for the QP
		N = 10
		T = 0.1

		# Initialize mathematical program and decalre decision variables
		prog = MathematicalProgram()
		x = np.zeros((N, self.nx), dtype="object")
		for i in range(N):
		x[i] = prog.NewContinuousVariables(self.nx, "x_" + str(i))
		u = np.zeros((N-1, self.nu), dtype="object")
		for i in range(N-1):
		u[i] = prog.NewContinuousVariables(self.nu, "u_" + str(i))

		# Add constraints and cost
		self.add_initial_state_constraint(prog, x, x_current)
		self.add_input_saturation_constraint(prog, x, u, N)
		self.add_dynamics_constraint(prog, x, u, N, T)
		self.add_cost(prog, x, u, N)

		# Solve the QP
		solver = OsqpSolver()
		result = solver.Solve(prog)

		u_mpc = np.zeros(2) # v_dot and theta_dot
		# TODO: retrieve the controller input from the solution of the optimization problem
		# and use it to compute the MPC input u
		# You should make use of result.GetSolution(decision_var) where decision_var
		# is the variable you want

		u_e = result.GetSolution(u)[0]
		u_mpc = u_e + self.u_d()
		
		return u_mpc

  	def compute_lqr_feedback(self, x):
		'''
		Infinite horizon LQR controller
		'''
		A, B = self.continuous_time_linearized_dynamics()
		S = solve_continuous_are(A, B, self.Q, self.R)
		K = -inv(self.R) @ B.T @ S
		u = self.u_d() + K @ x;
		return u
